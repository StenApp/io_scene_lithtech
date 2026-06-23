# -*- coding: utf-8 -*-
"""
reader_lta.py  --  Parse LithTech .lta (text) into the unified abc.py:Model in
RAW LithTech space, so it flows through the SAME builder_import + swap_* as the
binary readers. Parser and tree-walking logic are adapted from the LTA-Jupiter
(v1) importer -- we reuse that hard-won format knowledge instead of reinventing
it; only the coordinate conversion is removed (the shared builder owns that).

Produces: nodes (world bind_matrix), pieces (single LOD, indexed geometry
reconciled into Vertex+Face), per-vertex weights, sockets, animations.

NOTE ON TIME UNITS: LTA animation 'times' are MILLISECONDS (confirmed from the
v1 importer, which computes frames = (t - t0) * fps / 1000). The binary Model
uses ms too, so times are stored unchanged.
"""

import os
import re

from mathutils import Vector, Matrix, Quaternion

try:
    from .abc import (Model, Node, Piece, LOD, Vertex, Face, FaceVertex,
                      Weight, Socket, Animation, AnimBinding)
except ImportError:
    from abc import (Model, Node, Piece, LOD, Vertex, Face, FaceVertex,
                     Weight, Socket, Animation, AnimBinding)


_TIME_TO_MS = 1.0  # LTA 'times' are already milliseconds (confirmed from v1:
#                    frames = (t - t0) * fps / 1000.0 treats t as ms). The
#                    binary Model also uses ms, so we store them unchanged.


# ---------------------------------------------------------------------------
# LTA S-expression parser + helpers (from v1, verbatim behaviour)
# ---------------------------------------------------------------------------
_TOKEN_RE = re.compile(r'\(|\)|"[^"]*"|[^\s()"]+')


class LTAParseError(Exception):
    pass


def parse_lta(text):
    root, stack = [], None
    root = []
    stack = [root]
    for tok in _TOKEN_RE.findall(text):
        if tok == '(':
            new = []
            stack[-1].append(new)
            stack.append(new)
        elif tok == ')':
            if len(stack) == 1:
                raise LTAParseError("Unbalanced ')'")
            stack.pop()
        else:
            if tok.startswith('"'):
                tok = tok[1:-1]
            stack[-1].append(tok)
    if len(stack) != 1:
        raise LTAParseError("Unbalanced '(' (truncated?)")
    return root


def parse_lta_spans(text):
    """Like parse_lta but also returns {id(list_node): (start, end)} mapping
    each parsed list to its exact character span in the source. Used to
    re-emit preserved on-load-cmd blocks VERBATIM (quotes and all) instead of
    re-serializing the parsed tree, which loses string-quote information and
    would corrupt names containing spaces (e.g. "Upper Instant") or any
    quoted piece name in piece-priorities."""
    root = []
    stack = [root]
    starts = [None]
    spans = {}
    for m in _TOKEN_RE.finditer(text):
        tok = m.group(0)
        if tok == '(':
            new = []
            stack[-1].append(new)
            stack.append(new)
            starts.append(m.start())
        elif tok == ')':
            if len(stack) == 1:
                raise LTAParseError("Unbalanced ')'")
            node = stack.pop()
            st = starts.pop()
            spans[id(node)] = (st, m.end())
        else:
            if tok.startswith('"'):
                tok = tok[1:-1]
            stack[-1].append(tok)
    if len(stack) != 1:
        raise LTAParseError("Unbalanced '(' (truncated?)")
    return root, spans


def node_name(n):
    if isinstance(n, list) and n and isinstance(n[0], str):
        return n[0]
    return None


def shallow_find(node, name):
    if node is None:
        return None
    for c in node:
        if isinstance(c, list) and node_name(c) == name:
            return c
    return None


def shallow_find_all(node, name):
    return [c for c in node if isinstance(c, list) and node_name(c) == name]


def find_all(node, name, out=None):
    if out is None:
        out = []
    if isinstance(node, list):
        if node_name(node) == name:
            out.append(node)
        for c in node:
            if isinstance(c, list):
                find_all(c, name, out)
    return out


def lists_of(node):
    return [c for c in node if isinstance(c, list)]


def atoms_of(node):
    return [c for c in node if isinstance(c, str)]


def first_string(node, skip=1):
    for c in node[skip:]:
        if isinstance(c, str):
            return c
        break
    return None


def floats(seq):
    return [float(x) for x in seq if isinstance(x, str)]


def vector_list(node):
    if node is None:
        return []
    subs = lists_of(node)
    if len(subs) == 1 and lists_of(subs[0]) and not node_name(subs[0]):
        subs = lists_of(subs[0])
    return [floats(s) for s in subs if any(isinstance(c, str) for c in s)]


def flat_ints(node):
    if node is None:
        return []
    out = [int(float(a)) for a in atoms_of(node)[1:]]
    for sub in lists_of(node):
        out.extend(int(float(a)) for a in atoms_of(sub))
        for sub2 in lists_of(sub):
            out.extend(int(float(a)) for a in atoms_of(sub2))
    return out


def _serialize_node(node, indent=0):
    """Serialize a parsed S-expression node back to LTA text (for re-emitting
    preserved on-load-cmd blocks verbatim-ish on export)."""
    pad = '\t' * indent
    if isinstance(node, str):
        return pad + node
    atoms = [x for x in node if isinstance(x, str)]
    kids = [x for x in node if isinstance(x, list)]
    head = ' '.join(atoms)
    if not kids:
        return '%s(%s)' % (pad, head)
    lines = ['%s(%s' % (pad, head)]
    for k in kids:
        lines.append(_serialize_node(k, indent + 1))
    lines.append('%s)' % pad)
    return '\n'.join(lines)


def _quat_xyzw(vals):
    """LTA stores quaternions x y z w -> mathutils Quaternion(w, x, y, z)."""
    if len(vals) >= 4:
        return Quaternion((vals[3], vals[0], vals[1], vals[2]))
    return Quaternion()


# ---------------------------------------------------------------------------
class LTAModelReader(object):
    def from_file(self, path):
        with open(path, 'r', errors='replace') as f:
            src_text = f.read()
        tree, spans = parse_lta_spans(src_text)

        model = Model()
        model.name = os.path.splitext(os.path.basename(path))[0]
        model.version = 0  # LTA carries no binary version

        self._tree = tree
        self._spans = spans
        self._src_text = src_text
        self._world = {}          # node name -> world Matrix (raw LT)
        self._name_to_index = {}  # node name -> global index

        # model-level coord-frame-type sets the default for all transforms
        # (default GLOBAL; only explicit 'local' composes the hierarchy)
        self._frames_local = False
        root = tree[0] if (tree and isinstance(tree[0], list)) else tree
        mcft = shallow_find(root, 'coord-frame-type')
        if mcft and 'local' in atoms_of(mcft)[1:]:
            self._frames_local = True

        self._read_nodes(model)
        self._read_pieces(model)
        self._read_weights(model)
        self._read_sockets(model)
        self._read_animations(model)
        self._read_metadata(model)
        return model

    # -- on-load-cmds metadata ---------------------------------------------
    def _read_metadata(self, model):
        """Read on-load-cmds so the LTA Model carries the same metadata the
        binary readers produce: anim-bindings (dims/translation/interp/weight),
        set-node-flags, set-global-radius. Also capture weightsets / childmodels
        / set-repl-lod-original / node-obb as RAW text for loss-light round-trip.
        Mirrors the v1 importer's read_on_load_cmds."""
        by_name = {n.name: n for n in model.nodes}
        anim_by_name = {a.name: a for a in model.animations}

        cmds = []
        for olc in find_all(self._tree, 'on-load-cmds'):
            for sub in lists_of(olc):
                if node_name(sub):
                    cmds.append(sub)
                else:
                    cmds.extend(lists_of(sub))

        raw = {'lta_weightsets': [], 'lta_childmodels': [],
               'lta_lod': [], 'lta_obb': []}

        def _verbatim(cmd):
            sp = self._spans.get(id(cmd))
            if sp:
                return self._src_text[sp[0]:sp[1]]
            return _serialize_node(cmd)   # fallback (shouldn't happen)

        for cmd in cmds:
            kind = node_name(cmd)
            if kind == 'set-global-radius':
                vals = atoms_of(cmd)[1:]
                if vals:
                    try:
                        model.internal_radius = float(vals[0])
                    except ValueError:
                        pass
            elif kind == 'set-node-flags':
                for sub in lists_of(cmd):
                    entries = lists_of(sub) or [sub]
                    for e in entries:
                        a = atoms_of(e)
                        if len(a) >= 2 and a[0] in by_name:
                            try:
                                by_name[a[0]].flags = int(float(a[1]))
                            except ValueError:
                                pass
            elif kind == 'anim-bindings':
                for b in find_all(cmd, 'anim-binding'):
                    nm = shallow_find(b, 'name')
                    if not nm:
                        continue
                    bname = first_string(nm)
                    binding = AnimBinding()
                    binding.name = bname
                    for key, attr in (('dims', 'extents'), ('translation', 'origin')):
                        kn = shallow_find(b, key)
                        if kn:
                            vals = floats(atoms_of(kn)[1:])
                            if not vals and lists_of(kn):
                                vals = floats(lists_of(kn)[0])
                            if len(vals) >= 3:
                                setattr(binding, attr, Vector(vals[:3]))
                    it = shallow_find(b, 'interp-time')
                    if it and atoms_of(it)[1:]:
                        try:
                            binding.interp_time = float(atoms_of(it)[1])
                        except ValueError:
                            pass
                    ws = shallow_find(b, 'weight-set')
                    if ws:
                        wsv = first_string(ws)
                        if wsv:
                            binding.weight_set = wsv
                    model.anim_bindings.append(binding)
                    # also push interp onto the matching Animation
                    a = anim_by_name.get(bname)
                    if a is not None and hasattr(binding, 'interp_time'):
                        a.interpolation_time = binding.interp_time
            elif kind in ('add-anim-weightsets', 'anim-weightsets'):
                raw['lta_weightsets'].append(_verbatim(cmd))
            elif kind in ('add-childmodels', 'child-model'):
                raw['lta_childmodels'].append(_verbatim(cmd))
            elif kind == 'set-repl-lod-original':
                raw['lta_lod'].append(_verbatim(cmd))
            elif kind in ('add-node-obb-list', 'add-node-obb'):
                raw['lta_obb'].append(_verbatim(cmd))

        model.preserved_raw = {k: '\n'.join(v) for k, v in raw.items() if v}

    # -- skeleton -----------------------------------------------------------
    def _read_nodes(self, model):
        hierarchy = shallow_find(self._tree, 'hierarchy')
        if hierarchy is None:
            hs = find_all(self._tree, 'hierarchy')
            hierarchy = hs[0] if hs else self._tree
        children = shallow_find(hierarchy, 'children') or hierarchy
        for sub in lists_of(children):
            if node_name(sub) == 'transform':
                self._read_transform(model, sub, None, Matrix.Identity(4))
            else:
                for t in lists_of(sub):
                    if node_name(t) == 'transform':
                        self._read_transform(model, t, None, Matrix.Identity(4))
        if not model.nodes:
            raise LTAParseError("LTA hierarchy contains no transforms.")
        # resolve parent/children as Node REFERENCES -- builder_import and
        # animation_import expect node.parent (Node|None) and node.children
        # (list[Node]), the same shape build_undirected_tree gives binary models.
        by_name = {nd.name: nd for nd in model.nodes}
        for nd in model.nodes:
            nd.children = []
        for nd in model.nodes:
            pname = getattr(nd, '_parent_name', None)
            nd.parent = by_name.get(pname) if pname else None
            if nd.parent is not None:
                nd.parent.children.append(nd)
        for nd in model.nodes:
            nd.child_count = len(nd.children)

    def _read_transform(self, model, tnode, parent_name, parent_world):
        name = first_string(tnode) or "node%d" % len(model.nodes)
        base, k = name, 1
        while name in self._name_to_index:
            name = "%s.%03d" % (base, k); k += 1

        m = Matrix.Identity(4)
        mnode = shallow_find(tnode, 'matrix')
        if mnode:
            rows = lists_of(mnode)
            if rows and all(isinstance(c, str) for c in rows[0]):
                vals = [floats(r) for r in rows[:4]]
            else:
                vals = [floats(r) for r in lists_of(rows[0])[:4]] if rows else []
            if len(vals) == 4 and all(len(r) == 4 for r in vals):
                m = Matrix(vals)

        # LithTech LTA node matrices are GLOBAL (already world-space) by
        # default -- verified: with global frames the LTA skeleton matches the
        # ABC skeleton exactly (mean node distance 0.00 vs 12.4 when composed).
        # Only an explicit coord-frame-type 'local' switches to composition.
        # (Matches the v1 importer, which defaults global_frames=True.)
        is_local = getattr(self, '_frames_local', False)
        cft = shallow_find(tnode, 'coord-frame-type')
        if cft:
            flags = atoms_of(cft)[1:]
            if 'local' in flags:
                is_local = True
            elif 'global' in flags:
                is_local = False
        world = (parent_world @ m) if is_local else m

        node = Node()
        node.name = name
        node.index = len(model.nodes)
        node.bind_matrix = world.copy()
        node._parent_name = parent_name
        model.nodes.append(node)
        self._world[name] = world
        self._name_to_index[name] = node.index

        kids = shallow_find(tnode, 'children')
        if kids:
            for sub in lists_of(kids):
                if node_name(sub) == 'transform':
                    self._read_transform(model, sub, name, world)
                else:
                    for t in lists_of(sub):
                        if node_name(t) == 'transform':
                            self._read_transform(model, t, name, world)

    def _local_of(self, node):
        p = getattr(node, '_parent_name', None)
        if p and p in self._world:
            return self._world[p].inverted_safe() @ self._world[node.name]
        return self._world[node.name].copy()

    # -- geometry -----------------------------------------------------------
    def _read_pieces(self, model):
        self._shape_to_piece = {}
        for snode in find_all(self._tree, 'shape'):
            piece = self._read_shape(snode, len(model.pieces))
            if piece is not None:
                model.pieces.append(piece)

    def _read_shape(self, snode, idx):
        name = first_string(snode) or "shape%d" % idx
        geometry = shallow_find(snode, 'geometry')
        mesh = shallow_find(geometry, 'mesh') if geometry else None
        if mesh is None:
            return None

        positions = [v[:3] for v in vector_list(shallow_find(mesh, 'vertex')) if len(v) >= 3]
        tri_fs = flat_ints(shallow_find(mesh, 'tri-fs'))
        uvs = [v[:2] for v in vector_list(shallow_find(mesh, 'uvs')) if len(v) >= 2]
        tex_fs = flat_ints(shallow_find(mesh, 'tex-fs'))
        normals = [v[:3] for v in vector_list(shallow_find(mesh, 'normals')) if len(v) >= 3]
        nrm_fs = flat_ints(shallow_find(mesh, 'nrm-fs'))

        if not positions or not tri_fs:
            return None

        piece = Piece()
        piece.name = name
        # texture index varies by LTA dialect:
        #   Jupiter : shape > texture-indices (plural, directly under shape)
        #   LT2.2   : shape > appearance > pc-mat|material|ps2-mat > texture-index
        ti = shallow_find(snode, 'texture-indices')
        if ti:
            vals = [int(float(a)) for a in atoms_of(ti)[1:]]
            if not vals and lists_of(ti):
                vals = [int(float(a)) for a in atoms_of(lists_of(ti)[0])]
            if vals:
                piece.material_index = vals[0]
        else:
            app = shallow_find(snode, 'appearance')
            if app:
                matnode = (shallow_find(app, 'pc-mat') or shallow_find(app, 'material')
                           or shallow_find(app, 'ps2-mat'))
                if matnode:
                    tix = shallow_find(matnode, 'texture-index')
                    if tix and atoms_of(tix)[1:]:
                        piece.material_index = int(float(atoms_of(tix)[1]))

        lod = LOD()
        # normals are normally PER-VERTEX (one per vertex, parallel to the
        # vertex array; no nrm-fs in LT2.2 or Jupiter). nrm-fs (per-corner) is
        # only used as a fallback when present and counts don't line up.
        per_vertex_normals = (len(normals) == len(positions) and len(normals) > 0)
        verts = []
        for i, p in enumerate(positions):
            vtx = Vertex()
            vtx.location = Vector(p)
            vtx.normal = Vector(normals[i]) if per_vertex_normals else Vector((0.0, 0.0, 1.0))
            vtx.weights = []
            verts.append(vtx)

        ntri = len(tri_fs) // 3
        for t in range(ntri):
            face = Face()
            fvs = []
            for kk in range(3):
                ci = t * 3 + kk
                pidx = tri_fs[ci]
                fv = FaceVertex()
                if tex_fs and ci < len(tex_fs) and tex_fs[ci] < len(uvs):
                    u, v = uvs[tex_fs[ci]]
                    fv.texcoord = Vector((u, v))   # RAW; builder applies V-flip
                fv.vertex_index = pidx
                fvs.append(fv)
                # per-corner normal override only when not parallel (rare nrm-fs)
                if not per_vertex_normals and nrm_fs and ci < len(nrm_fs) and nrm_fs[ci] < len(normals):
                    verts[pidx].normal = Vector(normals[nrm_fs[ci]])
            face.vertices = fvs
            lod.faces.append(face)

        lod.vertices = verts
        lod.vert_count = len(verts)
        lod.face_count = len(lod.faces)
        piece.lods = [lod]
        self._shape_to_piece[name] = piece
        return piece

    # -- weights ------------------------------------------------------------
    def _read_weights(self, model):
        for deformer in find_all(self._tree, 'skel-deformer'):
            tnode = shallow_find(deformer, 'target')
            target = first_string(tnode) if tnode else None
            piece = self._shape_to_piece.get(target)
            if piece is None or not piece.lods:
                continue

            inode = shallow_find(deformer, 'influences')
            influences = []
            if inode:
                influences = atoms_of(inode)[1:]
                if not influences and lists_of(inode):
                    influences = atoms_of(lists_of(inode)[0])

            wnode = shallow_find(deformer, 'weightsets')
            if not wnode:
                continue
            entries = lists_of(wnode)
            if len(entries) == 1 and lists_of(entries[0]):
                entries = lists_of(entries[0])

            verts = piece.lods[0].vertices
            for vi, e in enumerate(entries):
                if vi >= len(verts):
                    break
                a = floats(atoms_of(e))
                for i in range(0, len(a) - 1, 2):
                    infl = int(a[i])
                    bias = a[i + 1]
                    if 0 <= infl < len(influences):
                        gname = influences[infl]
                        gidx = self._name_to_index.get(gname)
                        if gidx is not None:
                            w = Weight()
                            w.node_index = gidx
                            w.bias = bias
                            verts[vi].weights.append(w)

    # -- sockets ------------------------------------------------------------
    def _read_sockets(self, model):
        for sock in find_all(self._tree, 'socket'):
            s = Socket()
            s.name = first_string(sock) or "socket"
            pnode = shallow_find(sock, 'parent')
            pname = first_string(pnode) if pnode else None
            s.node_index = self._name_to_index.get(pname, 0)
            posn = shallow_find(sock, 'pos')
            if posn:
                vals = floats(atoms_of(posn)[1:]) or \
                    (floats(lists_of(posn)[0]) if lists_of(posn) else [])
                if len(vals) >= 3:
                    s.location = Vector(vals[:3])
            qn = shallow_find(sock, 'quat')
            if qn:
                vals = floats(atoms_of(qn)[1:]) or \
                    (floats(lists_of(qn)[0]) if lists_of(qn) else [])
                if len(vals) >= 4:
                    s.rotation = _quat_xyzw(vals)
            model.sockets.append(s)

    # -- animations ---------------------------------------------------------
    def _read_animations(self, model):
        all_anim_nodes = {}
        for a in find_all(self._tree, 'anim'):
            ident = first_string(a)
            if ident and shallow_find(a, 'frames'):
                all_anim_nodes.setdefault(ident, a)

        for aset in find_all(self._tree, 'animset'):
            name = first_string(aset) or "anim%d" % len(model.animations)
            times = self._anim_times(aset)
            node_tracks = {}

            anims = shallow_find(aset, 'anims')
            members = []
            if anims:
                for sub in lists_of(anims):
                    if node_name(sub) == 'anim':
                        members.append(sub)
                    else:
                        for a2 in lists_of(sub):
                            if node_name(a2) == 'anim':
                                members.append(a2)
                for ident in atoms_of(anims)[1:]:
                    if ident in all_anim_nodes:
                        members.append(all_anim_nodes[ident])

            for anode in members:
                self._read_anim_track(anode, node_tracks, times)

            if not times and not node_tracks:
                continue
            if not times:
                # derive a length from the longest track
                longest = max((len(t) for t in node_tracks.values()), default=0)
                times = [i for i in range(longest)]

            self._build_animation(model, name, times, node_tracks)

    def _anim_times(self, aset):
        kf_outer = shallow_find(aset, 'keyframe')
        if kf_outer:
            kf_inner = shallow_find(kf_outer, 'keyframe') or kf_outer
            tnode = shallow_find(kf_inner, 'times')
            if tnode:
                t = floats(atoms_of(tnode)[1:])
                if not t and lists_of(tnode):
                    t = floats(lists_of(tnode)[0])
                return t
        return []

    def _read_anim_track(self, anode, node_tracks, times):
        # the animated node name lives in 'parent' (LT2.2/Jupiter); some
        # dialects/converters use 'target'. Try both.
        tnode = shallow_find(anode, 'parent') or shallow_find(anode, 'target')
        target = first_string(tnode) if tnode else first_string(anode)
        frames = shallow_find(anode, 'frames')
        if frames is None or target not in self._name_to_index:
            return
        pq = shallow_find(frames, 'posquat')
        if pq is None:
            return
        entries = lists_of(pq)
        if len(entries) == 1 and lists_of(entries[0]) and not node_name(entries[0]):
            entries = lists_of(entries[0])

        track = []
        for e in entries:
            sub = lists_of(e)
            pos_v, quat_v = None, None
            np_, nq_ = shallow_find(e, 'pos'), shallow_find(e, 'quat')
            if np_ or nq_:
                if np_:
                    vv = floats(atoms_of(np_)[1:]) or (floats(lists_of(np_)[0]) if lists_of(np_) else [])
                    if len(vv) >= 3:
                        pos_v = vv
                if nq_:
                    vv = floats(atoms_of(nq_)[1:]) or (floats(lists_of(nq_)[0]) if lists_of(nq_) else [])
                    if len(vv) >= 4:
                        quat_v = vv
            elif len(sub) >= 2:
                a, b = floats(sub[0]), floats(sub[1])
                if len(a) >= 3:
                    pos_v = a
                if len(b) >= 4:
                    quat_v = b
            if pos_v is None and quat_v is None:
                continue
            loc = Vector(pos_v[:3]) if pos_v else Vector()
            rot = _quat_xyzw(quat_v) if quat_v else Quaternion()
            track.append((loc, rot))
        if track:
            node_tracks[target] = track

    def _build_animation(self, model, name, times, node_tracks):
        anim = Animation()
        anim.name = name
        kc = len(times)
        anim.keyframe_count = kc

        anim.keyframes = []
        for tt in times:
            kf = Animation.Keyframe()
            kf.time = int(round(tt * _TIME_TO_MS))
            kf.string = ''
            anim.keyframes.append(kf)

        # one (loc, rot) per node per keyframe; un-animated nodes hold rest-local
        anim.node_keyframe_transforms = []
        for node in model.nodes:
            track = node_tracks.get(node.name)
            row = []
            if track:
                for k in range(kc):
                    loc, rot = track[k] if k < len(track) else track[-1]
                    t = Animation.Keyframe.Transform()
                    t.location = loc.copy()
                    t.rotation = rot.copy()
                    row.append(t)
            else:
                lm = self._local_of(node)
                loc = lm.to_translation()
                rot = lm.to_quaternion()
                for _ in range(kc):
                    t = Animation.Keyframe.Transform()
                    t.location = loc.copy()
                    t.rotation = rot.copy()
                    row.append(t)
            anim.node_keyframe_transforms.append(row)

        model.animations.append(anim)
