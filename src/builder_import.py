# -*- coding: utf-8 -*-
"""
builder_import.py  --  ONE job: build Blender objects from an abc.py:Model
(raw LithTech space) via the swap_* convention. STATIC stage:
armature + meshes (normals/uvs) + vertex weights + sockets. No animation here.

Objects get an IDENTITY transform. No 90-degree rotation, no scale.z = -1.
All coordinate conversion lives in coordinates.py and is applied exactly once,
here, at the data boundary.

Works as an addon package member and standalone (path) via the import shim.
"""

import bpy
import json
import os
from mathutils import Matrix

try:
    from .coordinates import (swap_vec, swap_dir, swap_quat, swap_matrix,
                              flip_winding, geo_signature)
except ImportError:
    from coordinates import (swap_vec, swap_dir, swap_quat, swap_matrix,
                             flip_winding, geo_signature)


# --------------------------------------------------------------------------
# Serialize structured weight-sets / child-models (populated by the BINARY
# readers: ABC / LTB-PC / LTB-PS2) into the LT2.2 LTA on-load-cmd text used by
# the verbatim-preserved export path. LTA sources already carry this text
# verbatim in model.preserved_raw; binary sources do not, so we build it here.
# Validated byte-for-value against the canonical ModelEdit hero LTA: 109/109
# weight-sets identical, child-models (basemodel 816 / fembase 211) identical.
# save-index == ChildModel.build_number; the LTA filename drops the extension;
# the self-referencing first child model (empty name) is skipped.
def _serialize_weightsets(weight_sets, ffmt='%.6f'):
    if not weight_sets:
        return ''
    lines = ['(anim-weightsets (']
    for ws in weight_sets:
        vals = ' '.join(ffmt % float(w) for w in ws.node_weights)
        lines.append('\t(weightset ')
        lines.append('\t\t(name "%s" )' % ws.name)
        lines.append('\t\t(weights ')
        lines.append('\t\t\t(%s ) ))' % vals)
    lines.append('\t))')
    return '\n'.join(lines)


def _serialize_childmodels(child_models):
    if not child_models:
        return ''
    items = []
    for c in child_models:
        base = os.path.splitext(c.name or '')[0]
        if not base:                      # skip the self-reference entry
            continue
        items.append((base, int(getattr(c, 'build_number', 0) or 0)))
    if not items:
        return ''
    lines = ['(add-childmodels (']
    for fn, si in items:
        lines.append('\t(child-model ')
        lines.append('\t\t(filename "%s" )' % fn)
        lines.append('\t\t(save-index %d ))' % si)
    lines.append('\t))')
    return '\n'.join(lines)


# --------------------------------------------------------------------------
def _new_collection(name):
    col = bpy.data.collections.new(name)
    bpy.context.scene.collection.children.link(col)
    return col


# --------------------------------------------------------------------------
def build_armature(model, name, collection):
    arm_data = bpy.data.armatures.new(name)
    arm_obj = bpy.data.objects.new(name, arm_data)
    collection.objects.link(arm_obj)

    bpy.context.view_layer.objects.active = arm_obj
    bpy.ops.object.mode_set(mode='EDIT')

    world = {}
    for node in model.nodes:
        eb = arm_data.edit_bones.new(node.name)
        eb.head = (0.0, 0.0, 0.0)
        eb.tail = (0.0, 1.0, 0.0)          # unit length so .matrix is settable
        mw = swap_matrix(node.bind_matrix)  # LithTech world -> Blender world
        eb.matrix = mw
        world[node] = mw

    for node in model.nodes:
        eb = arm_data.edit_bones[node.name]
        if node.parent is not None:
            eb.parent = arm_data.edit_bones[node.parent.name]
        head = world[node].translation
        children = getattr(node, 'children', [])
        if children:
            d = min((world[c].translation - head).length for c in children)
            length = d if d > 1e-4 else 1.0
        elif node.parent is not None:
            length = (world[node.parent].translation - head).length * 0.5
            length = length if length > 1e-4 else 1.0
        else:
            length = 1.0
        eb.length = length

    bpy.ops.object.mode_set(mode='OBJECT')
    return arm_obj


# --------------------------------------------------------------------------
def build_piece(model, piece, arm_obj, collection, import_lod_levels=True,
                 base_lod_level=0, objs_out=None):
    """Build one Blender object per LOD level of `piece.lods`.

    Binary sources (ABC / LTB) can carry several real, separate LOD meshes
    per piece (piece.lods[0], [1], [2], ...) -- this addon used to always
    build only piece.lods[0] and silently drop the rest. LTA sources always
    carry exactly one LOD per piece (each LOD level is its own top-level
    shape/Piece there instead, grouped via model.lod_groups); base_lod_level
    carries that level in from build_model() purely for tagging.

    import_lod_levels=False keeps the old behaviour: only the first
    (highest-detail) level is built. When True, every level is built as its
    own object and every level above 0 is hidden by default -- the viewport
    looks the same either way, the lower-detail meshes are just there to
    inspect, edit or re-export.

    Returns the number of objects built (0 if the piece had none)."""
    if not piece.lods:
        return 0
    levels = piece.lods if import_lod_levels else piece.lods[:1]
    built = 0
    for i, lod in enumerate(levels):
        level = base_lod_level + i
        # Only binary multi-LOD pieces (i > 0) get a name suffix -- an LTA
        # shape's Blender object name IS its LTA shape name (the exporter
        # reads obj.name back directly), so it must never be touched.
        suffix = '' if i == 0 else '_lod%d' % i
        obj = _build_lod_object(model, piece, lod, piece.name + suffix,
                                arm_obj, collection, level)
        if obj is not None:
            built += 1
            if objs_out is not None:
                objs_out.append(obj)
            if level > 0:
                _hide_object(obj)
    return built


def _hide_object(obj):
    try:
        obj.hide_set(True, view_layer=bpy.context.view_layer)
    except Exception:
        try:
            obj.hide_set(True)
        except Exception:
            obj.hide_viewport = True


def _build_lod_object(model, piece, lod, obj_name, arm_obj, collection,
                      lod_level):
    # Null-mesh LODs (LOD.type == 7) carry no vertices/faces at all. Bail out
    # before touching Blender mesh APIs -- an empty mesh fed into
    # normals_split_custom_set_from_vertices() is a known crash trigger.
    if not lod.vertices or not lod.faces:
        print("  [skip] piece '%s': empty LOD (type=%s, %d verts, %d faces)"
              % (piece.name, getattr(lod, 'type', '?'), len(lod.vertices), len(lod.faces)))
        return None

    verts = [tuple(swap_vec(v.location)) for v in lod.vertices]
    normals = [tuple(swap_dir(v.normal)) for v in lod.vertices]

    # DIAGNOSTIC: some LTB vertex-format masks omit VTX_Normal entirely, in
    # which case abc.py:Vertex.normal stays at its default (0,0,0). Feeding a
    # zero/NaN vector into normals_split_custom_set_from_vertices() is the
    # other known crash trigger -- sanitize and report instead of crashing.
    bad_idx = [i for i, n in enumerate(normals)
               if not all(c == c for c in n)                 # NaN check
               or (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]) < 1e-8]  # zero-length
    if bad_idx:
        print("  [normals] piece '%s': %d/%d degenerate normal(s), e.g. vertex idx %s -- using fallback (0,0,1)"
              % (piece.name, len(bad_idx), len(normals), bad_idx[:5]))
        for i in bad_idx:
            normals[i] = (0.0, 0.0, 1.0)

    # DIAGNOSTIC: degenerate (zero-area) triangles -- duplicate vertex index
    # within one face -- also a known crash trigger for the custom-normals
    # backend. Report them; they are dropped from the face list below.
    degenerate_faces = 0

    faces, corner_uv = [], []
    for face in lod.faces:
        idx = [fv.vertex_index for fv in face.vertices]
        if len(set(idx)) != 3:
            degenerate_faces += 1
            continue
        # TODO: LTB-PC can carry up to 3 extra UV channels per FaceVertex
        # (fv.extra_texcoords[0..2], parsed in reader_ltb_pc.py) -- only the
        # primary UV (fv.texcoord) becomes a Blender UV map here; the extra
        # channels are read but silently discarded at build time.
        uv = [(fv.texcoord[0], 1.0 - fv.texcoord[1]) for fv in face.vertices]  # DX->Blender V
        a, c, b = flip_winding(idx)
        faces.append((a, c, b))
        corner_uv.append(flip_winding(uv))

    if degenerate_faces:
        print("  [faces] piece '%s': dropped %d degenerate (zero-area) face(s)"
              % (piece.name, degenerate_faces))

    if not faces:
        print("  [skip] piece '%s': no valid faces after filtering" % piece.name)
        return None

    mesh = bpy.data.meshes.new(obj_name)
    mesh.from_pydata(verts, [], faces)
    mesh.update()

    for poly in mesh.polygons:
        poly.use_smooth = True
    try:
        mesh.normals_split_custom_set_from_vertices(normals)
    except Exception as e:
        print("  [normals] skipped: %s" % e)

    # Blender's custom-split-normal round-trip through to_mesh()/modifier
    # evaluation is unreliable (4.x), so the exported shading came out wrong
    # (dark). Store the original per-vertex normal as a plain POINT attribute
    # too; the exporter reads this back for exact, robust normal preservation.
    try:
        attr = mesh.attributes.new('lt_normal', 'FLOAT_VECTOR', 'POINT')
        attr.data.foreach_set('vector', [c for n in normals for c in n])
        # geometry fingerprint so the exporter can tell if the mesh was later
        # deformed; if so it recomputes normals instead of re-using lt_normal.
        mesh['lt_geo_sig'] = geo_signature([v.co for v in mesh.vertices])
    except Exception as e:
        print("  [lt_normal] skipped: %s" % e)

    uv_layer = mesh.uv_layers.new(name="UVMap")
    li = 0
    for fi, poly in enumerate(mesh.polygons):
        for k in range(3):
            uv_layer.data[li].uv = corner_uv[fi][k]
            li += 1

    obj = bpy.data.objects.new(obj_name, mesh)
    collection.objects.link(obj)

    # preserve the texture/material index (model stores an index, NOT a filename).
    # store it on the object for round-trip, and attach a shared, EMPTY material
    # slot per index ("LT_tex<N>") so the user can assign their own DTX/image once.
    tex_index = int(getattr(piece, 'material_index', 0))
    obj["lt_texture_index"] = tex_index
    # preserve LOD piece-priority (ModelEdit's set-repl-lod-original uses this);
    # baron's values (1/2/2/1/1/3) come straight from the source lod_weight.
    obj["lt_lod_weight"] = float(getattr(piece, 'lod_weight', 1.0) or 1.0)
    obj["lt_lod_level"] = int(lod_level)
    # Typ A LOD (lod-groups, LTA-only): tag which group this shape belongs
    # to, purely informational.
    for g in getattr(model, 'lod_groups', None) or []:
        if piece.name in g.get('shapes', []):
            obj["lt_lod_group"] = g.get('name', '')
            break
    mat_name = "LT_tex%d" % tex_index
    mat = bpy.data.materials.get(mat_name)
    if mat is None:
        mat = bpy.data.materials.new(mat_name)
        mat.use_nodes = True
        mat["lt_texture_index"] = tex_index
    mesh.materials.append(mat)

    name_to_group = {nd.name: obj.vertex_groups.new(name=nd.name) for nd in model.nodes}
    for vi, v in enumerate(lod.vertices):
        for w in v.weights:
            if w.node_index < len(model.nodes):
                name_to_group[model.nodes[w.node_index].name].add([vi], float(w.bias), 'REPLACE')

    obj.parent = arm_obj
    mod = obj.modifiers.new("Armature", 'ARMATURE')
    mod.object = arm_obj
    return obj


# --------------------------------------------------------------------------
def check_bounds_alignment(model, built_objs):
    """Sanity check, diagnostic only: model.internal_radius is a field every
    reader (ABC/LTB/LTA) populates independently of piece/vertex parsing.
    The actual geometry, once built here in Blender space, should sit
    roughly inside a sphere of that radius around the origin. A big
    mismatch is a cheap, early signal that something upstream went wrong --
    wrong LOD level read, a coordinate-swap regression, corrupt vertex
    data, wrong scale -- rather than silently importing a garbled mesh that
    only becomes obvious later when it looks wrong in the viewport.

    Never blocks the import; only prints. Tolerance (0.5x - 1.5x) is a
    guess at a reasonable margin, not a value taken from any spec."""
    radius = float(getattr(model, 'internal_radius', 0.0) or 0.0)
    if radius <= 0.0 or not built_objs:
        return
    max_d2 = 0.0
    for obj in built_objs:
        for v in obj.data.vertices:
            d2 = v.co.length_squared
            if d2 > max_d2:
                max_d2 = d2
    actual = max_d2 ** 0.5
    if actual <= 0.0:
        return
    ratio = actual / radius
    if ratio > 1.5 or ratio < 0.5:
        print("  [bbox-check] WARNING: model.internal_radius=%.2f but built "
              "geometry extends to %.2f from origin (%.2fx) -- possible "
              "coordinate/scale mismatch or wrong LOD/vertex data"
              % (radius, actual, ratio))
    else:
        print("  [bbox-check] OK: internal_radius=%.2f, actual geometry radius=%.2f"
              % (radius, actual))


# --------------------------------------------------------------------------
def build_sockets(model, arm_obj, collection):
    for sock in getattr(model, 'sockets', []):
        if sock.node_index >= len(model.nodes):
            continue
        node = model.nodes[sock.node_index]
        empty = bpy.data.objects.new("s_" + sock.name, None)
        empty.empty_display_type = 'ARROWS'
        empty.empty_display_size = 2.0
        collection.objects.link(empty)

        # socket transform is LOCAL to its parent node -> world = node_world @ local
        node_world = swap_matrix(node.bind_matrix)
        sock_local = Matrix.LocRotScale(swap_vec(sock.location),
                                        swap_quat(sock.rotation), None)
        world = node_world @ sock_local

        empty.parent = arm_obj
        empty.parent_type = 'BONE'
        empty.parent_bone = node.name
        empty.matrix_world = world      # set AFTER parenting so placement is correct


# --------------------------------------------------------------------------
def apply_model_metadata(model, arm_obj):
    """Stash Model metadata as custom properties that the (v1) LTA exporter
    reads back: per-bone node flags, global radius, command string. Animation
    bindings (dims/translation/interp) are applied per-Action in
    animation_import. Covers every format, since all readers share one Model."""
    radius = getattr(model, 'internal_radius', 0.0) or 0.0
    if radius:
        arm_obj['lta_global_radius'] = float(radius)
    cmd = getattr(model, 'command_string', None)
    if cmd:
        arm_obj['lta_command_string'] = str(cmd)

    # per-bone node flags -> pose bone custom prop 'lta_node_flags'
    flagged = 0
    for node in model.nodes:
        flags = getattr(node, 'flags', 0) or 0
        pb = arm_obj.pose.bones.get(node.name)
        if pb is not None and flags:
            pb['lta_node_flags'] = int(flags)
            flagged += 1

    # store the EXACT intended rest matrix (Blender armature space) on each data
    # bone. Blender reconstructs bones from head/tail/roll, which does NOT
    # reproduce the original node matrix exactly -> the exporter must use this
    # stored matrix instead of bone.matrix_local, or the skeleton drifts and the
    # skinned mesh collapses on export.
    for node in model.nodes:
        db = arm_obj.data.bones.get(node.name)
        if db is not None:
            mw = swap_matrix(node.bind_matrix)
            db['lt_rest'] = [mw[r][c] for r in range(4) for c in range(4)]
            # Blender reconstructs bone.matrix_local from head/tail/roll, which
            # differs slightly from the true bind matrix above -- that's why we
            # keep lt_rest. Record THIS reconstructed baseline too, so the
            # exporter can detect whether the user later MOVED the bone: if the
            # live matrix_local still matches this baseline, the bone is
            # untouched and we emit the exact lt_rest; if it changed, the user
            # rerigged and we emit the live matrix_local instead.
            ml = db.matrix_local
            db['lt_rest_bl'] = [ml[r][c] for r in range(4) for c in range(4)]

    # preserved raw on-load-cmd blocks (anim-weightsets / child-models /
    # set-repl-lod-original / node-obb) for verbatim re-emission on export.
    preserved = dict(getattr(model, 'preserved_raw', None) or {})

    # Binary sources (ABC / LTB) carry no LTA text, but DO populate the
    # structured weight_sets / child_models. Synthesize the LTA text from them
    # so character weight-sets and animation child-models survive binary->LTA.
    if not preserved.get('lta_weightsets') and getattr(model, 'weight_sets', None):
        txt = _serialize_weightsets(model.weight_sets)
        if txt:
            preserved['lta_weightsets'] = txt
    if not preserved.get('lta_childmodels') and getattr(model, 'child_models', None):
        txt = _serialize_childmodels(model.child_models)
        if txt:
            preserved['lta_childmodels'] = txt

    for key, val in preserved.items():
        if val:
            arm_obj[key] = val

    # Typ A LOD (LTA-only, real per-distance meshes): (lod-groups
    # (create-lod-group ...)), read structurally by reader_lta.py into
    # model.lod_groups. Stashed as JSON so the exporter can re-emit it for
    # whichever shapes still exist; does not change how those shapes are
    # built here (still one object per shape, same as any ungrouped shape).
    lod_groups = getattr(model, 'lod_groups', None)
    if lod_groups:
        arm_obj['lta_lod_groups'] = json.dumps(lod_groups)

    # LOD Typ B ingredient (ABC only -- confirmed a real, structured binary
    # field via a user-supplied byte-level view of an .abc file matching
    # ModelEdit's own "Level of Detail Generation" dialog exactly: 300/600/
    # 1000/1500). This is the model-wide distance list a set-repl-lod-
    # original recipe needs; exporter_lta.py derives the matching tri-%
    # from the actual multi-LOD sibling objects at export time instead of
    # guessing it (nothing in any reader stores tri-% directly).
    # LTB-PC's equivalent (piece.lod_distances) is PER-PIECE, not global,
    # and not wired here -- merging per-piece lists into one global list
    # would be a guess about which piece's list should win.
    lod_distances = list(getattr(model, 'lod_distances', None) or [])
    if lod_distances:
        arm_obj['lta_lod_distances'] = json.dumps([float(d) for d in lod_distances])

    print("METADATA: radius=%s cmd=%s node_flags=%d preserved=%s lod_groups=%d"
          % (radius or '-', 'yes' if cmd else 'no', flagged,
             list(preserved.keys()) or '-', len(lod_groups or [])))


# --------------------------------------------------------------------------
def build_model(model, name="LTModel", import_lod_levels=True):
    """import_lod_levels=True builds every LOD level available (binary
    piece.lods[1:], and -- for LTA -- every extra shape a (lod-groups
    (create-lod-group ...)) block names as a lower-detail sibling of an
    existing shape), hidden by default. False keeps only the
    highest-detail level for both cases, i.e. the old behaviour."""
    col = _new_collection(name)
    arm_obj = build_armature(model, name, col)

    # LTA-only: map a piece name that is a non-first (lower-detail) entry
    # in a (lod-groups ...) block to its level, so those top-level Pieces --
    # unlike binary piece.lods entries -- can be tagged/hidden/skipped the
    # same way even though each one is its own separate Piece, not a
    # piece.lods[] entry of the group's first (highest-detail) Piece.
    lod_level_of = {}
    for g in getattr(model, 'lod_groups', None) or []:
        for lvl, sname in enumerate(g.get('shapes', [])):
            if lvl > 0:
                lod_level_of[sname] = lvl

    n_mesh = 0
    built_objs = []
    for piece in model.pieces:
        base_level = lod_level_of.get(piece.name, 0)
        if not import_lod_levels and base_level > 0:
            continue  # LTA lower-detail sibling shape, toggle off -> skip entirely
        n_mesh += build_piece(model, piece, arm_obj, col, import_lod_levels,
                              base_lod_level=base_level, objs_out=built_objs)
    build_sockets(model, arm_obj, col)
    apply_model_metadata(model, arm_obj)
    check_bounds_alignment(model, built_objs)
    print("=" * 56)
    print("BUILT '%s': %d bones, %d meshes, %d sockets"
          % (name, len(model.nodes), n_mesh, len(getattr(model, 'sockets', []))))
    print("Object transforms are IDENTITY (no 90-deg, no scale.z=-1).")
    print("=" * 56)
    return arm_obj
