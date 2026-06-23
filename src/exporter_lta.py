# SPDX-License-Identifier: GPL-3.0-or-later
#
# Jupiter / LithTech .LTA Model Exporter for Blender 4.5+
#
# Exports Blender armatures + meshes to the LithTech ASCII model format
# (lt-model-0 schema) used by Jupiter-engine games such as
# No One Lives Forever 2 and Tron 2.0. Written against the official
# ".LTA Schema Reference" (2002.04.29). The resulting .lta files are meant
# to be opened in ModelEdit and compiled to .ltb by ModelPacker, replacing
# the original Maya exporters.

# IMPORTANT!!!! AT THE CURRENT MOMENT, IMPORT ANIMS FROM ORIGINAL MODEL IN MODELEDIT AFTER YOU EXPORT!!!!!
# THE ANIMS DON'T SEEM TO ATTACH TO MODEL BUT THEY ARE THERE, SOMETHING I SCREWED UP! TO BE FIXED!
# Remember to have a triangulated mesh, the plugin doesn't do it.

bl_info = {
    "name": "Export Jupiter LithTech LTA (Model)",
    "author": "",
    "version": (2, 0, 0),
    "blender": (4, 5, 0),
    "location": "File > Export > LithTech Model (.lta)",
    "description": "Export LithTech Jupiter .LTA model files (NOLF2 etc.)",
    "category": "Import-Export",
}

import os
import time

import bpy
from bpy.props import (
    StringProperty,
    BoolProperty,
    FloatProperty,
    IntProperty,
    EnumProperty,
)
from bpy.types import Operator
from bpy_extras.io_utils import ExportHelper
from mathutils import Matrix, Vector, Quaternion

try:
    from .coordinates import geo_signature, mat_close
except ImportError:
    from coordinates import geo_signature, mat_close


# ---------------------------------------------------------------------------
# Coordinate conversion (Blender right-handed Z-up -> LithTech left-handed
# Y-up). The Y<->Z swap is an involution, so the same conjugation converts
# both ways; triangle winding is flipped to compensate the handedness flip.
# ---------------------------------------------------------------------------

_C = Matrix(((1, 0, 0, 0),
             (0, 0, 1, 0),
             (0, 1, 0, 0),
             (0, 0, 0, 1)))


def vec_to_lt(v, scale=1.0):
    return (v[0] * scale, v[2] * scale, v[1] * scale)


def dir_to_lt(v):
    return (v[0], v[2], v[1])


def quat_to_lt(q):
    # Blender Quaternion (w, x, y, z) -> LT (x, y, z, w)
    return (-q.x, -q.z, -q.y, q.w)


def mat_to_lt(m, scale=1.0):
    out = _C @ m @ _C
    out.translation = out.translation * scale
    return out


# ---------------------------------------------------------------------------
# LTA writer
# ---------------------------------------------------------------------------

class LTAWriter:
    """Builds nicely indented LTA text. Mirrors the structure produced by
    the original LithTech tool chain so ModelEdit's reader is happy."""

    def __init__(self, float_digits=6):
        self.lines = []
        self.depth = 0
        self.ffmt = "%%.%df" % float_digits

    # -- low level --
    def open(self, *head):
        base = '\t' * self.depth
        self.lines.append(base + '( ' + ' '.join(head) if head else base + '(')
        self.depth += 1

    def close(self):
        self.depth -= 1
        self.lines.append('\t' * self.depth + ')')

    def line(self, text):
        self.lines.append('\t' * self.depth + text)

    def raw_block(self, text):
        base = '\t' * self.depth
        for ln in text.splitlines():
            self.lines.append(base + ln)

    def f(self, v):
        s = self.ffmt % v
        return s

    def s(self, v):
        return '"%s"' % v

    def vec(self, v):
        return '( ' + ' '.join(self.f(c) for c in v) + ' )'

    def leaf(self, name, *vals):
        self.line('( %s %s )' % (name, ' '.join(vals)))

    def text(self):
        return '\n'.join(self.lines) + '\n'


# ---------------------------------------------------------------------------
# Scene data gathering
# ---------------------------------------------------------------------------

class ExportError(Exception):
    pass


class BoneInfo:
    __slots__ = ("name", "parent", "rest_arm", "index")

    def __init__(self, name, parent, rest_arm, index):
        self.name = name
        self.parent = parent          # BoneInfo or None
        self.rest_arm = rest_arm      # rest matrix in armature space
        self.index = index


class LTAExporter:

    def __init__(self, operator, context, filepath, opts):
        self.op = operator
        self.context = context
        self.filepath = filepath
        self.o = opts
        self.bones = []
        self.bone_by_name = {}
        self.warnings = []

    def warn(self, msg):
        self.warnings.append(msg)
        self.op.report({'WARNING'}, msg)

    # -- gathering -------------------------------------------------------

    def find_objects(self):
        ctx = self.context
        if self.o.use_selection:
            pool = list(ctx.selected_objects)
        else:
            pool = [o for o in ctx.scene.objects if o.visible_get()]

        armatures = [o for o in pool if o.type == 'ARMATURE']
        if not armatures:
            # fall back: armature referenced by selected meshes
            for o in pool:
                if o.type == 'MESH':
                    for mod in o.modifiers:
                        if mod.type == 'ARMATURE' and mod.object:
                            armatures.append(mod.object)
                    if o.parent and o.parent.type == 'ARMATURE':
                        armatures.append(o.parent)
        armatures = list(dict.fromkeys(armatures))
        if not armatures:
            raise ExportError(
                "No armature found. A LithTech model needs a node "
                "hierarchy: add an armature with at least one bone and "
                "parent/skin your meshes to it.")
        if len(armatures) > 1:
            self.warn("Multiple armatures found; exporting '%s'."
                      % armatures[0].name)
        self.arm_obj = armatures[0]

        self.mesh_objs = []
        scene_meshes = ctx.selected_objects if self.o.use_selection \
            else ctx.scene.objects
        for o in scene_meshes:
            if o.type != 'MESH' or not o.visible_get():
                continue
            uses_arm = (o.parent == self.arm_obj)
            for mod in o.modifiers:
                if mod.type == 'ARMATURE' and mod.object == self.arm_obj:
                    uses_arm = True
            if uses_arm or self.o.use_selection:
                self.mesh_objs.append(o)
        if not self.mesh_objs:
            self.warn("No mesh objects found for the armature; exporting "
                      "skeleton/animations only (child-model style file).")

        self.socket_objs = []
        if self.o.export_sockets:
            for o in ctx.scene.objects:
                if o.type == 'EMPTY' and o.parent == self.arm_obj and \
                        (o.parent_type == 'BONE' or 'lta_socket' in o):
                    self.socket_objs.append(o)

    def gather_bones(self):
        arm = self.arm_obj.data
        index = 0
        ordered = []

        def visit(bone, parent_info):
            nonlocal index
            # prefer the EXACT rest stored at import over Blender's reconstructed
            # bone.matrix_local (which drifts and collapses the skinned mesh).
            # Prefer the EXACT bind matrix stored at import over Blender's
            # reconstructed bone.matrix_local (which drifts and collapses the
            # skinned mesh) -- BUT only while the bone is untouched. If the user
            # moved the bone (rerigging), matrix_local no longer matches the
            # baseline we recorded at import, so emit the live matrix_local so
            # the edit takes effect.
            stored = bone.get('lt_rest')
            baseline = bone.get('lt_rest_bl')
            live = bone.matrix_local.copy()
            if stored is not None and len(stored) == 16:
                rest_stored = Matrix([[stored[r * 4 + c] for c in range(4)]
                                      for r in range(4)])
                moved = False
                if baseline is not None and len(baseline) == 16:
                    bl = Matrix([[baseline[r * 4 + c] for c in range(4)]
                                 for r in range(4)])
                    moved = not mat_close(live, bl)
                rest = live if moved else rest_stored
            else:
                rest = live
            info = BoneInfo(bone.name, parent_info, rest, index)
            index += 1
            self.bones.append(info)
            self.bone_by_name[bone.name] = info
            for c in bone.children:
                visit(c, info)

        roots = [b for b in arm.bones if b.parent is None]
        if not roots:
            raise ExportError("Armature '%s' has no bones." %
                              self.arm_obj.name)
        for r in roots:
            visit(r, None)
        if len(roots) > 1:
            self.warn("Armature has %d root bones. ModelEdit generally "
                      "expects a single root node; consider parenting "
                      "everything under one root bone." % len(roots))

    # -- mesh extraction ----------------------------------------------------

    def extract_meshes(self):
        """Returns a list of dicts describing each shape in LT space."""
        depsgraph = None
        disabled = []
        if self.o.apply_modifiers:
            # bind pose must be exported: hide armature modifiers during eval
            for o in self.mesh_objs:
                for mod in o.modifiers:
                    if mod.type == 'ARMATURE' and mod.show_viewport:
                        mod.show_viewport = False
                        disabled.append(mod)
            depsgraph = self.context.evaluated_depsgraph_get()

        arm_inv = self.arm_obj.matrix_world.inverted_safe()
        shapes = []
        try:
            for obj in self.mesh_objs:
                if self.o.apply_modifiers:
                    ev = obj.evaluated_get(depsgraph)
                    mesh = ev.to_mesh()
                else:
                    ev = obj
                    mesh = obj.to_mesh()
                try:
                    shapes.append(self._extract_one(obj, mesh, arm_inv))
                finally:
                    ev.to_mesh_clear()
        finally:
            for mod in disabled:
                mod.show_viewport = True
        return [s for s in shapes if s]

    def _extract_one(self, obj, mesh, arm_inv):
        scale = self.o.scale
        to_arm = arm_inv @ obj.matrix_world
        nrm_mat = to_arm.to_3x3().inverted_safe().transposed()

        mesh.calc_loop_triangles()
        if not mesh.loop_triangles:
            self.warn("Mesh '%s' has no faces; skipped." % obj.name)
            return None

        verts_lt = [vec_to_lt(to_arm @ v.co, scale) for v in mesh.vertices]

        # vertex groups -> influences/weights
        deformer = None
        if self.o.export_weights:
            valid_groups = {g.index: g.name for g in obj.vertex_groups
                            if g.name in self.bone_by_name}
            if valid_groups:
                influences = []
                inf_index = {}
                weights = []
                limit = self.o.max_weights
                for v in mesh.vertices:
                    pairs = []
                    for ge in v.groups:
                        nm = valid_groups.get(ge.group)
                        if nm and ge.weight > 1e-5:
                            pairs.append((nm, ge.weight))
                    pairs.sort(key=lambda p: -p[1])
                    if limit > 0:
                        pairs = pairs[:limit]
                    total = sum(w for _n, w in pairs)
                    if total <= 0.0:
                        root = self.bones[0].name
                        pairs, total = [(root, 1.0)], 1.0
                    out = []
                    for nm, w in pairs:
                        if nm not in inf_index:
                            inf_index[nm] = len(influences)
                            influences.append(nm)
                        out.append((inf_index[nm], w / total))
                    weights.append(out)
                deformer = (influences, weights)

        # UVs ---------------------------------------------------------------
        uv_layer = mesh.uv_layers.active if self.o.export_uvs else None
        uvs, uv_index = [], {}
        loop_uv = None
        if uv_layer:
            loop_uv = [None] * len(mesh.loops)
            data = uv_layer.uv if hasattr(uv_layer, "uv") else uv_layer.data
            for li in range(len(mesh.loops)):
                uv = data[li].vector if hasattr(data[li], "vector") \
                    else data[li].uv
                key = (round(uv[0], 6), round(1.0 - uv[1], 6))
                i = uv_index.get(key)
                if i is None:
                    i = len(uvs)
                    uv_index[key] = i
                    uvs.append(key)
                loop_uv[li] = i

        # normals -------------------------------------------------------------
        # LT2.2 / Jupiter ModelEdit read ONE normal per vertex, PARALLEL to the
        # vertex array, with NO nrm-fs (LTA schema: "if there are no nrm-fs
        # nodes ... a normal per vertex"). Indexed normals + nrm-fs are valid
        # too (stripped meshes) but ModelEdit can render them dark, so that form
        # is opt-in via 'indexed_normals'. Read the exact import-time normals
        # from the ORIGINAL mesh (obj.data); the evaluated copy drops them.
        normals, nrm_index, loop_nrm = [], {}, None
        if self.o.export_normals:
            src = obj.data
            same_topo = len(src.vertices) == len(mesh.vertices)
            # lt_normal holds the exact imported per-vertex normals, but they
            # go stale if the mesh is deformed. Compare the geometry signature
            # taken at import: if the vertices moved, recompute normals from the
            # current geometry instead of re-using the stale imported ones.
            deformed = False
            sig0 = src.get('lt_geo_sig')
            if sig0 is not None and same_topo:
                deformed = (geo_signature([v.co for v in src.vertices]) != sig0)
            lt_attr = (src.attributes.get('lt_normal')
                       if same_topo and not deformed else None)
            use_lt = lt_attr is not None and lt_attr.domain == 'POINT'

            def _vnorm(vi):
                vec = lt_attr.data[vi].vector
                n = nrm_mat @ Vector((vec[0], vec[1], vec[2]))
                if n.length < 1e-9:
                    n = Vector((0.0, 0.0, 1.0))
                n.normalize()
                return dir_to_lt((round(n[0], 4), round(n[1], 4),
                                  round(n[2], 4)))

            if getattr(self.o, 'indexed_normals', False):
                # de-duplicated normals + nrm-fs (per-loop indices)
                loop_nrm = [0] * len(mesh.loops)
                corner = None if use_lt else mesh.corner_normals
                for li in range(len(mesh.loops)):
                    if use_lt:
                        key = _vnorm(mesh.loops[li].vertex_index)
                    else:
                        n = nrm_mat @ Vector(corner[li].vector)
                        if n.length < 1e-9:
                            n = Vector((0.0, 0.0, 1.0))
                        n.normalize()
                        key = dir_to_lt((round(n[0], 4), round(n[1], 4),
                                         round(n[2], 4)))
                    i = nrm_index.get(key)
                    if i is None:
                        i = len(normals)
                        nrm_index[key] = i
                        normals.append(key)
                    loop_nrm[li] = i
            else:
                # one normal per vertex, parallel to the vertex array (default)
                for vi in range(len(mesh.vertices)):
                    if use_lt:
                        normals.append(_vnorm(vi))
                    else:
                        n = nrm_mat @ mesh.vertices[vi].normal
                        if n.length < 1e-9:
                            n = Vector((0.0, 0.0, 1.0))
                        n.normalize()
                        normals.append(dir_to_lt((round(n[0], 4),
                                                  round(n[1], 4),
                                                  round(n[2], 4))))




        # vertex colors ---------------------------------------------------------
        colors, col_index, loop_col = [], {}, None
        if self.o.export_colors and mesh.color_attributes:
            attr = mesh.color_attributes.active_color or \
                mesh.color_attributes[0]
            if attr.domain in {'CORNER', 'POINT'}:
                loop_col = [0] * len(mesh.loops)
                for li, loop in enumerate(mesh.loops):
                    di = li if attr.domain == 'CORNER' else loop.vertex_index
                    c = attr.data[di].color
                    key = (round(c[0], 4), round(c[1], 4),
                           round(c[2], 4), round(c[3], 4))
                    i = col_index.get(key)
                    if i is None:
                        i = len(colors)
                        col_index[key] = i
                        colors.append(key)
                    loop_col[li] = i

        # triangles (flip winding back to left-handed) -------------------------
        tri_fs, tex_fs, nrm_fs, col_fs = [], [], [], []
        for tri in mesh.loop_triangles:
            lv = tri.vertices
            ll = tri.loops
            order = (0, 2, 1)
            tri_fs.extend(lv[i] for i in order)
            if loop_uv is not None:
                tex_fs.extend(loop_uv[ll[i]] for i in order)
            if loop_nrm is not None:
                nrm_fs.extend(loop_nrm[ll[i]] for i in order)
            if loop_col is not None:
                col_fs.extend(loop_col[ll[i]] for i in order)

        # material ------------------------------------------------------------
        mat = obj.active_material
        material = {'name': mat.name if mat else obj.name}
        texture = None
        if mat:
            material['texture-index'] = int(mat.get('lt_texture_index', mat.get('lta_texture_index', 0)))
            texture = mat.get('lta_texture')
            if mat.use_nodes:
                bsdf = next((n for n in mat.node_tree.nodes
                             if n.type == 'BSDF_PRINCIPLED'), None)
                if bsdf:
                    bc = bsdf.inputs['Base Color']
                    material['diffuse'] = tuple(bc.default_value)[:4]
                    if not texture:
                        for link in bc.links:
                            n = link.from_node
                            if n.type == 'TEX_IMAGE' and n.image:
                                texture = os.path.splitext(
                                    os.path.basename(
                                        n.image.filepath or n.image.name))[0]
                                texture += ".dtx"
        else:
            material['texture-index'] = 0

        parent = self.bones[0].name
        if obj.parent_type == 'BONE' and obj.parent_bone in self.bone_by_name:
            parent = obj.parent_bone

        return {
            'name': obj.name,
            'parent': parent,
            'verts': verts_lt,
            'tri_fs': tri_fs,
            'uvs': uvs, 'tex_fs': tex_fs,
            'normals': normals, 'nrm_fs': nrm_fs,
            'colors': colors, 'col_fs': col_fs,
            'deformer': deformer,
            'material': material,
            'texture': texture,
            'render_priority': obj.get('lta_render_priority'),
        }

    # -- sockets ----------------------------------------------------------

    def extract_sockets(self):
        out = []
        arm_inv = self.arm_obj.matrix_world.inverted_safe()
        for e in self.socket_objs:
            bone_name = e.parent_bone or e.get('lta_socket_parent')
            info = self.bone_by_name.get(bone_name)
            if info is None:
                self.warn("Socket empty '%s' is not parented to a known "
                          "bone; skipped." % e.name)
                continue
            rel = info.rest_arm.inverted_safe() @ arm_inv @ e.matrix_world
            loc, rot, _s = rel.decompose()
            name = e.get('lta_socket') or \
                (e.name[2:] if e.name.lower().startswith('s_') else e.name)
            name = name.split('.')[0] if name.count('.') and \
                name.split('.')[-1].isdigit() else name
            out.append((name, bone_name,
                        vec_to_lt(loc, self.o.scale), quat_to_lt(rot)))
        return out

    # -- animations -----------------------------------------------------------

    def collect_actions(self):
        mode = self.o.anim_mode
        if mode == 'NONE':
            actions = []
        elif mode == 'ACTIVE':
            ad = self.arm_obj.animation_data
            actions = [ad.action] if ad and ad.action else []
        else:
            # ALL: only THIS armature's animations -- the ones on its own NLA
            # tracks (our importer puts one per animation there) plus its active
            # action. Using bpy.data.actions would drag in leftover actions from
            # other imported models (e.g. a hero rig's anims exported onto a
            # different character), which then deform the wrong skeleton.
            actions = []
            seen = set()
            ad = self.arm_obj.animation_data
            if ad:
                for tr in ad.nla_tracks:
                    for st in tr.strips:
                        a = getattr(st, 'action', None)
                        if a is not None and a.name not in seen:
                            seen.add(a.name)
                            actions.append(a)
                if ad.action is not None and ad.action.name not in seen:
                    seen.add(ad.action.name)
                    actions.append(ad.action)
            # fallback (no NLA tracks): actions whose bone channels match THIS
            # armature's bones
            if not actions:
                actions = [a for a in bpy.data.actions
                           if self._action_targets_bones(a)]
        if not actions and self.o.add_base_anim:
            actions = [None]   # sentinel: synthesize static 'base'
        elif not actions:
            self.warn("No animations exported. ModelEdit requires every "
                      "model to contain at least one animation; enable "
                      "'Add Base Animation' or export an action.")
        return actions

    def _action_targets_bones(self, action):
        try:
            fcurves = action.fcurves
        except Exception:
            return False
        import re
        arm_bones = set(b.name for b in self.arm_obj.data.bones)
        for fc in fcurves:
            if fc.data_path.startswith('pose.bones['):
                m = re.match(r'pose\.bones\["(.+?)"\]', fc.data_path)
                if m and m.group(1) in arm_bones:
                    return True
        return False

    def sample_action(self, action):
        """Sample an action (or rest pose if None) into
        (name, times_ms, {bone: [(pos, quat)]}, binding dict)."""
        scn = self.context.scene
        fps = scn.render.fps / scn.render.fps_base
        arm_obj = self.arm_obj

        if action is None:
            name = "base"
            frames = [scn.frame_current]
            explicit_times = None
            binding = {'dims': (16.0, 16.0, 16.0),
                       'translation': (0.0, 0.0, 0.0),
                       'interp-time': 200}
        else:
            name = action.name
            f0, f1 = action.frame_range
            # Sample only the action's REAL keyframes, not every integer frame.
            # Sampling every frame bloated the export (385 keyframes vs the
            # original 161) and diverged from canonical ModelEdit output.
            explicit_times = action.get('lta_keyframe_times')
            if explicit_times:
                # exact original timing: derive the frame for each stored time
                explicit_times = [float(t) for t in explicit_times]
                frames = [int(round(t * fps / 1000.0)) for t in explicit_times]
            else:
                kf_frames = set()
                for fc in action.fcurves:
                    for kp in fc.keyframe_points:
                        kf_frames.add(int(round(kp.co[0])))
                if kf_frames:
                    frames = sorted(kf_frames)
                else:
                    step = max(1, self.o.frame_step)
                    frames = list(range(int(round(f0)),
                                        int(round(f1)) + 1, step))
            if not frames:
                frames = [int(round(f0))]
            binding = {
                'dims': tuple(action.get('lta_dims', (16.0, 16.0, 16.0))),
                'translation': tuple(action.get('lta_translation',
                                                (0.0, 0.0, 0.0))),
                'interp-time': int(action.get('lta_interp_time', 200)),
            }
            ws = action.get('lta_weight_set')
            if ws:
                binding['weight-set'] = ws

        ad = arm_obj.animation_data_create()
        prev_action = ad.action
        prev_slot = getattr(ad, "action_slot", None)
        prev_frame = scn.frame_current

        # The NLA player solos an animation by leaving one track un-muted.
        # Blender stacks un-muted NLA tracks ON TOP of the active action, so the
        # last-played animation would contaminate every pose we sample here
        # (e.g. a 'base' export coming out as base + Bar3SitEat -> deformed).
        # Mute all NLA tracks while sampling; only ad.action should drive the
        # pose. Restore the mute state afterwards so the player keeps working.
        nla_mute = []
        if ad.nla_tracks:
            for tr in ad.nla_tracks:
                nla_mute.append((tr, tr.mute))
                tr.mute = True
        prev_use_nla = getattr(ad, "use_nla", None)
        try:
            ad.use_nla = False
        except Exception:
            pass

        if action is not None:
            ad.action = action
            if hasattr(action, "slots") and len(action.slots):
                try:
                    ad.action_slot = action.slots[0]
                except Exception:
                    pass
        else:
            ad.action = None

        tracks = {b.name: [] for b in self.bones}
        f_start = frames[0]
        times = []
        try:
            for fi, f in enumerate(frames):
                scn.frame_set(f)
                depsgraph = self.context.evaluated_depsgraph_get()
                ev = arm_obj.evaluated_get(depsgraph)
                pose_arm = {pb.name: pb.matrix.copy()
                            for pb in ev.pose.bones}
                for b in self.bones:
                    P = pose_arm[b.name]
                    if b.parent:
                        local_b = pose_arm[b.parent.name].inverted_safe() @ P
                    else:
                        local_b = P
                    local_lt = mat_to_lt(local_b, self.o.scale)
                    loc, rot, _s = local_lt.decompose()
                    # decompose() of the conjugated matrix yields the LT
                    # rotation directly in (w,x,y,z); reorder to x y z w
                    tracks[b.name].append(
                        ((loc.x, loc.y, loc.z),
                         (rot.x, rot.y, rot.z, rot.w)))
                if explicit_times is not None:
                    times.append(int(round(explicit_times[fi])))
                else:
                    times.append(int(round((f - f_start) * 1000.0 / fps)))
        finally:
            ad.action = prev_action
            if prev_slot is not None and hasattr(ad, "action_slot"):
                try:
                    ad.action_slot = prev_slot
                except Exception:
                    pass
            # restore NLA state so the animation player keeps working
            for tr, m in nla_mute:
                try:
                    tr.mute = m
                except Exception:
                    pass
            if prev_use_nla is not None:
                try:
                    ad.use_nla = prev_use_nla
                except Exception:
                    pass
            scn.frame_set(prev_frame)

        # quaternion continuity per track
        for tk in tracks.values():
            for i in range(1, len(tk)):
                p, q = tk[i]
                qp = tk[i - 1][1]
                if sum(a * b for a, b in zip(q, qp)) < 0.0:
                    tk[i] = (p, tuple(-c for c in q))

        # Guarantee at least 2 keyframes only for a SYNTHESIZED base (no source
        # timing). Real imported actions keep their exact keyframe count to
        # match canonical ModelEdit output (e.g. baron's 1-frame 'base').
        if len(times) == 1 and explicit_times is None:
            times.append(times[0] + 200)
            for tk in tracks.values():
                tk.append(tk[0])

        return name, times, tracks, binding

    # -- writing ----------------------------------------------------------------

    def write(self, shapes, sockets, anims):
        w = LTAWriter(self.o.float_digits)
        model_name = self.o.model_name.strip() or \
            os.path.splitext(os.path.basename(self.filepath))[0]

        w.open('lt-model-0', w.s(model_name))

        # ---- on-load-cmds ----
        w.open('on-load-cmds')
        w.open()

        if anims:
            w.open('anim-bindings')
            w.open()
            for (aname, _times, _tracks, binding) in anims:
                w.open('anim-binding')
                w.leaf('name', w.s(aname))
                w.leaf('dims', w.vec(binding['dims']))
                w.leaf('translation', w.vec(binding['translation']))
                w.leaf('interp-time', str(int(binding['interp-time'])))
                if binding.get('weight-set'):
                    w.leaf('weight-set', w.s(binding['weight-set']))
                w.close()
            w.close()
            w.close()

        # node flags (from pose-bone custom props; default 0)
        flag_entries = []
        for b in self.bones:
            pb = self.arm_obj.pose.bones.get(b.name)
            flags = int(pb.get('lta_node_flags', 0)) if pb else 0
            flag_entries.append((b.name, flags))
        if self.o.write_node_flags:
            w.open('set-node-flags')
            w.open()
            for nm, fl in flag_entries:
                w.line('( %s %d )' % (w.s(nm), fl))
            w.close()
            w.close()

        for shape in shapes:
            if shape['deformer']:
                influences, weights = shape['deformer']
                w.open('add-deformer')
                w.open('skel-deformer', w.s(shape['name'] + "_deformer"))
                w.leaf('target', w.s(shape['name']))
                w.open('influences')
                w.line('( ' + ' '.join(w.s(n) for n in influences) + ' )')
                w.close()
                w.open('weightsets')
                w.open()
                for pairs in weights:
                    w.line('( ' + ' '.join(
                        '%d %s' % (i, w.f(wt)) for i, wt in pairs) + ' )')
                w.close()
                w.close()
                w.close()
                w.close()

        # set-repl-lod-original: ModelEdit's distance-LOD recipe. This is a
        # DISTANCE-LOD GENERATION instruction: on load, ModelEdit decimates
        # LOD 0 down to the tri-% targets. LT2.2 ModelEdit accepts it, but
        # Jupiter ModelEdit's BuildLODs can FAIL on it ("BuildLODs failed"),
        # blocking the load entirely. It is OPTIONAL -- without it the model
        # keeps LOD 0 only and loads everywhere. So: OFF by default, and never
        # synthesize when the source already carried a real recipe (that one is
        # re-emitted verbatim further below as the preserved 'lta_lod' block).
        has_preserved_lod = bool(self.arm_obj.get('lta_lod'))
        if (getattr(self.o, 'write_lod_recipe', False) and shapes
                and not has_preserved_lod):
            lodw = {}
            for o in self.mesh_objs:
                try:
                    lodw[o.name] = float(o.get('lt_lod_weight', 1.0) or 1.0)
                except Exception:
                    lodw[o.name] = 1.0
            w.open('set-repl-lod-original')
            w.open('tri-%')
            w.line('(%s )' % ' '.join(w.f(x) for x in
                   (0.799414, 0.600293, 0.399707, 0.250366)))
            w.close()
            w.open('dists')
            w.line('(%s )' % ' '.join(w.f(x) for x in
                   (300.0, 600.0, 1000.0, 1500.0)))
            w.close()
            w.open('piece-priorities')
            w.open()
            for shape in shapes:
                nm = shape['name']
                w.line('( %s %s )' % (w.s(nm),
                                      w.f(lodw.get(nm, 1.0))))
            w.close()
            w.close()
            w.close()

        if sockets:
            w.open('add-sockets')
            w.open()
            for (name, parent, pos, quat) in sockets:
                w.open('socket', w.s(name))
                w.leaf('parent', w.s(parent))
                w.leaf('pos', w.vec(pos))
                w.leaf('quat', w.vec(quat))
                w.close()
            w.close()
            w.close()

        radius = self.arm_obj.get('lta_global_radius',
                                  self.o.global_radius)
        w.leaf('set-global-radius', w.f(float(radius)))

        # round-trip preserved blocks captured by the importer
        if self.o.write_preserved:
            for key in ('lta_weightsets', 'lta_childmodels',
                        'lta_lod', 'lta_obb'):
                blob = self.arm_obj.get(key)
                if blob:
                    w.raw_block(str(blob))

        w.close()  # anonymous list
        w.close()  # on-load-cmds

        # ---- hierarchy ----
        w.open('hierarchy', w.s(model_name))
        w.open('children')
        w.open()
        roots = [b for b in self.bones if b.parent is None]
        for r in roots:
            self._write_transform(w, r)
        w.close()
        w.close()
        w.close()

        # ---- shapes ----
        for shape in shapes:
            self._write_shape(w, shape)

        # ---- animsets ----
        for (aname, times, tracks, _binding) in anims:
            self._write_animset(w, aname, times, tracks)

        # ---- tools-info ----
        seen = {}
        for shape in shapes:
            tex = shape['texture']
            idx = shape['material'].get('texture-index', 0)
            if tex and idx not in seen:
                seen[idx] = tex
        if seen:
            w.open('tools-info')
            w.open()
            w.open('texture-bindings')
            w.open()
            for idx in sorted(seen):
                w.line('( %d %s )' % (idx, w.s(seen[idx])))
            w.close()
            w.close()
            w.close()
            w.close()

        w.close()  # lt-model-0
        return w.text()

    def _write_transform(self, w, bone):
        m = mat_to_lt(bone.rest_arm, self.o.scale)
        w.open('transform', w.s(bone.name))
        w.open('matrix')
        w.open()
        for r in range(4):
            w.line(w.vec(m[r]))
        w.close()
        w.close()
        children = [b for b in self.bones if b.parent is bone]
        if children:
            w.open('children')
            w.open()
            for c in children:
                self._write_transform(w, c)
            w.close()
            w.close()
        w.close()

    def _write_shape(self, w, shape):
        w.open('shape', w.s(shape['name']))
        w.leaf('parent', w.s(shape['parent']))
        if shape['render_priority'] is not None:
            w.leaf('render-priority', str(int(shape['render_priority'])))

        w.open('geometry')
        w.open('mesh', w.s(shape['name']))

        w.open('vertex')
        w.open()
        for v in shape['verts']:
            w.line(w.vec(v))
        w.close()
        w.close()

        if shape['normals']:
            w.open('normals')
            w.open()
            for n in shape['normals']:
                w.line(w.vec(n))
            w.close()
            w.close()

        if shape['uvs']:
            w.open('uvs')
            w.open()
            for uv in shape['uvs']:
                w.line(w.vec(uv))
            w.close()
            w.close()

        if shape['colors']:
            w.open('colors')
            w.open()
            for c in shape['colors']:
                w.line(w.vec(c))
            w.close()
            w.close()

        self._write_faceset(w, 'tri-fs', shape['tri_fs'])
        if shape['tex_fs']:
            self._write_faceset(w, 'tex-fs', shape['tex_fs'])
        if shape['nrm_fs']:
            self._write_faceset(w, 'nrm-fs', shape['nrm_fs'])
        if shape['col_fs']:
            self._write_faceset(w, 'col-fs', shape['col_fs'])

        w.close()  # mesh
        w.close()  # geometry

        idx = int(shape['material'].get('texture-index', 0))
        if getattr(self.o, 'lta_dialect', 'lt22') == 'jupiter':
            # Jupiter: a texture-indices node directly under the shape.
            w.open('texture-indices')
            w.line('( %d )' % idx)
            w.close()
        else:
            # LT2.2: appearance > material > texture-index (singular).
            w.open('appearance')
            w.open('material', w.s(shape['material']['name']))
            w.leaf('texture-index', str(idx))
            diffuse = shape['material'].get('diffuse')
            if diffuse:
                w.leaf('diffuse', w.vec(diffuse[:4]))
            w.close()
            w.close()

        w.close()  # shape

    @staticmethod
    def _write_faceset(w, name, indices, per_line=30):
        w.open(name)
        for i in range(0, len(indices), per_line):
            chunk = indices[i:i + per_line]
            prefix = '( ' if i == 0 else '  '
            suffix = ' )' if i + per_line >= len(indices) else ''
            w.line(prefix + ' '.join(str(x) for x in chunk) + suffix)
        if not indices:
            w.line('( )')
        w.close()

    def _write_animset(self, w, name, times, tracks):
        w.open('animset', w.s(name))

        w.open('keyframe')
        w.open('keyframe', w.s(name))
        w.open('times')
        self._write_numbers(w, [str(t) for t in times])
        w.close()
        w.open('values')
        self._write_numbers(w, ['""'] * len(times))
        w.close()
        w.close()
        w.close()

        w.open('anims')
        w.open()
        for b in self.bones:
            track = tracks.get(b.name)
            if not track:
                continue
            # canonical ModelEdit format identifies the animated node via
            # (parent "name"); it does NOT name the (anim ...) block and does
            # NOT use (target ...). Writing target/name makes ModelEdit fail to
            # map tracks to nodes -> scrambled pose. Match canonical exactly.
            w.open('anim')
            w.leaf('parent', w.s(b.name))
            w.open('frames')
            w.open('posquat')
            w.open()
            for (pos, quat) in track:
                w.line('( %s %s )' % (w.vec(pos), w.vec(quat)))
            w.close()
            w.close()
            w.close()
            w.close()
        w.close()
        w.close()

        w.close()  # animset

    @staticmethod
    def _write_numbers(w, items, per_line=20):
        for i in range(0, len(items), per_line):
            chunk = items[i:i + per_line]
            prefix = '( ' if i == 0 else '  '
            suffix = ' )' if i + per_line >= len(items) else ''
            w.line(prefix + ' '.join(chunk) + suffix)
        if not items:
            w.line('( )')

    # -- run --------------------------------------------------------------------

    def run(self):
        t0 = time.time()
        self.find_objects()
        self.gather_bones()
        shapes = self.extract_meshes()
        sockets = self.extract_sockets() if self.o.export_sockets else []
        anims = []
        for action in self.collect_actions():
            anims.append(self.sample_action(action))
        text = self.write(shapes, sockets, anims)
        with open(self.filepath, 'w', encoding='ascii', errors='replace',
                  newline='\n') as f:
            f.write(text)
        self.op.report(
            {'INFO'},
            "Exported '%s': %d nodes, %d shapes, %d sockets, %d animations "
            "in %.2fs" % (os.path.basename(self.filepath), len(self.bones),
                          len(shapes), len(sockets), len(anims),
                          time.time() - t0))


# ---------------------------------------------------------------------------
# Operator
# ---------------------------------------------------------------------------

class EXPORT_SCENE_OT_lta_jupiter(Operator, ExportHelper):
    """Export a LithTech Jupiter .LTA model file (open it in ModelEdit and
    compile to .ltb with ModelPacker)"""
    bl_idname = "export_scene.lta_jupiter"
    bl_label = "Export LithTech LTA"
    bl_options = {'REGISTER', 'PRESET'}

    filename_ext = ".lta"
    filter_glob: StringProperty(default="*.lta", options={'HIDDEN'})

    # --- options ---
    model_name: StringProperty(
        name="Model Name",
        description="Identifier written to the lt-model-0 node "
                    "(file name is used when empty)",
        default="")

    lta_dialect: EnumProperty(
        name="LTA Dialect",
        description="Which LithTech LTA dialect to write. LT2.2 (NOLF1) nests "
                    "textures as appearance > material > texture-index; Jupiter "
                    "(NOLF2 etc.) writes a texture-indices node directly under "
                    "the shape. The skeleton, mesh, UVs, normals and animations "
                    "are identical in both. Preserved metadata blocks "
                    "(weight-sets / child-models) are re-emitted in their "
                    "source dialect either way",
        items=(('lt22',    "LT2.2 (NOLF1)",  "appearance > material > texture-index"),
               ('jupiter', "Jupiter (NOLF2)", "texture-indices under shape")),
        default='lt22')

    use_selection: BoolProperty(
        name="Selection Only",
        description="Export only selected objects (otherwise all visible "
                    "objects tied to the first armature)",
        default=False)

    scale: FloatProperty(
        name="Scale",
        description="Multiply Blender units by this factor to get LithTech "
                    "units (use the inverse of your import scale; NOLF2 "
                    "humans are ~75 units tall)",
        default=1.0, min=0.0001, max=10000.0)

    apply_modifiers: BoolProperty(
        name="Apply Modifiers",
        description="Apply mesh modifiers (the Armature modifier is "
                    "temporarily disabled so the bind pose is exported)",
        default=True)

    export_uvs: BoolProperty(name="Export UVs", default=True)

    export_normals: BoolProperty(
        name="Export Normals",
        description="Write per-vertex normals (the format LT2.2 and Jupiter "
                    "ModelEdit read by default)",
        default=True)

    indexed_normals: BoolProperty(
        name="Indexed Normals (nrm-fs)",
        description="Write de-duplicated normals plus an nrm-fs face set "
                    "instead of one normal per vertex. Per the LTA schema this "
                    "is only needed for stripped/indexed meshes; leave OFF for "
                    "normal models (ModelEdit shades per-vertex normals "
                    "correctly, indexed ones it can render dark)",
        default=False)

    export_colors: BoolProperty(
        name="Export Vertex Colors",
        description="Write the active color attribute as colors + col-fs",
        default=False)

    export_weights: BoolProperty(
        name="Export Skin Weights",
        description="Write a skel-deformer per skinned shape from vertex "
                    "groups matching bone names",
        default=True)

    max_weights: IntProperty(
        name="Max Weights per Vertex",
        description="Strongest influences kept per vertex (weights are "
                    "re-normalized). 4 matches what the LTB compiler "
                    "handles best; 0 = unlimited",
        default=4, min=0, max=16)

    export_sockets: BoolProperty(
        name="Export Sockets",
        description="Export empties parented to bones as sockets "
                    "(name prefix 's_' is stripped)",
        default=True)

    anim_mode: EnumProperty(
        name="Animations",
        items=(('ALL', "All Actions",
                "Export every action that animates pose bones as a "
                "separate animset"),
               ('ACTIVE', "Active Action",
                "Export only the armature's current action"),
               ('NONE', "None", "Do not export animations")),
        default='ALL')

    frame_step: IntProperty(
        name="Frame Step",
        description="Sample every Nth frame (1 = bake every frame)",
        default=1, min=1, max=10)

    add_base_anim: BoolProperty(
        name="Add 'base' Animation if None",
        description="ModelEdit requires at least one animation; write a "
                    "static 'base' animation from the current pose when "
                    "no actions are exported",
        default=True)

    write_node_flags: BoolProperty(
        name="Write Node Flags",
        description="Write set-node-flags from pose-bone 'lta_node_flags' "
                    "custom properties (defaults to 0)",
        default=True)

    write_lod_recipe: BoolProperty(
        name="Synthesize LOD Recipe",
        description="Emit a set-repl-lod-original distance-LOD recipe built "
                    "from each piece's lod_weight. LT2.2 ModelEdit rebuilds "
                    "LODs from it, but Jupiter ModelEdit's BuildLODs can FAIL "
                    "on it and refuse to load. Leave OFF unless you target "
                    "LT2.2 and want the regenerated LOD chain. A real recipe "
                    "from an LTA source is always preserved regardless",
        default=False)

    write_preserved: BoolProperty(
        name="Write Preserved LTA Blocks",
        description="Re-emit anim-weightsets, child models, LOD and OBB "
                    "blocks stored on the armature by the importer "
                    "(round-trip)",
        default=True)

    global_radius: FloatProperty(
        name="Global Radius",
        description="set-global-radius value (visibility radius) used when "
                    "the armature has no 'lta_global_radius' property",
        default=96.0, min=0.0)

    float_digits: IntProperty(
        name="Float Precision",
        description="Decimal places written for floats",
        default=6, min=3, max=9)

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True
        layout.use_property_decorate = False

        box = layout.box()
        box.label(text="General", icon='EXPORT')
        box.prop(self, "model_name")
        box.prop(self, "lta_dialect")
        box.prop(self, "use_selection")
        box.prop(self, "scale")
        box.prop(self, "float_digits")

        box = layout.box()
        box.label(text="Geometry", icon='MESH_DATA')
        box.prop(self, "apply_modifiers")
        box.prop(self, "export_uvs")
        box.prop(self, "export_normals")
        row = box.row()
        row.enabled = self.export_normals
        row.prop(self, "indexed_normals")
        box.prop(self, "export_colors")
        box.prop(self, "export_weights")
        sub = box.column()
        sub.enabled = self.export_weights
        sub.prop(self, "max_weights")

        box = layout.box()
        box.label(text="Animation", icon='ARMATURE_DATA')
        box.prop(self, "anim_mode")
        sub = box.column()
        sub.enabled = self.anim_mode != 'NONE'
        sub.prop(self, "frame_step")
        box.prop(self, "add_base_anim")

        box = layout.box()
        box.label(text="LithTech Extras", icon='TOOL_SETTINGS')
        box.prop(self, "export_sockets")
        box.prop(self, "write_node_flags")
        box.prop(self, "write_preserved")
        box.prop(self, "write_lod_recipe")
        box.prop(self, "global_radius")

    def execute(self, context):
        if context.object and context.object.mode != 'OBJECT':
            bpy.ops.object.mode_set(mode='OBJECT')
        try:
            LTAExporter(self, context, self.filepath, self).run()
        except ExportError as ex:
            self.report({'ERROR'}, str(ex))
            return {'CANCELLED'}
        except Exception as ex:
            import traceback
            traceback.print_exc()
            self.report({'ERROR'}, "Unexpected error: %s" % ex)
            return {'CANCELLED'}
        return {'FINISHED'}


def menu_func_export(self, context):
    self.layout.operator(EXPORT_SCENE_OT_lta_jupiter.bl_idname,
                         text="LithTech Model (.lta)")


classes = (
    EXPORT_SCENE_OT_lta_jupiter,
)


def register():
    for c in classes:
        bpy.utils.register_class(c)
    bpy.types.TOPBAR_MT_file_export.append(menu_func_export)


def unregister():
    bpy.types.TOPBAR_MT_file_export.remove(menu_func_export)
    for c in reversed(classes):
        bpy.utils.unregister_class(c)


if __name__ == "__main__":
    register()
