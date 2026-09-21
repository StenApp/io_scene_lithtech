# -*- coding: utf-8 -*-
"""
animation_import.py  --  ONE job: turn model.animations into Blender Actions,
each pushed onto its OWN NLA track (NLA-ready). Additive to the static build.

Keyframe transforms are LOCAL poses relative to the parent node (as the readers
produce them). Per bone/keyframe:

    matrix_basis = rest_rel^-1 @ swap_matrix(L_local)
                 = rest_rel^-1 @ LocRotScale(swap_vec(loc), swap_quat(rot))

decomposed into location + rotation_quaternion and keyed. Because swap_* is
multiplicative + self-inverse, converting each LOCAL transform once is enough;
Blender composes the hierarchy correctly. Verified on real hero_action data:
the 'base' frame-0 rotation matches the bind pose exactly (no spurious flip).

Quaternion sign continuity is enforced per bone to avoid 360-degree interpolation
spins between keyframes.
"""

import bpy
from mathutils import Matrix, Quaternion

try:
    from .coordinates import swap_vec, swap_quat
except ImportError:
    from coordinates import swap_vec, swap_quat


_FPS_FALLBACK = 30


def _frame_for(time_ms, fps):
    return round(time_ms * fps / 1000.0)


def _bind_action_slot(action, id_data):
    """Blender 4.4+ split Action into 'slots': one Action can now carry
    several independent animations, each bound to one ID. Without binding a
    slot -- on animation_data.action_slot AND separately on every NLA strip
    that uses this action -- the action exists and looks fine in the data-
    blocks list, but does not actually drive anything; Blender silently
    shows no animation, no error. Pre-4.4 Blender (this addon's stated
    minimum is 4.1) has no Action.slots at all, so this is a no-op there.

    UNVERIFIED against a real Blender 4.4+ install (not available in this
    environment) -- API names (Action.slots.new(id_type=..., name=...),
    id_type='OBJECT', .action_slot on animation_data/NLA strip) are best-
    effort from the documented 4.4 Action/slot redesign, not run-tested.
    Defensive by design: every step is guarded, so on an API mismatch this
    degrades to the pre-4.4 behaviour (action still created, just possibly
    unbound) instead of raising.
    """
    slots = getattr(action, 'slots', None)
    if slots is None:
        return None  # pre-4.4 Blender: actions don't have slots at all
    id_type = getattr(id_data, 'id_type', 'OBJECT')
    slot = None
    try:
        for s in slots:
            if getattr(s, 'target_id_type', id_type) == id_type:
                slot = s
                break
        if slot is None:
            slot = slots.new(id_type=id_type, name=id_data.name)
    except Exception as e:
        print("  [slot] could not get/create action slot for '%s': %s"
              % (action.name, e))
        return None
    return slot


def _set_action_slot(target, slot, label):
    """Assign `slot` to animation_data.action_slot or an NLA strip's
    action_slot. Both assignment points are needed (see _bind_action_slot);
    guarded the same way, for the same reason."""
    if slot is None:
        return
    try:
        target.action_slot = slot
    except Exception as e:
        print("  [slot] could not bind action_slot on %s: %s" % (label, e))


def import_animations(model, arm_obj, fps=None):
    anims = getattr(model, 'animations', [])
    if not anims:
        print("animation_import: no animations in model")
        return 0

    if fps is None:
        fps = bpy.context.scene.render.fps or _FPS_FALLBACK

    if arm_obj.animation_data is None:
        arm_obj.animation_data_create()
    ad = arm_obj.animation_data

    bones = arm_obj.data.bones
    pbs = arm_obj.pose.bones

    # quaternion rotation mode for all posed bones
    for nd in model.nodes:
        if nd.name in pbs:
            pbs[nd.name].rotation_mode = 'QUATERNION'

    # rest transform relative to parent, Blender armature space
    rest_rel = {}
    for nd in model.nodes:
        if nd.name not in bones:
            continue
        bl = bones[nd.name].matrix_local
        if nd.parent is not None and nd.parent.name in bones:
            rest_rel[nd.name] = bones[nd.parent.name].matrix_local.inverted() @ bl
        else:
            rest_rel[nd.name] = bl

    # anim-binding metadata (dims/translation) by animation name, for round-trip
    bindings = {}
    for b in getattr(model, 'anim_bindings', []) or []:
        ext = getattr(b, 'extents', None)
        org = getattr(b, 'origin', None)
        bindings[b.name] = (
            list(ext) if ext is not None else None,
            list(org) if org is not None else None,
        )

    made = 0
    for anim in anims:
        # anim.keyframes ist bei JEDEM Reader garantiert befuellt; das separate
        # Attribut anim.keyframe_count wird nur von manchen Readern (LTB PC/PS2/
        # DHNP, ABC PC, LTA) explizit gesetzt -- der ABC-v6-Reader setzt es nie
        # (er braucht dafuer nur eine lokale Variable beim Parsen), wodurch
        # anim.keyframe_count fuer v6-Modelle mit AttributeError abbrach
        # ("Animations failed: 'Animation' object has no attribute
        # 'keyframe_count'"). len(anim.keyframes) ist reader-unabhaengig korrekt.
        if not anim.keyframes:
            continue

        action = bpy.data.actions.new(name=anim.name)
        action.use_fake_user = True            # survive even when not active
        slot = _bind_action_slot(action, arm_obj)
        ad.action = action
        _set_action_slot(ad, slot, "animation_data (action '%s')" % anim.name)

        # stash anim-binding props the (v1) exporter reads back; defaults match v1
        dims, trans = bindings.get(anim.name, (None, None))
        action['lta_dims'] = dims if dims else [16.0, 16.0, 16.0]
        action['lta_translation'] = trans if trans else [0.0, 0.0, 0.0]
        itime = getattr(anim, 'interpolation_time', None)
        action['lta_interp_time'] = int(itime) if itime else 200
        # store the exact original keyframe times (ms); the exporter samples
        # only these instead of every frame, so the round-trip keeps the
        # original keyframe count/timing (e.g. 161, not 385).
        action['lta_keyframe_times'] = [float(kf.time) for kf in anim.keyframes]
        # per-keyframe command/frame strings (e.g. "cmd msg c2cam2 trigger"),
        # parallel to lta_keyframe_times by index. These have no Blender
        # F-Curve representation, so they would otherwise silently vanish the
        # moment a mesh edit forces a re-export -- stash them as a custom
        # property on the action so they survive untouched.
        action['lta_keyframe_strings'] = [kf.string or '' for kf in anim.keyframes]

        prev_q = {}                            # per-bone quaternion continuity
        for ki, kf in enumerate(anim.keyframes):
            frame = _frame_for(kf.time, fps)
            for ni, nd in enumerate(model.nodes):
                if nd.name not in pbs:
                    continue
                t = anim.node_keyframe_transforms[ni][ki]
                L_b = Matrix.LocRotScale(swap_vec(t.location), swap_quat(t.rotation), None)
                basis = rest_rel[nd.name].inverted() @ L_b
                loc, rot, _ = basis.decompose()

                # keep quaternion on the same hemisphere as the previous key
                pq = prev_q.get(nd.name)
                if pq is not None and pq.dot(rot) < 0.0:
                    rot.negate()
                prev_q[nd.name] = rot.copy()

                pb = pbs[nd.name]
                pb.location = loc
                pb.rotation_quaternion = rot
                pb.keyframe_insert('location', frame=frame, group=nd.name)
                pb.keyframe_insert('rotation_quaternion', frame=frame, group=nd.name)

        # Visible reference markers for non-empty frame strings (e.g.
        # "cmd msg c2cam2 trigger") so they show up on the action's timeline
        # too, not just in the hidden custom property above.
        for ki, kf in enumerate(anim.keyframes):
            if kf.string:
                try:
                    pm = action.pose_markers.new(name=kf.string[:63])
                    pm.frame = _frame_for(kf.time, fps)
                except Exception:
                    pass

        # One NLA track per animation, MUTED by default. The LithTech
        # Animations panel selects one at a time (solo) -- NLA-based playback,
        # no overlap. Tracks stay visible/usable in the NLA editor.
        try:
            start = int(action.frame_range[0])
            track = ad.nla_tracks.new()
            track.name = anim.name
            strip = track.strips.new(anim.name, start, action)
            _set_action_slot(strip, slot, "NLA strip '%s'" % anim.name)
            track.mute = True
        except Exception as e:
            print("  [nla] '%s' strip not created: %s" % (anim.name, e))
        ad.action = None
        made += 1

    # Leave the armature in its REST pose, not the last frame of the last
    # animation. The keyframing loop above sets pb.location / rotation as a
    # side effect; with ad.action=None and all NLA tracks muted, those basis
    # values would otherwise persist and the viewport would show the last
    # sampled pose statically. Clear every pose bone back to identity.
    ident = Quaternion((1.0, 0.0, 0.0, 0.0))
    for pb in pbs:
        pb.location = (0.0, 0.0, 0.0)
        pb.rotation_quaternion = ident
        pb.scale = (1.0, 1.0, 1.0)

    print("=" * 56)
    print("ANIMATIONS: %d actions imported as fake-user Actions (fps=%d)" % (made, fps))
    print("=" * 56)
    return made
