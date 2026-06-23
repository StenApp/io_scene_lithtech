"""
LithTech Animations panel.

Simple list selector that plays animations via NLA: each imported animation is
its own NLA track (muted by default). Clicking an entry SOLOS that track
(un-mutes it, mutes the others) and sets the frame range -- so exactly one
animation plays at a time, NLA-based, no overlap.

Layout follows the original io_scene_lithtech player (Properties > Object tab,
boxed clickable list).
"""
import bpy
from bpy.props import StringProperty


def _tracks(obj):
    if obj and obj.type == 'ARMATURE' and obj.animation_data:
        return list(obj.animation_data.nla_tracks)
    return []


def _solo(obj, track_name, scene):
    ad = obj.animation_data
    ad.action = None  # active action would add on top of the NLA result
    target = None
    for tr in ad.nla_tracks:
        tr.mute = (tr.name != track_name)
        if tr.name == track_name:
            target = tr
    if target is not None and target.strips:
        s = target.strips[0]
        f0 = int(round(s.frame_start))
        f1 = int(round(s.frame_end))
        # fall back to the action's own keyframe range if the strip looks off
        if f1 <= f0 and getattr(s, "action", None):
            f0 = int(round(s.action.frame_range[0]))
            f1 = int(round(s.action.frame_range[1]))
        scene.frame_start = f0
        scene.frame_end = max(f1, f0 + 1)
        scene.use_preview_range = False   # so playback honors start/end, not a stale preview range
        scene.frame_set(f0)
    return target is not None


class LITHTECH_OT_play_animation(bpy.types.Operator):
    """Play this animation (solo its NLA track)"""
    bl_idname = "lithtech.play_animation"
    bl_label = "Play Animation"
    bl_options = {'REGISTER', 'UNDO'}

    track_name: StringProperty()

    def execute(self, context):
        obj = context.active_object
        if not obj or obj.type != 'ARMATURE' or not obj.animation_data:
            self.report({'ERROR'}, "Select a LithTech armature")
            return {'CANCELLED'}
        if not _solo(obj, self.track_name, context.scene):
            self.report({'ERROR'}, "Animation track not found")
            return {'CANCELLED'}
        if not context.screen.is_animation_playing:
            bpy.ops.screen.animation_play()
        self.report({'INFO'}, "Playing: %s" % self.track_name)
        return {'FINISHED'}


class LITHTECH_OT_register_animation(bpy.types.Operator):
    """Register the armature's active Action as a LithTech animation so the LTA
    exporter writes it. This pushes the active Action down onto its own NLA
    track (the exporter only finds animations that are on a track or active).
    Author the Action first in an Action Editor, then press this."""
    bl_idname = "lithtech.register_animation"
    bl_label = "Register New Animation"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        obj = context.active_object
        if not obj or obj.type != 'ARMATURE':
            self.report({'ERROR'}, "Select the armature object first.")
            return {'CANCELLED'}
        ad = obj.animation_data or obj.animation_data_create()
        action = ad.action
        if action is None:
            self.report({'ERROR'}, "No active Action. Open an Action Editor, "
                         "create/select the Action you authored, then press "
                         "this.")
            return {'CANCELLED'}
        # already pushed down?
        for tr in ad.nla_tracks:
            for st in tr.strips:
                if getattr(st, 'action', None) is action:
                    ad.action = None
                    self.report({'WARNING'},
                                "'%s' is already registered." % action.name)
                    return {'CANCELLED'}
        f0 = int(round(action.frame_range[0]))
        track = ad.nla_tracks.new()
        track.name = action.name
        try:
            track.strips.new(action.name, f0, action)
        except Exception as e:
            ad.nla_tracks.remove(track)
            self.report({'ERROR'}, "Push-down failed: %s" % e)
            return {'CANCELLED'}
        track.mute = True            # consistent with imported tracks
        ad.action = None             # now lives only on the track
        self.report({'INFO'}, "Registered '%s' - it will be exported. Set its "
                     "collision dims in ModelEdit afterwards." % action.name)
        return {'FINISHED'}


class LITHTECH_OT_rest_pose(bpy.types.Operator):
    """Mute all animation tracks (show the bind/rest pose)"""
    bl_idname = "lithtech.rest_pose"
    bl_label = "Rest Pose"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        obj = context.active_object
        if not obj or obj.type != 'ARMATURE' or not obj.animation_data:
            return {'CANCELLED'}
        for tr in obj.animation_data.nla_tracks:
            tr.mute = True
        obj.animation_data.action = None
        return {'FINISHED'}


class LITHTECH_PT_animation_panel(bpy.types.Panel):
    """Animation selector panel"""
    bl_label = "LithTech Animations"
    bl_idname = "LITHTECH_PT_animation_panel"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "object"

    @classmethod
    def poll(cls, context):
        return context.active_object and context.active_object.type == 'ARMATURE'

    def draw(self, context):
        layout = self.layout
        obj = context.active_object
        tracks = _tracks(obj)

        box = layout.box()
        box.label(text="Status", icon='INFO')
        playing = next((t.name for t in tracks if not t.mute), None)
        if playing:
            box.label(text="Playing: %s" % playing, icon='PLAY')
        else:
            box.label(text="Rest pose (none active)", icon='BLANK1')
        box.label(text="Available: %d" % len(tracks), icon='ACTION')

        # Authoring: register a newly authored Action for export (always shown,
        # also when the rig has no animations yet).
        box = layout.box()
        box.label(text="Authoring", icon='ACTION_TWEAK')
        box.operator("lithtech.register_animation",
                     text="Register New Animation", icon='PLUS')
        box.label(text="Pushes the active Action to a", icon='BLANK1')
        box.label(text="track so the LTA export finds it.", icon='BLANK1')

        if not tracks:
            b = layout.box()
            b.label(text="No animations found!", icon='ERROR')
            b.label(text="Import with 'Import Animations' enabled,")
            b.label(text="or author one and Register it above.")
            return

        box = layout.box()
        box.label(text="Animations", icon='SEQUENCE')
        col = box.column(align=True)
        for tr in tracks:
            is_on = not tr.mute
            frames = 0
            if tr.strips:
                s = tr.strips[0]
                frames = int(s.frame_end - s.frame_start)
            row = col.row(align=True)
            row.operator("lithtech.play_animation", text=tr.name,
                         icon='PLAY' if is_on else 'BLANK1',
                         depress=is_on).track_name = tr.name
            row.label(text="%df" % frames)

        layout.separator()
        box = layout.box()
        box.label(text="Playback", icon='TIME')
        row = box.row(align=True)
        if context.screen.is_animation_playing:
            row.operator("screen.animation_play", text="Pause", icon='PAUSE')
        else:
            row.operator("screen.animation_play", text="Play", icon='PLAY')
        box.operator("lithtech.rest_pose", text="Rest Pose", icon='ARMATURE_DATA')


_classes = (
    LITHTECH_OT_play_animation,
    LITHTECH_OT_register_animation,
    LITHTECH_OT_rest_pose,
    LITHTECH_PT_animation_panel,
)


def register():
    for c in _classes:
        bpy.utils.register_class(c)


def unregister():
    for c in reversed(_classes):
        try:
            bpy.utils.unregister_class(c)
        except Exception:
            pass
