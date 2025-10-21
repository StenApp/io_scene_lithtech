"""
Animation UI Panel für Lithtech Tools
Ort: io_scene_lithtech/animation_ui.py
"""

import bpy
from bpy.props import IntProperty

class LITHTECH_OT_check_animations(bpy.types.Operator):
    """Debug: Check if animations were imported"""
    bl_idname = "lithtech.check_animations"
    bl_label = "Check Animations"
    bl_description = "List all available animations in the console"
    
    def execute(self, context):
        actions = list(bpy.data.actions)
        print(f"\n{'='*50}")
        print(f"LITHTECH ANIMATION CHECK")
        print(f"{'='*50}")
        print(f"Total Actions Found: {len(actions)}\n")
        
        if len(actions) == 0:
            print("⚠️  No animations found!")
            print("Make sure to enable 'Import Animations' when importing LTB files.")
        else:
            for i, action in enumerate(actions):
                frames = int(action.frame_range[1] - action.frame_range[0])
                print(f"  [{i}] {action.name}")
                print(f"      Frames: {frames} | Range: {action.frame_range[0]:.0f}-{action.frame_range[1]:.0f}")
        
        print(f"{'='*50}\n")
        self.report({'INFO'}, f"Found {len(actions)} animations (check console)")
        return {'FINISHED'}


class LITHTECH_OT_play_animation(bpy.types.Operator):
    """Play selected animation"""
    bl_idname = "lithtech.play_animation"
    bl_label = "Play Animation"
    
    animation_index: IntProperty(description="Index of animation to play")
    
    def execute(self, context):
        obj = context.active_object
        if not obj or obj.type != 'ARMATURE':
            self.report({'ERROR'}, "Select an Armature object")
            return {'CANCELLED'}
        
        actions = list(bpy.data.actions)
        
        if not actions:
            self.report({'ERROR'}, "No animations available")
            return {'CANCELLED'}
        
        if self.animation_index >= len(actions):
            self.report({'ERROR'}, "Animation index out of range")
            return {'CANCELLED'}
        
        action = actions[self.animation_index]
        
        # Erstelle animation_data falls nicht vorhanden
        if obj.animation_data is None:
            obj.animation_data_create()
        
        # Setze die Action
        obj.animation_data.action = action
        
        # Berechne Frame Range aus der Action
        frame_start = int(action.frame_range[0])
        frame_end = int(action.frame_range[1])
        
        # Setze Timeline
        context.scene.frame_start = frame_start
        context.scene.frame_end = frame_end
        context.scene.frame_set(frame_start)
        
        # Starte Playback
        if not context.screen.is_animation_playing:
            bpy.ops.screen.animation_play()
        
        self.report({'INFO'}, f"Playing: {action.name} ({frame_end - frame_start} frames)")
        return {'FINISHED'}


class LITHTECH_OT_setup_nla_tracks(bpy.types.Operator):
    """Create NLA editor tracks for all animations"""
    bl_idname = "lithtech.setup_nla_tracks"
    bl_label = "Setup NLA Tracks"
    bl_description = "Organize all animations as NLA tracks (open NLA Editor to view)"
    
    def execute(self, context):
        obj = context.active_object
        if not obj or obj.type != 'ARMATURE':
            self.report({'ERROR'}, "Select an Armature object")
            return {'CANCELLED'}
        
        if obj.animation_data is None:
            obj.animation_data_create()
        
        # Leere existing tracks
        nla_tracks = obj.animation_data.nla_tracks
        for track in nla_tracks:
            nla_tracks.remove(track)
        
        actions = list(bpy.data.actions)
        
        if not actions:
            self.report({'ERROR'}, "No animations to add")
            return {'CANCELLED'}
        
        for action in actions:
            track = nla_tracks.new()
            track.name = action.name
            
            frame_start = int(action.frame_range[0])
            strip = track.strips.new(action.name, frame_start, action)
        
        self.report({'INFO'}, f"Created {len(actions)} NLA tracks")
        return {'FINISHED'}


class LITHTECH_PT_animation_panel(bpy.types.Panel):
    """Animation manager panel in properties"""
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
        
        # === STATUS ===
        box = layout.box()
        box.label(text="Status", icon='INFO')
        
        current_action = None
        if obj.animation_data and obj.animation_data.action:
            current_action = obj.animation_data.action
            row = box.row()
            row.label(text=f"Playing: {current_action.name}", icon='PLAY')
        else:
            box.label(text="No animation active", icon='BLANK1')
        
        actions = list(bpy.data.actions)
        row = box.row()
        row.label(text=f"Available: {len(actions)}", icon='ACTION')
        
        # === NO ANIMATIONS ===
        if len(actions) == 0:
            box = layout.box()
            box.label(text="No animations found!", icon='ERROR')
            box.label(text="Check import options:", icon='INFO')
            box.label(text="1. Import LTB file")
            box.label(text="2. Enable 'Import Animations'")
            row = box.row()
            row.operator("lithtech.check_animations", icon='CONSOLE')
            return
        
        # === ANIMATION SELECTOR ===
        box = layout.box()
        box.label(text="Animations", icon='SEQUENCE')
        
        col = box.column(align=True)
        for index, action in enumerate(actions):
            frames = int(action.frame_range[1] - action.frame_range[0])
            icon = 'PLAY' if action == current_action else 'BLANK1'
            
            row = col.row(align=True)
            op = row.operator("lithtech.play_animation", text=action.name, icon=icon)
            op.animation_index = index
            
            row.label(text=f"{frames}f")
        
        # === PLAYBACK ===
        layout.separator()
        box = layout.box()
        box.label(text="Playback", icon='TIME')
        
        row = box.row(align=True)
        if context.screen.is_animation_playing:
            row.operator("screen.animation_play", text="Pause", icon='PAUSE')
        else:
            row.operator("screen.animation_play", text="Play", icon='PLAY')
        
        # === ADVANCED ===
        layout.separator()
        box = layout.box()
        box.label(text="Advanced", icon='MODIFIER')
        
        row = box.row()
        row.operator("lithtech.setup_nla_tracks", icon='NLA')
        row.label(text="(NLA Editor)", icon='INFO')
        
        # === DEBUG ===
        layout.separator()
        row = layout.row()
        row.operator("lithtech.check_animations", text="Debug: Check Animations", icon='CONSOLE')


def register():
    """Register animation UI classes"""
    bpy.utils.register_class(LITHTECH_OT_check_animations)
    bpy.utils.register_class(LITHTECH_OT_play_animation)
    bpy.utils.register_class(LITHTECH_OT_setup_nla_tracks)
    bpy.utils.register_class(LITHTECH_PT_animation_panel)


def unregister():
    """Unregister animation UI classes"""
    bpy.utils.unregister_class(LITHTECH_PT_animation_panel)
    bpy.utils.unregister_class(LITHTECH_OT_setup_nla_tracks)
    bpy.utils.unregister_class(LITHTECH_OT_play_animation)
    bpy.utils.unregister_class(LITHTECH_OT_check_animations)