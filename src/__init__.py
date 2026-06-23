# -*- coding: utf-8 -*-
"""
io_scene_lithtech_clean
=======================
Saubere LithTech-Import-Pipeline fuer Blender, eine IR + eine
Konvertierungsgrenze:

    .abc / .ltb  ->  reader_dispatch  ->  abc.py:Model (roh LithTech)
                     builder_import + coordinates.swap_*  ->  Blender

Keine 90-Grad-Drehung, kein scale.z=-1. Alle Koordinaten werden einmalig an
der Datengrenze konvertiert; Objekte haben Identity-Transform.

STATISCHE STUFE: Skelett + Mesh + Normalen + UVs + Weights + Sockets.
(Animation folgt als eigenes Modul animation_import.py.)

Installation:  Edit > Preferences > Add-ons > Install... > diese ZIP waehlen,
dann Haken setzen.  Danach: File > Import > LithTech Model (.abc/.ltb/.lta).
"""

bl_info = {
    "name": "LithTech Model Import",
    "author": "StenApp",
    "version": (1, 0, 0),
    "blender": (4, 1, 0),
    "location": "File > Import > LithTech Model (.abc/.ltb/.lta)",
    "description": "Import NOLF ABC PC / LTB PC / LTB PS2 models via the swap_* convention",
    "category": "Import-Export",
}

import importlib

import bpy
from bpy.props import StringProperty, BoolProperty
from bpy_extras.io_utils import ImportHelper

# submodules (reload-safe on re-enable)
from . import coordinates
from . import reader_dispatch
from . import builder_import
from . import animation_import
from . import exporter_lta
from . import ui_anim

for _m in (coordinates, reader_dispatch, builder_import, animation_import, exporter_lta, ui_anim):
    importlib.reload(_m)


class IMPORT_OT_lithtech_clean(bpy.types.Operator, ImportHelper):
    """Import a LithTech model (ABC PC / LTB PC / LTB PS2)"""
    bl_idname = "import_scene.lithtech_clean"
    bl_label = "Import LithTech Model"
    bl_options = {'REGISTER', 'UNDO'}

    filename_ext = ".abc"
    filter_glob: StringProperty(default="*.abc;*.ltb;*.lta", options={'HIDDEN'})
    import_anims: BoolProperty(
        name="Import Animations",
        description="Import each animation as its own Action on a separate NLA track",
        default=True,
    )

    def execute(self, context):
        import os
        try:
            model = reader_dispatch.read_model(self.filepath)
        except Exception as e:
            self.report({'ERROR'}, "Read failed: %s" % e)
            return {'CANCELLED'}
        try:
            name = os.path.splitext(os.path.basename(self.filepath))[0]
            arm_obj = builder_import.build_model(model, name)
        except Exception as e:
            self.report({'ERROR'}, "Build failed: %s" % e)
            return {'CANCELLED'}

        n_anims = 0
        if self.import_anims:
            try:
                n_anims = animation_import.import_animations(model, arm_obj)
            except Exception as e:
                self.report({'WARNING'}, "Animations failed: %s" % e)

        self.report({'INFO'}, "Imported %s (%d bones, %d pieces, %d anims)"
                    % (name, len(model.nodes), len(model.pieces), n_anims))
        return {'FINISHED'}


def menu_func_import(self, context):
    self.layout.operator(IMPORT_OT_lithtech_clean.bl_idname,
                         text="LithTech Model (.abc/.ltb/.lta)")


def register():
    bpy.utils.register_class(IMPORT_OT_lithtech_clean)
    bpy.types.TOPBAR_MT_file_import.append(menu_func_import)
    exporter_lta.register()
    ui_anim.register()


def unregister():
    ui_anim.unregister()
    exporter_lta.unregister()
    bpy.types.TOPBAR_MT_file_import.remove(menu_func_import)
    bpy.utils.unregister_class(IMPORT_OT_lithtech_clean)


if __name__ == "__main__":
    register()
