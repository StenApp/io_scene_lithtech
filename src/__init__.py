bl_info = {
    'name': 'Lithtech Tools',
    'description': 'Import and export various Lithtech models and animations files.',
    'author': 'Colin Basnett and HeyJake',
    'version': (1, 1, 0),
    'blender': (2, 80, 0),
    'location': 'File > Import-Export',
    'warning': 'This add-on is under development.',
    'wiki_url': 'https://github.com/haekb/io_scene_lithtech/wiki',
    'tracker_url': 'https://github.com/haekb/io_scene_lithtech/issues',
    'support': 'COMMUNITY',
    'category': 'Import-Export'
}

if 'bpy' in locals():
    import importlib
    if 's3tc'               in locals(): importlib.reload(s3tc)
    if 'dxt'                in locals(): importlib.reload(dtx)
    if 'abc'                in locals(): importlib.reload(abc)
    if 'builder'            in locals(): importlib.reload(builder)
    if 'reader_abc_pc'      in locals(): importlib.reload(reader_abc_pc)
    if 'reader_ltb_ps2'     in locals(): importlib.reload(reader_ltb_ps2)
    if 'writer_abc_pc'      in locals(): importlib.reload(writer_abc_pc)
    if 'importer'           in locals(): importlib.reload(importer)
    if 'exporter'           in locals(): importlib.reload(exporter)
    if 'converter'          in locals(): importlib.reload(converter)

import bpy
from . import s3tc
from . import dtx
from . import abc
from . import builder
from . import reader_abc_pc
from . import reader_ltb_ps2
from . import writer_abc_pc
from . import importer
from . import exporter
from . import converter

from bpy.utils import register_class, unregister_class

classes = (
    importer.ImportOperatorABC,
    importer.ImportOperatorLTB,
    exporter.ExportOperatorABC,
    converter.ConvertLTBToABC,
)

def register():
    for cls in classes:
        register_class(cls)

    # Import options
    bpy.types.TOPBAR_MT_file_import.append(importer.ImportOperatorABC.menu_func_import)
    bpy.types.TOPBAR_MT_file_import.append(importer.ImportOperatorLTB.menu_func_import)

    # Export options
    bpy.types.TOPBAR_MT_file_export.append(exporter.ExportOperatorABC.menu_func_export)


def unregister():
    for cls in reversed(classes):
        unregister_class(cls)

    # Import options
    bpy.types.TOPBAR_MT_file_import.remove(importer.ImportOperatorABC.menu_func_import)
    bpy.types.TOPBAR_MT_file_import.remove(importer.ImportOperatorLTB.menu_func_import)

    # Export options
    bpy.types.TOPBAR_MT_file_export.remove(exporter.ExportOperatorABC.menu_func_export)

