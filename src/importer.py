import bpy
import bpy_extras
import bmesh
import os
import math
from math import pi, ceil
from mathutils import Vector, Matrix, Quaternion, Euler
from bpy.props import StringProperty, BoolProperty, FloatProperty
from .dtx import DTX
from .utils import show_message_box, get_framerate

# Format imports
from .reader_abc_v6_pc import ABCV6ModelReader
from .reader_abc_pc import ABCModelReader
from .reader_ltb_pc import PCLTBModelReader
from .reader_ltb_ps2 import PS2LTBModelReader

from . import utils

# Model Types (for rigid mesh detection)
MT_RIGID = 4
MT_SKELETAL = 5
MT_VERTEX_ANIMATED = 6

class ModelImportOptions(object):
    def __init__(self):
        self.should_merge_duplicate_verts = False
        self.should_import_animations = False
        self.should_import_sockets = False
        self.bone_length_min = 0.1
        self.should_import_lods = False
        self.should_merge_pieces = False
        self.images = []


def import_model(model, options):
    try:
        #delete all old actions (animation player)
        if options.should_import_animations:
            for action in list(bpy.data.actions):
                bpy.data.actions.remove(action)
                
        # utils.delete_all_objects()

        # Validate model before proceeding
        if not model.pieces:
            show_message_box("The model contains no pieces to import.", "Empty Model", 'ERROR')
            return {'CANCELLED'}

        # This seems to be the rage these days
        Context = bpy.context
        Data = bpy.data
        Ops = bpy.ops
        Types = bpy.types

        # Create our new collection. This will help us later on..
        collection = Data.collections.new(model.name)
        # Add our collection to the scene
        Context.scene.collection.children.link(collection)

        # TODO: clear the orphan list for ultra name purity
        sun = Data.lights.new(name="Sun", type='SUN')
        sun_object = Data.objects.new("Sun", sun)
        collection.objects.link(sun_object)

        # Create the armature
        armature = bpy.data.armatures.new(model.name)
        armature_object = bpy.data.objects.new(model.name, armature)
        armature_object.data.display_type = 'STICK'
        armature_object.show_in_front = True

        collection.objects.link(armature_object)
        armature_object.select_set(True)
        #armature_object.edit_bones()

        Context.view_layer.objects.active = armature_object
        Ops.object.mode_set(mode='EDIT')

        # Process nodes only if there are any
        if model.nodes:
            for node in model.nodes:
                try:
                    bone = armature.edit_bones.new(node.name)
                    '''
                    We can be assured that the parent will always be found because
                    the node list is in the same order as a depth-first traversal of the
                    node hierarchy.
                    '''
                    try:
                        bone.parent = armature.edit_bones[node.parent.name] if node.parent else None
                    except (KeyError, AttributeError) as e:
                        print(f"Warning: Could not find parent for bone {node.name}: {e}")
                        
                    # Apply our bind matrix with proper tail and roll.
                    try:
                        tail, roll = Types.Bone.AxisRollFromMatrix(node.bind_matrix.to_3x3())
                        bone.head = node.bind_matrix.to_translation()
                        bone.tail = tail + bone.head
                        bone.roll = roll
                        
                        if bone.parent is not None:
                            bone.use_connect = bone.parent.tail == bone.head
                    except Exception as e:
                        print(f"Error setting bone matrix for {node.name}: {e}")
                        # Set fallback values
                        bone.head = (0, 0, 0)
                        bone.tail = (0, 0, 1)
                except Exception as e:
                    print(f"Error creating bone for node {node.name}: {e}")
        else:
            # Create a minimal armature if there are no nodes
            print("Warning: Model has no nodes, creating a minimal armature")
            bone = armature.edit_bones.new("Root")
            bone.head = (0, 0, 0)
            bone.tail = (0, 0, 1)

        Ops.object.mode_set(mode='OBJECT')

        ''' Add sockets as empties with a child-of constraint to the appropriate bone. '''
        if options.should_import_sockets and model.sockets:
            for socket in model.sockets:
                try:
                    empty_object = Data.objects.new("s_" + socket.name, None)
                    empty_object.location = socket.location
                    empty_object.rotation_quaternion = socket.rotation
                    empty_object.empty_display_type = 'PLAIN_AXES'
                    
                    child_of_constraint = empty_object.constraints.new('CHILD_OF')
                    child_of_constraint.target = armature_object
                    
                    # Make sure node index is valid
                    if 0 <= socket.node_index < len(model.nodes):
                        child_of_constraint.subtarget = model.nodes[socket.node_index].name
                    else:
                        print(f"Warning: Socket {socket.name} has invalid node index {socket.node_index}")
                        
                    empty_object.parent = armature_object
                    collection.objects.link(empty_object)
                except Exception as e:
                    print(f"Error creating socket {socket.name}: {e}")

        ''' Determine the amount of LODs to import. '''
        lod_import_count = model.lod_count if options.should_import_lods else 1

        ''' Create materials. '''
        materials = []

        for i, piece in enumerate(model.pieces):
            try:
                while len(materials) <= piece.material_index:
                    ''' Create a material for the new piece. '''
                    material = Data.materials.new(piece.name)
                    material.specular_intensity = piece.specular_power / 100

                    material.use_nodes = True

                    # TODO: maybe put something in here for the specular scale?
                    materials.append(material)

                    ''' Create texture. '''

                    # Swapped over to nodes
                    bsdf = material.node_tree.nodes["Principled BSDF"]
                    texImage = material.node_tree.nodes.new('ShaderNodeTexImage')
                    material.node_tree.links.new(bsdf.inputs['Base Color'], texImage.outputs['Color'])

                    texture = Data.textures.new(piece.name, type='IMAGE')

                    # Note: Texture image names are stored in ModelButes.txt
                    if options.image is not None:
                        texture.image = bpy.data.images.new(piece.name, width=options.image.width, height=options.image.height, alpha=True) # TODO: get real name
                        texture.image.pixels[:] = options.image.pixels[:]

                    texImage.image = texture.image
            except Exception as e:
                print(f"Error creating material for piece {piece.name}: {e}")
                # Create a fallback material
                material = Data.materials.new(f"Fallback_{len(materials)}")
                materials.append(material)


        ''' Create a mesh for each piece of each LOD level that we are importing. '''
        for lod_index in range(min(lod_import_count, 1)):  # Ensure at least one LOD
            for piece_index, piece in enumerate(model.pieces):
                try:
                    # Make sure we have valid LODs for this piece
                    if not piece.lods or lod_index >= len(piece.lods):
                        print(f"Warning: Piece {piece.name} has no LOD at index {lod_index}, skipping")
                        continue
                        
                    lod = piece.lods[lod_index]
                    
                    # Skip empty LODs
                    if not lod.vertices or not lod.faces:
                        print(f"Warning: LOD {lod_index} for piece {piece.name} has no geometry, skipping")
                        continue

                    ''' Create the object and mesh. '''
                    mesh_name = piece.name
                    if options.should_import_lods:
                        mesh_name += '.LOD{0!s}'.format(lod_index)
                    mesh = Data.meshes.new(model.name)
                    mesh_object = Data.objects.new(mesh_name, mesh)

                    ''' Add materials to mesh. '''
                    for material in materials:
                        try:
                            ''' Create UV map. '''
                            uv_texture = mesh.uv_layers.new()
                            mesh.materials.append(material)
                        except Exception as e:
                            print(f"Error adding material to mesh: {e}")

                    # Create vertex groups for nodes if we have them
                    if model.nodes:
                        for node in model.nodes:
                            try:
                                mesh_object.vertex_groups.new(name=node.name)
                            except Exception as e:
                                print(f"Error creating vertex group for node {node.name}: {e}")

                    # TODO: these need to be reset for each mesh
                    vertex_offset = 0
                    face_offset = 0

                    ''' Populate the actual mesh data. '''
                    bm = bmesh.new()
                    bm.from_mesh(mesh)

                    # Add vertices
                    for vertex in lod.vertices:
                        try:
                            bm.verts.new(vertex.location)
                        except Exception as e:
                            print(f"Error adding vertex: {e}")
                            # Add a fallback vertex at origin
                            bm.verts.new((0, 0, 0))

                    bm.verts.ensure_lookup_table()
                    duplicate_face_indices = []
                    
                    # Add faces
                    for face_index, face in enumerate(lod.faces):
                        try:
                            # Validate vertex indices
                            valid_face = True
                            verts = []
                            for vert in face.vertices:
                                if vert.vertex_index >= len(bm.verts):
                                    print(f"Warning: Invalid vertex index {vert.vertex_index} in face {face_index}, skipping")
                                    valid_face = False
                                    break
                                verts.append(bm.verts[vertex_offset + vert.vertex_index])
                                
                            if not valid_face:
                                duplicate_face_indices.append(face_index)
                                continue
                                
                            bmface = bm.faces.new(verts)
                            
                            # Assign material index if valid
                            if piece_index < len(model.pieces) and model.pieces[piece_index].material_index < len(mesh.materials):
                                bmface.material_index = model.pieces[piece_index].material_index
                            else:
                                bmface.material_index = 0
                                
                            bmface.smooth = True
                            
                        except ValueError:
                            '''
                            This face is a duplicate of another face, which is disallowed by Blender.
                            Mark this face for deletion after iteration.
                            '''
                            duplicate_face_indices.append(face_index)
                            continue
                        except Exception as e:
                            print(f"Error adding face {face_index}: {e}")
                            duplicate_face_indices.append(face_index)
                            continue

                    bm.faces.ensure_lookup_table()

                    '''
                    Warn the user of the number of duplicate faces detected, if any.
                    '''
                    if len(duplicate_face_indices) > 0:
                        print(f'WARNING: {len(duplicate_face_indices)} duplicate or invalid faces detected.')

                    '''
                    Delete any of the duplicate faces from the mesh.
                    '''
                    for face_index in reversed(sorted(duplicate_face_indices)):
                        if face_index < len(lod.faces):
                            del lod.faces[face_index]

                    vertex_offset += len(lod.vertices)
                    face_offset += len(lod.faces)

                    bm.to_mesh(mesh)
                    bm.free()

                    # Check if LOD normals are found
                    if len(lod.get_normals()):
                        custom_normals = lod.get_normals()

                    else:
                        # Fallback to Blender's smooth normals if no LOD normals are found
                        custom_normals = mesh.calculate_smooth_normals()

                    # Set the normals (either custom or smooth)
                    mesh.normals_split_custom_set_from_vertices(custom_normals)

                    # Assign texture coordinates
                    try:
                        material_face_offsets = [0] * len(mesh.materials)
                        
                        # Make sure we have UV layers
                        if mesh.uv_layers and piece.material_index < len(mesh.uv_layers):
                            uv_texture = mesh.uv_layers[piece.material_index]
                            
                            # Set the correct UV as active
                            uv_texture.active = True
                            uv_texture.active_render = True
                            
                            for face_index, face in enumerate(lod.faces):
                                try:
                                    material_face_offset = material_face_offsets[0]  # TODO: is this right?
                                    
                                    # Make sure we have valid vertices
                                    if len(face.vertices) >= 3:
                                        texcoords = [vertex.texcoord for vertex in face.vertices]
                                        for i in range(3):
                                            if material_face_offset + face_index < len(uv_texture.data) // 3:
                                                uv = texcoords[i][0], 1.0 - texcoords[i][1]
                                                uv_texture.data[(material_face_offset + face_index) * 3 + i].uv = uv
                                    else:
                                        print(f"Warning: Face {face_index} has fewer than 3 vertices")
                                except Exception as e:
                                    print(f"Error assigning UV for face {face_index}: {e}")
                                    
                            material_face_offsets[0] += len(lod.faces)
                    except Exception as e:
                        print(f"Error assigning texture coordinates: {e}")

                    # # Assign normals
                    # try:
                        # face_offset = 0
                        # polygons = mesh.polygons[face_offset:face_offset + len(lod.faces)]
                        # for face_index, (face, polygon) in enumerate(zip(lod.faces, polygons)):
                            # try:
                                # vertices = [lod.vertices[fv.vertex_index] for fv in face.vertices]
                                # for vertex, loop_index in zip(vertices, polygon.loop_indices):
                                    # # Ensure valid normal vector
                                    # if vertex.normal.length > 0:
                                        # n = Vector(vertex.normal)
                                        # mesh.loops[loop_index].normal = n
                                    # else:
                                        # # Use a default up vector if normal is invalid
                                        # mesh.loops[loop_index].normal = Vector((0, 0, 1))
                            # except Exception as e:
                                # print(f"Error assigning normal for face {face_index}: {e}")
                                
                        # face_offset += len(lod.faces)
                    # except Exception as e:
                        # print(f"Error assigning normals: {e}")

                    # mesh.validate(clean_customdata=False)
                    # mesh.update(calc_edges=False)

                    # add it to our collection c:
                    collection.objects.link(mesh_object)

                    if Ops.object.mode_set.poll():
                        Ops.object.mode_set(mode='OBJECT')
                    Ops.object.select_all(action='DESELECT')

                    ''' Add an armature modifier. '''
                    armature_modifier = mesh_object.modifiers.new(name='Armature', type='ARMATURE')
                    armature_modifier.object = armature_object
                    # TODO: remove if we fix mesh neutral pose bug?
                    armature_modifier.show_in_editmode = True
                    armature_modifier.show_on_cage = True

                    # Around line 400-450 in importer.py, in the vertex weighting section:

                    ''' Assign vertex weighting. '''
                    try:
                        vertex_offset = 0
                        
                        # Check if this is a rigid mesh piece
                        is_rigid_mesh = hasattr(piece, 'mesh_type') and piece.mesh_type == 4  # MT_RIGID = 4
                        
                        if is_rigid_mesh and hasattr(piece, 'attached_node_index'):
                            # Handle rigid mesh - assign all vertices to single bone
                            attachment_node_index = piece.attached_node_index
                            if 0 <= attachment_node_index < len(model.nodes):
                                attachment_bone_name = model.nodes[attachment_node_index].name
                                
                                if attachment_bone_name in mesh_object.vertex_groups:
                                    vertex_group = mesh_object.vertex_groups[attachment_bone_name]
                                    vertex_indices = list(range(len(lod.vertices)))
                                    vertex_group.add(vertex_indices, 1.0, 'REPLACE')
                                    print(f"  ✅ Rigid mesh '{piece.name}' weighted 1.0 to '{attachment_bone_name}' ({len(vertex_indices)} vertices)")
                                else:
                                    print(f"  ⚠️  Attachment bone '{attachment_bone_name}' not found in vertex groups")
                            else:
                                print(f"  ⚠️  Invalid attachment node index {attachment_node_index}")
                        else:
                            # Handle skeletal mesh - use individual vertex weights
                            for (vertex_index, vertex) in enumerate(lod.vertices):
                                try:
                                    for weight in vertex.weights:
                                        if 0 <= weight.node_index < len(model.nodes):
                                            vertex_group_name = model.nodes[weight.node_index].name
                                            if vertex_group_name in mesh_object.vertex_groups:
                                                vertex_group = mesh_object.vertex_groups[vertex_group_name]
                                                vertex_group.add([vertex_offset + vertex_index], weight.bias, 'REPLACE')
                                        else:
                                            print(f"Warning: Invalid node index {weight.node_index} in vertex {vertex_index}")
                                except Exception as e:
                                    print(f"Error assigning weights for vertex {vertex_index}: {e}")
                                    
                        vertex_offset += len(lod.vertices)
                    except Exception as e:
                        print(f"Error assigning vertex weights: {e}")
                    # Work-around for PC LTB meshes having overlapping but not connected vertices...
                    if options.should_merge_duplicate_verts:
                        try:
                            # Merge duplicates
                            bm = bmesh.new()
                            bm.from_mesh(mesh)
                            bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=0.0001)
                            bm.to_mesh(mesh)
                            bm.free()
                        except Exception as e:
                            print(f"Error merging duplicate vertices: {e}")

                    ''' Parent the mesh to the armature. '''
                    mesh_object.parent = armature_object
                    
                except Exception as e:
                    print(f"Error processing piece {piece_index} ({piece.name if hasattr(piece, 'name') else 'Unknown'}): {e}")

        ''' Animations '''
        if options.should_import_animations and model.animations:
            try:
                for ob in bpy.context.scene.objects:
                    ob.animation_data_clear()

                if model.nodes:
                    assert (len(armature.bones) == len(model.nodes))

                armature_object.animation_data_create()

                if hasattr(options, 'should_import_vertex_animations') and options.should_import_vertex_animations:
                    for obj in armature_object.children:
                        obj.shape_key_add(name="neutral_pose", from_mix=False)
                        # we'll animate using mesh.shape_keys.eval_time
                    mesh.shape_keys.animation_data_create()
                    mesh.shape_keys.use_relative = False

                actions = []
                md_actions = []

                index = 0
                processed_frame_count = 1 # 1 for neutral_pose
                
                for animation in model.animations:
                    try:
                        print("Processing", animation.name)

                        index += 1
                        # Create a new action with the animation name
                        action = bpy.data.actions.new(name=animation.name)

                        # Temp set
                        armature_object.animation_data.action = action

                        if hasattr(options, 'should_import_vertex_animations') and options.should_import_vertex_animations:
                            # Create a new shape key action with d_ prefixed animation name
                            md_action = Data.actions.new(name="d_%s" % (animation.name))
                            mesh.shape_keys.animation_data.action = md_action

                        # For every keyframe
                        for keyframe_index, keyframe in enumerate(animation.keyframes):
                            try:
                                # Set keyframe time - Scale it down to the default blender animation framerate (25fps)
                                subframe_time = keyframe.time * get_framerate()
                                '''
                                Recursively apply transformations to a nodes children
                                Notes: It carries everything (nodes, pose_bones..) with it, because I expected it to not be a child of this scope...oops!
                                '''
                                def recursively_apply_transform(nodes, node_index, pose_bones, parent_matrix):
                                    try:
                                        # keyframe_index = 0
                                        node = nodes[node_index]
                                        pose_bone = pose_bones[node_index]
                                        original_index = node_index

                                        # Get the current transform
                                        transform = animation.node_keyframe_transforms[node_index][keyframe_index]

                                        mat_scale = Matrix()

                                        if model.version == 6 and model.flip_anim:
                                            transform.rotation.conjugate()
                                        # End

                                        # Form our animation matrix
                                        mat_rot = transform.rotation.to_matrix()
                                        mat_loc = Matrix.Translation(transform.location)
                                        matrix = mat_loc @ mat_rot.to_4x4() @ mat_scale

                                        # If we have a parent, make sure to apply their matrix with ours to get position relative to our parent
                                        # otherwise just use our matrix
                                        if parent_matrix != None:
                                            matrix = parent_matrix @ matrix

                                        pose_bone.matrix = matrix

                                        for _ in range(0, node.child_count):
                                            node_index = node_index + 1
                                            node_index = recursively_apply_transform(nodes, node_index, pose_bones, pose_bone.matrix)

                                        return node_index
                                    except Exception as e:
                                        print(f"Error applying transform for node {node_index}, keyframe {keyframe_index}: {e}")
                                        return node_index + node.child_count
                                '''
                                Func End
                                '''
                                recursively_apply_transform(model.nodes, 0, armature_object.pose.bones, None)

                                # For every bone
                                for bone, node in zip(armature_object.pose.bones, model.nodes):
                                    bone.keyframe_insert('location', frame=subframe_time)
                                    bone.keyframe_insert('rotation_quaternion', frame=subframe_time)
                                # End For

                                if hasattr(options, 'should_import_vertex_animations') and options.should_import_vertex_animations:
                                    # shape keys, here I go!
                                    for obj in armature_object.children:
                                        # create our shape key
                                        shape_key = obj.shape_key_add(name="%s_%d" % (animation.name, keyframe_index), from_mix=False)

                                        for vert_index, vert in enumerate(obj.data.vertices):
                                            our_vert_index = vert_index
                                            node_index = model.pieces[0].lods[0].vertices[our_vert_index].weights[0].node_index
                                            node = model.nodes[node_index]

                                            if node.md_vert_count > 0:
                                                md_vert = node.md_vert_list.index(our_vert_index) + (keyframe_index * node.md_vert_count)

                                                vertex_transform = animation.vertex_deformations[node_index][md_vert].location
                                                shape_key.data[vert_index].co = node.bind_matrix @ vertex_transform
                                            # End If
                                        # End For
                                    # End For

                                    mesh.shape_keys.eval_time = shape_key.frame
                                    mesh.shape_keys.keyframe_insert("eval_time", frame=subframe_time)
                                # End For
                            except Exception as e:
                                print(f"Error processing keyframe {keyframe_index} for animation {animation.name}: {e}")

                        processed_frame_count += len(animation.keyframes)

                        # Add to actions array
                        actions.append(action)
                        if hasattr(options, 'should_import_vertex_animations') and options.should_import_vertex_animations:
                            md_actions.append(md_action)
                    except Exception as e:
                        print(f"Error processing animation {animation.name}: {e}")

                # Add our actions to animation data
                if actions:
                    armature_object.animation_data.action = actions[0]
                if hasattr(options, 'should_import_vertex_animations') and options.should_import_vertex_animations and md_actions:
                    mesh.shape_keys.animation_data.action = md_actions[0]
            except Exception as e:
                print(f"Error importing animations: {e}")

        # Set almost sane defaults
        Context.scene.frame_start = 0
        #Context.scene.frame_end = ceil(max([animation.keyframes[-1].time * get_framerate() for animation in model.animations]))
        # Set our keyframe time to 0
        Context.scene.frame_set(0)
        # Set this because almost 100% chance you're importing keyframes that aren't aligned to 25fps
        Context.scene.show_subframe = True

        # Apply coordinate system transformations
        armature_object.rotation_euler.x = math.radians(90)
        #armature_object.scale.x = -1.0

        # Apply the geometry flip in armature space
        # This may not be the best place to do it, but it works for now!
        if hasattr(model, 'version') and (model.version == 6 or getattr(model, 'flip_geom', False)):
            armature_object.scale.z = -1.0

        return {'FINISHED'}
    except Exception as e:
        show_message_box(f"Error during model import: {str(e)}", "Import Error", 'ERROR')
        return {'CANCELLED'}


#
# Real rough, but here's the importer classes
# They're connected in __init__
# These can definitely be combined at some point...
#


class ImportOperatorABC(bpy.types.Operator, bpy_extras.io_utils.ImportHelper):
    """This appears in the tooltip of the operator and in the generated docs"""
    bl_idname = 'io_scene_abc.abc_import'  # important since its how bpy.ops.import_test.some_data is constructed
    bl_label = 'Import Lithtech ABC'
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'

    # ImportHelper mixin class uses this
    filename_ext = ".abc"

    filter_glob: StringProperty(
        default="*.abc",
        options={'HIDDEN'},
        maxlen=255,  # Max internal buffer length, longer would be clamped.
    )

    # List of operator properties, the attributes will be assigned
    # to the class instance from the operator settings before calling.
    bone_length_min: FloatProperty(
        name='Bone Length',
        default=0.1,
        description='The minimum bone length',
        min=0.01
    )

    should_import_lods: BoolProperty(
        name="Import LODs (Not implemented)",
        description="When checked, LOD meshes will be imported (suffixed with .LOD0, .LOD1 etc.)",
        default=False,
    )

    should_import_animations: BoolProperty(
        name="Import Animations (Experimental)",
        description="When checked, animations will be imported as actions.",
        default=True,
    )

    should_import_vertex_animations: BoolProperty(
        name="Import Vertex Animations (Experimental)",
        description="When checked, vertex animations will be imported. (Requires Import Animations.)",
        default=False,
    )

    should_import_sockets: BoolProperty(
        name="Import Sockets",
        description="When checked, sockets will be imported as Empty objects.",
        default=False,
    )

    should_merge_pieces: BoolProperty(
        name="Merge Pieces (Not implemented)",
        description="When checked, pieces that share a material index will be merged.",
        default=False,
    )

    should_import_textures: BoolProperty(
        name="Import Textures (WIP)",
        description="When checked, pieces that share a material index will be merged.",
        default=False,
    )

    should_clear_scene: BoolProperty(
        name="Clear Scene",
        description="When checked, the scene will be cleared before the model is imported.",
        default=False,
    )

    def draw(self, context):
        layout = self.layout

        box = layout.box()
        box.label(text='Nodes')
        box.row().prop(self, 'bone_length_min')
        box.row().prop(self, 'should_import_sockets')

        box = layout.box()
        box.label(text='Meshes')
        box.row().prop(self, 'should_import_lods')
        box.row().prop(self, 'should_merge_pieces')

        box = layout.box()
        box.label(text='Materials')
        box.row().prop(self, 'should_import_textures')
        # box.row().prop(self, 'should_assign_materials')

        box = layout.box()
        box.label(text='Animations')
        box.row().prop(self, 'should_import_animations')
        box.row().prop(self, 'should_import_vertex_animations')

        box = layout.box()
        box.label(text='Misc')
        box.row().prop(self, 'should_clear_scene')

    def execute(self, context):
        # Load the model
        try:    
            try:
                model = ABCModelReader().from_file(self.filepath)
                print("Successfully loaded model with ABC reader")
            except Exception as e:
                print(f"ABC reader failed: {str(e)}")
                print("Attempting to load with ABC V6 reader")
                model = ABCV6ModelReader().from_file(self.filepath)
                print("Successfully loaded model with ABC V6 reader")
        except Exception as e:
            show_message_box(str(e), "Read Error", 'ERROR')
            return {'CANCELLED'}
            
        # Ensure the model has a valid name
        model.name = os.path.splitext(os.path.basename(self.filepath))[0]
        
        # Try to load image if requested
        image = None
        if self.should_import_textures:
            filename = os.path.splitext(os.path.basename(self.filepath))[0]
            skins_directory = os.path.join(os.path.dirname(self.filepath), '..', 'SKINS')
            texture = os.path.join(skins_directory, filename + '.DTX')
            try:
                image = DTX(texture)
            except IOError as e:
                print(f"Could not load texture: {e}")
                
        # Set up import options
        options = ModelImportOptions()
        options.bone_length_min = self.bone_length_min
        options.should_import_lods = self.should_import_lods
        options.should_import_animations = self.should_import_animations
        options.should_import_vertex_animations = self.should_import_vertex_animations
        options.should_import_sockets = self.should_import_sockets
        options.should_merge_pieces = self.should_merge_pieces
        options.should_clear_scene = self.should_clear_scene
        options.image = image
        
        # Import the model
        return import_model(model, options)

    @staticmethod
    def menu_func_import(self, context):
        self.layout.operator(ImportOperatorABC.bl_idname, text='Lithtech ABC (.abc)')


class ImportOperatorLTB(bpy.types.Operator, bpy_extras.io_utils.ImportHelper):
    """This appears in the tooltip of the operator and in the generated docs"""
    bl_idname = 'io_scene_lithtech.ltb_import'  # important since its how bpy.ops.import_test.some_data is constructed
    bl_label = 'Import Lithtech LTB'
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'

    # ImportHelper mixin class uses this
    filename_ext = ".ltb"

    filter_glob: StringProperty(
        default="*.ltb",
        options={'HIDDEN'},
        maxlen=255,  # Max internal buffer length, longer would be clamped.
    )

    # List of operator properties, the attributes will be assigned
    # to the class instance from the operator settings before calling.
    bone_length_min: FloatProperty(
        name='Bone Length',
        default=0.1,
        description='The minimum bone length',
        min=0.01
    )

    should_import_lods: BoolProperty(
        name="Import LODs",
        description="When checked, LOD meshes will be imported (suffixed with .LOD0, .LOD1 etc.)",
        default=False,
    )

    should_import_animations: BoolProperty(
        name="Import Animations",
        description="When checked, animations will be imported as actions.",
        default=True,
    )

    should_import_sockets: BoolProperty(
        name="Import Sockets",
        description="When checked, sockets will be imported as Empty objects.",
        default=False,
    )

    should_merge_pieces: BoolProperty(
        name="Merge Pieces (not yet working)",
        description="When checked, pieces that share a material index will be merged.",
        default=False,
    )

    should_import_textures: BoolProperty(
        name="Import Textures (WIP)",
        description="When checked, pieces that share a material index will be merged.",
        default=False,
    )

    should_clear_scene: BoolProperty(
        name="Clear Scene",
        description="When checked, the scene will be cleared before the model is imported.",
        default=True,
    )

    should_merge_duplicate_verts: BoolProperty(
        name="Merge Duplicate Vertices",
        description="When checked, any overlapping but non-connected vertices will be merged. (Recommended for PC LTB files.)",
        default=True,
    )

    def draw(self, context):
        layout = self.layout

        box = layout.box()
        box.label(text='Nodes')
        # box.row().prop(self, 'bone_length_min')
        box.row().prop(self, 'should_import_sockets')

        box = layout.box()
        box.label(text='Meshes')
        box.row().prop(self, 'should_merge_duplicate_verts')
        # box.row().prop(self, 'should_import_lods')
        # box.row().prop(self, 'should_merge_pieces')

        # box = layout.box()
        # box.label(text='Materials')
        # box.row().prop(self, 'should_import_textures')
        # # box.row().prop(self, 'should_assign_materials')

        box = layout.box()
        box.label(text='Animations')
        box.row().prop(self, 'should_import_animations')

        box = layout.box()
        box.label(text='Misc')
        box.row().prop(self, 'should_clear_scene')

    def execute(self, context):
        # Load the model with improved error handling
        try:
            try:
                print("Attempting to load model with PC LTB reader")
                model = PCLTBModelReader().from_file(self.filepath)
                print("Successfully loaded model with PC LTB reader")
            except Exception as e:
                print(f"PC LTB reader failed: {str(e)}")
                print("Attempting to load with PS2 LTB reader")
                model = PS2LTBModelReader().from_file(self.filepath)
                print("Successfully loaded model with PS2 LTB reader")
                
            # Verify model has required attributes
            # The updated PS2 reader should always return a valid model with these attributes,
            # but let's check anyway to be safe
            if not hasattr(model, 'pieces') or not hasattr(model, 'nodes'):
                raise ValueError("Invalid model structure - missing required attributes")
                
            # Ensure the model has valid pieces
            if not model.pieces:
                print("WARNING: Model has no pieces")
                # Continue anyway, as the user might just want to view the skeleton
                
            # Set model name from filepath
            model.name = os.path.splitext(os.path.basename(self.filepath))[0]
            
            # Try to load texture if requested
            image = None
            if self.should_import_textures:
                try:
                    filename = os.path.splitext(os.path.basename(self.filepath))[0]
                    skins_directory = os.path.join(os.path.dirname(self.filepath), '..', 'SKINS')
                    texture = os.path.join(skins_directory, filename + '.DTX')
                    try:
                        image = DTX(texture)
                    except IOError as e:
                        print(f"Could not load texture: {e}")
                except Exception as e:
                    print(f"Error processing texture: {e}")
                    
            # Set up import options
            options = ModelImportOptions()
            options.bone_length_min = self.bone_length_min
            options.should_import_lods = self.should_import_lods
            options.should_import_animations = self.should_import_animations
            options.should_import_vertex_animations = False
            options.should_import_sockets = self.should_import_sockets
            options.should_merge_pieces = self.should_merge_pieces
            options.should_clear_scene = self.should_clear_scene
            options.should_merge_duplicate_verts = self.should_merge_duplicate_verts
            options.image = image
            
            # Import the model
            return import_model(model, options)
            
        except Exception as e:
            show_message_box(f"Error loading model: {str(e)}", "Import Error", 'ERROR')
            import traceback
            traceback.print_exc()
            return {'CANCELLED'}

    @staticmethod
    def menu_func_import(self, context):
        self.layout.operator(ImportOperatorLTB.bl_idname, text='Lithtech LTB (.ltb)')
    