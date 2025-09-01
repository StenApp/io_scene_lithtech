import struct
import itertools
from mathutils import Vector, Quaternion, Matrix
from .utils import LTAVersion
from .reader_ltb_ps2 import MT_RIGID, MT_SKELETAL, MT_VERTEX_ANIMATED

class LTANode(object):

    def __init__(self, name='unnamed-node', attribute=None):
        self._name = name
        self._attribute = attribute
        self._depth = 0
        self._children = []

    def create_property(self, value=''):
        return self.create_child('', value)

    def create_container(self):
        return self.create_child('', None)

    def create_child(self, name, attribute=None):
        node = LTANode(name, attribute)
        node._depth = self._depth + 1
        self._children.append(node)
        return node

    def serialize(self):
        output_string = ""
        output_string += self._write_depth()
        output_string += "(%s " % self._name

        if self._attribute is not None:
            output_string += self._resolve_type(self._attribute)

        if len(self._children) == 0:
            output_string += ")\n"
            return output_string

        output_string += "\n"

        for child in self._children:
            output_string += child.serialize()

        output_string += self._write_depth()
        output_string += ")\n"

        return output_string

    def _write_depth(self):
        output_string = ""
        for _ in range(self._depth):
            output_string += "\t"
        return output_string

    def _resolve_type(self, value):
        if type(value) is str:
            return self._serialize_string(value)
        if type(value) is float:
            return self._serialize_float(value)
        if type(value) is Vector:
            return self._serialize_vector(value)
        if type(value) is Quaternion:
            return self._serialize_quat(value)
        if type(value) is Matrix:
            return self._serialize_matrix(value)
        if type(value) is list:
            return self._serialize_list(value)
        return str(value)

    def _serialize_string(self, value):
        return "\"%s\"" % value

    def _serialize_float(self, value):
        return "%.6f" % value

    def _serialize_vector(self, value):
        return "%.6f %.6f %.6f" % (value.x, value.y, value.z)

    def _serialize_quat(self, value):
        return "%.6f %.6f %.6f %.6f" % (value.x, value.y, value.z, value.w)

    def _serialize_matrix(self, value):
        output_string = ""

        for row in value:
            output_string += "\n"
            output_string += self._write_depth()
            output_string += "("
            for column in row:
                output_string += " "
                output_string += self._serialize_float(column)
            output_string += " )"

        output_string += "\n"
        output_string += self._write_depth()

        return output_string

    def _serialize_list(self, value):
        output_string = ""

        for i, item in enumerate(value):
            output_string += self._resolve_type(item)
            if i != len(value) - 1:
                output_string += " "

        return output_string

class NodeWriter(object):
    def __init__(self):
        self._index = 0

    def create_children_node(self, root_node):
        children_node = root_node.create_child('children')
        return children_node.create_container()

    def write_node_recursively(self, root_node, model):
        model_node = model.nodes[self._index]

        transform_node = root_node.create_child('transform', model_node.name)
        transform_node.create_child('matrix').create_property(model_node.bind_matrix)

        if model_node.child_count == 0:
            return

        children_container_node = self.create_children_node(transform_node)

        for _ in range(model_node.child_count):
            self._index += 1
            self.write_node_recursively(children_container_node, model)

class LTAModelWriter(object):

    def __init__(self):
        self._version = 'not-set'

    def write(self, model, path, version):
        self._version = version
        
        # Character model detection - check for 109 weightsets
        is_character = (hasattr(model, 'weight_sets') and 
                       model.weight_sets and 
                       len(model.weight_sets) == 109)
        
        print(f"Model is character (109 weightsets): {is_character}")
        
        root_node = LTANode('lt-model-0')

        on_load_cmds_node = root_node.create_child('on-load-cmds')
        on_load_cmds_container = on_load_cmds_node.create_container()

        # AnimBindings
        ab_list_node = on_load_cmds_container.create_child('anim-bindings')
        ab_container_node = ab_list_node.create_container()
        
        if hasattr(model, 'animations') and len(model.animations) > 0:
            if hasattr(model, 'anim_bindings') and len(model.anim_bindings) > 0:
                for i, anim_binding in enumerate(model.anim_bindings):
                    anim_dims = anim_binding.extents

                    if (anim_dims.magnitude == 0.0):
                        anim_dims = Vector((10.0, 10.0, 10.0))

                    ab_node = ab_container_node.create_child('anim-binding')
                    ab_node.create_child('name', anim_binding.name)
                    ab_node.create_child('dims').create_property(anim_dims)
                    ab_node.create_child('translation').create_property(anim_binding.origin)
                    interp_time = 200
                    if i < len(model.animations) and hasattr(model.animations[i], 'interpolation_time'):
                        interp_time = model.animations[i].interpolation_time
                    ab_node.create_child('interp-time', interp_time)
            else:
                for i, animation in enumerate(model.animations):
                    ab_node = ab_container_node.create_child('anim-binding')
                    ab_node.create_child('name', animation.name)
                    
                    anim_dims = getattr(animation, 'extents', Vector((24.0, 53.0, 24.0)))
                    if (anim_dims.magnitude == 0.0):
                        anim_dims = Vector((24.0, 53.0, 24.0))
                    ab_node.create_child('dims').create_property(anim_dims)
                    
                    ab_node.create_child('translation').create_property(Vector((0.0, 0.0, 0.0)))
                    
                    interp_time = getattr(animation, 'interpolation_time', 200)
                    ab_node.create_child('interp-time', interp_time)

        # Node Flags
        #Characters need 1 on root/null and 0 in _zN... , 2 in rest
        #guns need 1 on root/null and 0 on rest nodes
        snf_list_node = on_load_cmds_container.create_child('set-node-flags')
        snf_container = snf_list_node.create_container()

        
        for i, node in enumerate(model.nodes):
            if i == 0:
                flag_value = 1  # Root node always gets flag 1
            elif is_character:
                # Character model: _zN_ nodes get 0, others get 2
                if "_zN_" in node.name or "zN" in node.name:
                    flag_value = 0
                else:
                    flag_value = 2
            else:
                # Prop model: all non-root nodes get 0
                flag_value = 0
                
            snf_container.create_property([node.name, flag_value])
            print(f"Set node flag: {node.name} = {flag_value}")

        # Skeleton Deformers
        for piece in model.pieces:
            ad_node = on_load_cmds_container.create_child('add-deformer')
            sd_node = ad_node.create_child('skel-deformer')
            
            sd_node.create_child('target', piece.name)
            
            influences_node = sd_node.create_child('influences')
            weightsets_node = sd_node.create_child('weightsets')
            weightsets_container = weightsets_node.create_container()

            bone_influences = []
            for node in model.nodes:
                bone_influences.append(node.name)
                
            # Check if this is a rigid mesh
            is_rigid = hasattr(piece, 'mesh_type') and piece.mesh_type == MT_RIGID    

            print(f"\n=== Deformer for piece '{piece.name}' ===")
            print(f"Is rigid: {is_rigid}")
            
            if is_rigid:
                print(f"Attached to bone index: {getattr(piece, 'attached_node_index', 'UNKNOWN')}")

            for lod in piece.lods:
                for vertex_index, vertex in enumerate(lod.vertices):
                    weights = []
                    
                    if is_rigid:
                        # CRITICAL: For rigid meshes, ALL vertices must have IDENTICAL weights
                        # They should ALL point to the SAME bone with weight 1.0
                        if hasattr(piece, 'attached_node_index') and 0 <= piece.attached_node_index < len(model.nodes):
                            attachment_bone_index = piece.attached_node_index
                            
                            # Only add the attachment bone with full weight
                            weights.append(attachment_bone_index)
                            weights.append(1.0)
                            
                            print(f"  Vertex {vertex_index}: bone {attachment_bone_index} weight 1.0")
                        else:
                            # Fallback to root bone
                            weights.append(0)
                            weights.append(1.0)
                            print(f"  Vertex {vertex_index}: fallback to root bone")
                            
                    elif len(vertex.weights) == 0:
                        # Unweighted vertex - use root bone
                        weights.append(0)
                        weights.append(1.0)
                    else:
                        # Skeletal mesh - use actual vertex weights
                        for weight in vertex.weights:
                            if 0 <= weight.node_index < len(model.nodes):
                                weights.append(weight.node_index)
                                weights.append(weight.bias)

                    weightsets_container.create_property(weights)

            influences_node.create_property(bone_influences)
            print(f"=== End deformer for '{piece.name}' ===")


        # Command String
        if model.command_string is None:
            model.command_string = ""

        on_load_cmds_container.create_child('set-command-string', model.command_string)

        # Radius
        on_load_cmds_container.create_child('set-global-radius', model.internal_radius)

        # Sockets
        if len(model.sockets) > 0:
            ad_node = on_load_cmds_container.create_child('add-sockets')
            for socket in model.sockets:
                parent_node_name = model.nodes[socket.node_index].name

                socket_node = ad_node.create_child('socket', socket.name)
                socket_node.create_child('parent', parent_node_name)
                socket_node.create_child('pos').create_property(socket.location)
                socket_node.create_child('quat').create_property(socket.rotation)

                   
        # Animation Weightsets
        if is_character:  # <- Verwende die gleiche Variable
            print("Writing standard 109 PC character weightsets")
            
            aws_node = on_load_cmds_container.create_child('anim-weightsets')
            weightsets_container = aws_node.create_container()
                
            # Use the existing standard_weightsets code block
            standard_weightsets = [
                ("Blink", [0.0] * 8 + [2.0] + [0.0] * 16),
                ("Upper", [0.0] * 3 + [1.0] * 14 + [0.0] * 8),
                ("Lower", [1.0] * 3 + [0.0] * 14 + [1.0] * 8),
                ("Upper Instant", [0.0] * 3 + [0.5] * 14 + [0.0] * 8),
                ("Lower Instant", [0.5] * 3 + [0.0] * 14 + [0.5] * 8),
                ("Null", [0.0] * 25),
                ("Twitch", [2.0] * 25),
                ("Waist", [0.0] * 3 + [2.0, 2.0] + [0.0] * 20),
                ("Morph0", [0.0] * 25),
                ("Morph5", [0.0] * 3 + [0.05, 0.05] + [0.0] * 20),
                ("Morph10", [0.0] * 3 + [0.1, 0.1] + [0.0] * 20),
                ("Morph15", [0.0] * 3 + [0.15, 0.15] + [0.0] * 20),
                ("Morph20", [0.0] * 3 + [0.2, 0.2] + [0.0] * 20),
                ("Morph25", [0.0] * 3 + [0.25, 0.25] + [0.0] * 20),
                ("Morph30", [0.0] * 3 + [0.3, 0.3] + [0.0] * 20),
                ("Morph35", [0.0] * 3 + [0.35, 0.35] + [0.0] * 20),
                ("Morph40", [0.0] * 3 + [0.4, 0.4] + [0.0] * 20),
                ("Morph45", [0.0] * 3 + [0.45, 0.45] + [0.0] * 20),
                ("Morph50", [0.0] * 3 + [0.5, 0.5] + [0.0] * 20),
                ("Morph55", [0.0] * 3 + [0.55, 0.55] + [0.0] * 20),
                ("Morph60", [0.0] * 3 + [0.6, 0.6] + [0.0] * 20),
                ("Morph70", [0.0] * 3 + [0.7, 0.7] + [0.0] * 20),
                ("Morph80", [0.0] * 3 + [0.8, 0.8] + [0.0] * 20),
                ("Morph85", [0.0] * 3 + [0.85, 0.85] + [0.0] * 20),
                ("Morph90", [0.0] * 3 + [0.9, 0.9] + [0.0] * 20),
                ("Morph95", [0.0] * 3 + [0.95, 0.95] + [0.0] * 20),
                ("Morph100", [0.0] * 3 + [1.0, 1.0] + [0.0] * 20),
                ("Morph75", [0.0] * 3 + [0.75, 0.75] + [0.0] * 20),
                ("Morph65", [0.0] * 3 + [0.65, 0.65] + [0.0] * 20),
                ("Morph1", [0.0] * 3 + [0.01, 0.01] + [0.0] * 20),
                ("Morph2", [0.0] * 3 + [0.02, 0.02] + [0.0] * 20),
                ("Morph3", [0.0] * 3 + [0.03, 0.03] + [0.0] * 20),
                ("Morph4", [0.0] * 3 + [0.04, 0.04] + [0.0] * 20),
                ("Morph6", [0.0] * 3 + [0.06, 0.06] + [0.0] * 20),
                ("Morph7", [0.0] * 3 + [0.07, 0.07] + [0.0] * 20),
                ("Morph8", [0.0] * 3 + [0.08, 0.08] + [0.0] * 20),
                ("Morph9", [0.0] * 3 + [0.09, 0.09] + [0.0] * 20),
                ("Morph11", [0.0] * 3 + [0.11, 0.11] + [0.0] * 20),
                ("Morph12", [0.0] * 3 + [0.12, 0.12] + [0.0] * 20),
                ("Morph13", [0.0] * 3 + [0.13, 0.13] + [0.0] * 20),
                ("Morph14", [0.0] * 3 + [0.14, 0.14] + [0.0] * 20),
                ("Morph16", [0.0] * 3 + [0.16, 0.16] + [0.0] * 20),
                ("Morph17", [0.0] * 3 + [0.17, 0.17] + [0.0] * 20),
                ("Morph18", [0.0] * 3 + [0.18, 0.18] + [0.0] * 20),
                ("Morph19", [0.0] * 3 + [0.19, 0.19] + [0.0] * 20),
                ("Morph21", [0.0] * 3 + [0.21, 0.21] + [0.0] * 20),
                ("Morph22", [0.0] * 3 + [0.22, 0.22] + [0.0] * 20),
                ("Morph23", [0.0] * 3 + [0.23, 0.23] + [0.0] * 20),
                ("Morph24", [0.0] * 3 + [0.24, 0.24] + [0.0] * 20),
                ("Morph26", [0.0] * 3 + [0.26, 0.26] + [0.0] * 20),
                ("Morph27", [0.0] * 3 + [0.27, 0.27] + [0.0] * 20),
                ("Morph28", [0.0] * 3 + [0.28, 0.28] + [0.0] * 20),
                ("Morph29", [0.0] * 3 + [0.29, 0.29] + [0.0] * 20),
                ("Morph31", [0.0] * 3 + [0.31, 0.31] + [0.0] * 20),
                ("Morph32", [0.0] * 3 + [0.32, 0.32] + [0.0] * 20),
                ("Morph33", [0.0] * 3 + [0.33, 0.33] + [0.0] * 20),
                ("Morph34", [0.0] * 3 + [0.34, 0.34] + [0.0] * 20),
                ("Morph36", [0.0] * 3 + [0.36, 0.36] + [0.0] * 20),
                ("Morph37", [0.0] * 3 + [0.37, 0.37] + [0.0] * 20),
                ("Morph38", [0.0] * 3 + [0.38, 0.38] + [0.0] * 20),
                ("Morph39", [0.0] * 3 + [0.39, 0.39] + [0.0] * 20),
                ("Morph41", [0.0] * 3 + [0.41, 0.41] + [0.0] * 20),
                ("Morph42", [0.0] * 3 + [0.42, 0.42] + [0.0] * 20),
                ("Morph43", [0.0] * 3 + [0.43, 0.43] + [0.0] * 20),
                ("Morph44", [0.0] * 3 + [0.44, 0.44] + [0.0] * 20),
                ("Morph46", [0.0] * 3 + [0.46, 0.46] + [0.0] * 20),
                ("Morph47", [0.0] * 3 + [0.47, 0.47] + [0.0] * 20),
                ("Morph48", [0.0] * 3 + [0.48, 0.48] + [0.0] * 20),
                ("Morph49", [0.0] * 3 + [0.47, 0.49] + [0.0] * 20),
                ("Morph51", [0.0] * 3 + [0.51, 0.51] + [0.0] * 20),
                ("Morph52", [0.0] * 3 + [0.52, 0.52] + [0.0] * 20),
                ("Morph53", [0.0] * 3 + [0.53, 0.53] + [0.0] * 20),
                ("Morph54", [0.0] * 3 + [0.54, 0.54] + [0.0] * 20),
                ("Morph56", [0.0] * 3 + [0.56, 0.56] + [0.0] * 20),
                ("Morph57", [0.0] * 3 + [0.57, 0.57] + [0.0] * 20),
                ("Morph58", [0.0] * 3 + [0.58, 0.58] + [0.0] * 20),
                ("Morph59", [0.0] * 3 + [0.59, 0.59] + [0.0] * 20),
                ("Morph61", [0.0] * 3 + [0.61, 0.61] + [0.0] * 20),
                ("Morph62", [0.0] * 3 + [0.62, 0.62] + [0.0] * 20),
                ("Morph63", [0.0] * 3 + [0.63, 0.63] + [0.0] * 20),
                ("Morph64", [0.0] * 3 + [0.64, 0.64] + [0.0] * 20),
                ("Morph66", [0.0] * 3 + [0.66, 0.66] + [0.0] * 20),
                ("Morph67", [0.0] * 3 + [0.67, 0.67] + [0.0] * 20),
                ("Morph68", [0.0] * 3 + [0.68, 0.68] + [0.0] * 20),
                ("Morph69", [0.0] * 3 + [0.69, 0.69] + [0.0] * 20),
                ("Morph71", [0.0] * 3 + [0.71, 0.71] + [0.0] * 20),
                ("Morph72", [0.0] * 3 + [0.72, 0.72] + [0.0] * 20),
                ("Morph73", [0.0] * 3 + [0.73, 0.73] + [0.0] * 20),
                ("Morph74", [0.0] * 3 + [0.74, 0.74] + [0.0] * 20),
                ("Morph76", [0.0] * 3 + [0.76, 0.76] + [0.0] * 20),
                ("Morph77", [0.0] * 3 + [0.77, 0.77] + [0.0] * 20),
                ("Morph78", [0.0] * 3 + [0.78, 0.78] + [0.0] * 20),
                ("Morph79", [0.0] * 3 + [0.79, 0.79] + [0.0] * 20),
                ("Morph81", [0.0] * 3 + [0.81, 0.81] + [0.0] * 20),
                ("Morph82", [0.0] * 3 + [0.82, 0.82] + [0.0] * 20),
                ("Morph84", [0.0] * 3 + [0.84, 0.84] + [0.0] * 20),
                ("Morph83", [0.0] * 3 + [0.83, 0.83] + [0.0] * 20),
                ("Morph86", [0.0] * 3 + [0.86, 0.86] + [0.0] * 20),
                ("Morph87", [0.0] * 3 + [0.87, 0.87] + [0.0] * 20),
                ("Morph88", [0.0] * 3 + [0.88, 0.88] + [0.0] * 20),
                ("Morph89", [0.0] * 3 + [0.89, 0.89] + [0.0] * 20),
                ("Morph91", [0.0] * 3 + [0.91, 0.91] + [0.0] * 20),
                ("Morph92", [0.0] * 3 + [0.92, 0.92] + [0.0] * 20),
                ("Morph93", [0.0] * 3 + [0.93, 0.93] + [0.0] * 20),
                ("Morph94", [0.0] * 3 + [0.94, 0.94] + [0.0] * 20),
                ("Morph96", [0.0] * 3 + [0.96, 0.96] + [0.0] * 20),
                ("Morph97", [0.0] * 3 + [0.97, 0.97] + [0.0] * 20),
                ("Morph98", [0.0] * 3 + [0.98, 0.98] + [0.0] * 20),
                ("Morph99", [0.0] * 3 + [0.99, 0.99] + [0.0] * 20)
            ]
                
            for name, weights in standard_weightsets:
                weightset_node = weightsets_container.create_child('anim-weightset')
                weightset_node.create_child('name', name)
                weightset_node.create_child('weights').create_property(weights)
        else:
            print("Prop model - no weightsets written")
            
        
        # NODES
        h_node = root_node.create_child('hierarchy')
        node_writer = NodeWriter()
        node_writer.write_node_recursively(node_writer.create_children_node(h_node), model)

        # GEOMETRY
        for piece in model.pieces:
            p_node = root_node.create_child('shape', piece.name)
            geometry_node = p_node.create_child('geometry')
            mesh_node = geometry_node.create_child('mesh', piece.name)

            vertex_node = mesh_node.create_child('vertex')
            vertex_container = vertex_node.create_container()

            normal_node = mesh_node.create_child('normals')
            normal_container = normal_node.create_container()

            uv_node = mesh_node.create_child('uvs')
            uv_container = uv_node.create_container()

            tex_fs_node = mesh_node.create_child('tex-fs')
            tri_fs_node = mesh_node.create_child('tri-fs')

            # Check if this is a rigid mesh
            is_rigid = hasattr(piece, 'mesh_type') and piece.mesh_type == MT_RIGID
            
            print(f"\n=== Processing piece '{piece.name}' ===")
            print(f"Mesh type: {getattr(piece, 'mesh_type', 'UNKNOWN')}")
            print(f"Is rigid mesh: {is_rigid}")
            
            if is_rigid:
                print(f"Attached to node index: {getattr(piece, 'attached_node_index', 'NOT FOUND')}")
            
            face_index_list = []
            uv_index_list = []

            for lod in piece.lods:
                for face in lod.faces:
                    for face_vertex in face.vertices:
                        texcoords = [face_vertex.texcoord.x, face_vertex.texcoord.y]
                        uv_container.create_property(texcoords)
                        face_index_list.append(face_vertex.vertex_index)

                for vertex in lod.vertices:
                    if is_rigid:
                        # For rigid meshes, we need to transform coordinates to the correct space for LTA
                        if hasattr(vertex, 'original_location') and vertex.original_location is not None:
                            # Get the attachment bone
                            attachment_bone = model.nodes[piece.attached_node_index]
                            
                            # The original_location contains bone-local coordinates from LTB
                            # For LTA, we need to transform these to be relative to the model origin
                            # but positioned where the bone is located
                            
                            # Transform from bone-local space to model space
                            world_space_vertex = attachment_bone.bind_matrix @ vertex.original_location
                            world_space_normal = attachment_bone.bind_matrix.to_3x3() @ vertex.original_normal
                            world_space_normal.normalize()
                            
                            # Write the world-space coordinates to LTA
                            vertex_container.create_property(world_space_vertex)
                            normal_container.create_property(world_space_normal)
                            
                            print(f"  Bone-local: {vertex.original_location} -> World: {world_space_vertex}")
                            
                        else:
                            # Fallback: compute world space from current coordinates
                            attachment_bone = model.nodes[piece.attached_node_index]
                            world_to_object_transform = attachment_bone.bind_matrix.inverted()
                            object_space_vertex = world_to_object_transform @ vertex.location
                            object_space_normal = world_to_object_transform.to_3x3() @ vertex.normal
                            object_space_normal.normalize()
                            
                            # Transform back to world space for LTA
                            world_space_vertex = attachment_bone.bind_matrix @ object_space_vertex
                            world_space_normal = attachment_bone.bind_matrix.to_3x3() @ object_space_normal
                            world_space_normal.normalize()
                            
                            vertex_container.create_property(world_space_vertex)
                            normal_container.create_property(world_space_normal)
                            
                            print(f"  Fallback computed world-space vertex: {world_space_vertex}")
                    else:
                        # For skeletal meshes, use coordinates as-is
                        vertex_container.create_property(vertex.location)
                        normal_container.create_property(vertex.normal)


            for i in range(len(face_index_list)):
                uv_index_list.append(i)

            tri_fs_node.create_property(face_index_list)
            tex_fs_node.create_property(uv_index_list)

            texture_indices_node = p_node.create_child('texture-indices')
            texture_indices_node.create_property([piece.material_index])
            
        # ANIMATIONS
        for animation in model.animations:
            as_node = root_node.create_child('animset', animation.name)
            as_node.create_child('dims').create_property(animation.extents)

            keyframe_node = as_node.create_child('keyframe')
            keyframe2_node = keyframe_node.create_child('keyframe')

            times_node = keyframe2_node.create_child('times')
            values_node = keyframe2_node.create_child('values')

            times_list = []
            values_list = []

            for keyframe in animation.keyframes:
                times_list.append(keyframe.time)

                if keyframe.string is None:
                    keyframe.string = ""

                values_list.append(keyframe.string)

            times_node.create_property(times_list)
            values_node.create_property(values_list)

            anims_node = as_node.create_child('anims')
            anims_container_node = anims_node.create_container()

            for i, node_keyframe_transform_list in enumerate(animation.node_keyframe_transforms):
                anim_node = anims_container_node.create_child('anim')
                anim_node.create_child('parent', model.nodes[i].name)

                frames_node = anim_node.create_child('frames')
                posquat_node = frames_node.create_child('posquat')
                posquat_container = posquat_node.create_container()

                for keyframe_transform in node_keyframe_transform_list:
                    keyframe_container = posquat_container.create_container()
                    keyframe_container.create_property(keyframe_transform.location)
                    keyframe_container.create_property(keyframe_transform.rotation)

        # WRITE TO FILE
        with open(path, 'w') as f:
            print("Serializing node list...")
            s_root_node = root_node.serialize()
            f.write(s_root_node)
            print("Finished serializing node list!")