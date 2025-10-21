import os
from .abc import *
from .io import unpack
from mathutils import Vector, Matrix, Quaternion
from functools import cmp_to_key
import math
import copy
from .hash_ps2 import HashLookUp

#########################################################################################
# PS2 LTB Model Reader by Jake Breen
# Modified to better match the BT file structure
# Based on ps2-ltb-bt6.bt template
#########################################################################################

# Constants remain the same
# PS2 LTB
REQUESTED_FILE_TYPE = 2
# Version of the LTB
REQUESTED_VERSION = 16

# Model Types
MT_RIGID = 4
MT_SKELETAL = 5
MT_VERTEX_ANIMATED = 6

# Winding orders
WO_NORMAL = 0x412
WO_REVERSED = 0x8412

# Vif commands
VIF_FLUSH = 0x11
VIF_MSCALF = 0x15000000
VIF_DIRECT = 0x50
VIF_UNPACK = 0x6C

FLOAT_COMPARE = 1e-04

# Classes remain the same
class VIFCommand(object):
    def __init__(self):
        self.constant = 0
        self.variable = 0
        self.code = 0

    def read(self, f):
        self.constant = unpack('h', f)[0]
        self.variable = unpack('B', f)[0]
        self.code = unpack('B', f)[0]

class EndCommand(object):
    def __init__(self):
        self.code = 0

    def read(self, f):
        f.seek(4 * 3, 1)
        self.code = unpack('i', f)[0]

class LocalVertex(object):
    def __init__(self):
        self.id = 0
        self.merge_string = ""
        self.vertex = Vertex()
        self.associated_ids = []
        
class LocalFace(object):
    def __init__(self):
        self.group_id = 0
        self.face_vertex = FaceVertex()

# VertexList remains the same
class VertexList(object):
    def __init__(self):
        self.auto_increment = 0
        self.groups = []
        self.list = []
        self.face_verts = []
        self.faces = []
    
    def append(self, vertex, group_id, face_vertex, unknown_flag = False):
        # Method implementation remains the same
        local_vertex = LocalVertex()
        local_vertex.id = self.auto_increment
        local_vertex.vertex = vertex
        local_vertex.merge_string = self.generate_merge_string(vertex.location)
        local_vertex.associated_ids.append(group_id)

        local_face = LocalFace()
        local_face.group_id = group_id

        vertex_index = self.find_in_list(local_vertex.merge_string)

        if vertex_index == -1:
            self.list.append(local_vertex)
            vertex_index = self.auto_increment
            self.auto_increment += 1
        else:
            self.list[vertex_index].associated_ids.append(group_id)
            
        face_vertex.vertex_index = vertex_index
        local_face.face_vertex = face_vertex
        self.groups.append(group_id)
        self.groups = list(set(self.groups))
        self.face_verts.append(local_face)

    # Other methods remain the same
    def generate_faces(self):
        faces = []

        print("------------------------------------")
        print("Generating Faces :) ")
        print("Groups: ",self.groups)
        
        for j in range( len(self.groups) ):
            flip = False
            group_id = self.groups[j]
            grouped_faces = []

            for i in range( len(self.face_verts) ):
                face_vert = self.face_verts[i]

                if face_vert.group_id != group_id:
                    continue

                grouped_faces.append(face_vert)
            
            for i in range( len(grouped_faces) ):
                if i < 2:
                    continue

                face = Face()

                if grouped_faces[i].face_vertex.reversed:
                    if flip:
                        face.vertices = [ grouped_faces[i - 1].face_vertex, grouped_faces[i].face_vertex, grouped_faces[i - 2].face_vertex ]
                    else:
                        face.vertices = [ grouped_faces[i - 2].face_vertex, grouped_faces[i].face_vertex, grouped_faces[i - 1].face_vertex ]
                else:
                    if flip:
                        face.vertices = [ grouped_faces[i].face_vertex, grouped_faces[i - 1].face_vertex, grouped_faces[i - 2].face_vertex ]
                    else:
                        face.vertices = [ grouped_faces[i].face_vertex, grouped_faces[i - 2].face_vertex, grouped_faces[i - 1].face_vertex ]

                faces.append(face)
                flip = not flip

        self.faces = faces

    def find_in_list(self, merge_string):
        for i in range( len(self.list) ):
            if merge_string == self.list[i].merge_string:
                return i

        return -1

    def generate_merge_string(self, vector):
        return "%f/%f/%f" % (vector.x, vector.y, vector.z)

    def get_vertex_list(self):
        print("Getting vertex list! Length: %d" % len(self.list))
        out_list = []
        for i in range ( len(self.list) ):
            out_list.append(self.list[i].vertex)

        return out_list

    def get_face_list(self):
        return self.faces

# Modified PS2LTBModelReader
class PS2LTBModelReader(object):
    def __init__(self):
        self._file_type = 0
        self._version = 0
        self._node_count = 0
        self._lod_count = 0
        self._socket_counter = 0
        self._animations_processed = 0
        self._hasher = None

    # Utility methods remain the same
    def _read_matrix(self, f):
        data = unpack('16f', f)
        rows = [data[0:4], data[4:8], data[8:12], data[12:16]]
        return Matrix(rows)

    def _read_vector(self, f):
        return Vector(unpack('3f', f))

    def _read_quaternion(self, f):
        x, y, z, w = unpack('4f', f)
        return Quaternion((w, x, y, z))

    def _read_string(self, f):
        try:
            string_length = unpack('H', f)[0]
            return f.read(string_length).decode('ascii')
        except UnicodeDecodeError:
            # Return a placeholder name if we encounter decoding issues
            f.seek(f.tell() - 2)  # Go back to before the string length
            return "UnknownName"

    def _read_weight(self, f):
        weight = Weight()
        weight.node_index = unpack('I', f)[0]
        weight.location = self._read_vector(f)
        weight.bias = unpack('f', f)[0]
        return weight

    def _read_vertex(self, f):
        vertex = Vertex()
        weight_count = unpack('H', f)[0]
        vertex.sublod_vertex_index = unpack('H', f)[0]
        vertex.weights = [self._read_weight(f) for _ in range(weight_count)]
        vertex.location = self._read_vector(f)
        vertex.normal = self._read_vector(f)
        return vertex

    def _read_face_vertex(self, f):
        face_vertex = FaceVertex()
        face_vertex.texcoord.xy = unpack('2f', f)
        face_vertex.vertex_index = unpack('H', f)[0]
        return face_vertex

    def _read_face(self, f):
        face = Face()
        face.vertices = [self._read_face_vertex(f) for _ in range(3)]
        return face

    def _read_lod(self, f):
        lod = LOD()
        face_count = unpack('I', f)[0]
        lod.faces = [self._read_face(f) for _ in range(face_count)]
        vertex_count = unpack('I', f)[0]
        lod.vertices = [self._read_vertex(f) for _ in range(vertex_count)]
        return lod

    def _read_piece(self, f):
        piece = Piece()
        piece.material_index = unpack('H', f)[0]
        piece.specular_power = unpack('f', f)[0]
        piece.specular_scale = unpack('f', f)[0]
        if self._version > 9:
            piece.lod_weight = unpack('f', f)[0]
        piece.padding = unpack('H', f)[0]
        piece.name = self._read_string(f)
        piece.lods = [self._read_lod(f) for _ in range(self._lod_count)]
        return piece

    def _read_node(self, f):
        node = Node()
        node.name = self._read_string(f)
        node.bind_matrix = self._read_matrix(f)
        f.seek(4, 1) 
        node.child_count = unpack('I', f)[0]
        node.index = unpack('H', f)[0]
        f.seek(2, 1)
        return node

    def _read_transform(self, f):
        transform = Animation.Keyframe.Transform()
        
        location = unpack('3h', f)
        location_small_scale = unpack('h', f)[0]
        rotation = unpack('4h', f)
        
        SCALE_ROT = 0x4000
        SCALE_LOC = 0x10
        
        if location_small_scale == 0:
            SCALE_LOC = 0x1000
            
        transform.location.x = location[0] / SCALE_LOC
        transform.location.y = location[1] / SCALE_LOC
        transform.location.z = location[2] / SCALE_LOC
        
        transform.rotation.x = rotation[0] / SCALE_ROT
        transform.rotation.y = rotation[1] / SCALE_ROT
        transform.rotation.z = rotation[2] / SCALE_ROT
        transform.rotation.w = rotation[3] / SCALE_ROT
        
        return transform

    def _read_child_model(self, f):
        #only for character models, we count ourselves as 1!
        child_model = ChildModel()
        child_model.name = self._read_string(f)
        #child_model.build_number = unpack('I', f)[0]
        #child_model.transforms = [self._read_transform(f) for _ in range(self._node_count)]
        return child_model

    def _read_keyframe(self, f):
        keyframe = Animation.Keyframe()
        keyframe.time = unpack('I', f)[0]
        keyframe.string = self._read_string(f)
        return keyframe

    def _read_animation(self, f):
        animation = Animation()
        animation.name = "Animation_%d" % self._animations_processed
        animation.extents = self._read_vector(f)
        
        unknown_vector_maybe = self._read_vector(f)
        hashed_string = unpack('I', f)[0]
        animation.interpolation_time = unpack('I', f)[0]
        animation.keyframe_count = unpack('I', f)[0]
        animation.keyframes = [self._read_keyframe(f) for _ in range(animation.keyframe_count)]
        animation.node_keyframe_transforms = []
        for _ in range(self._node_count):
            start_marker = unpack('I', f)[0]
            animation.node_keyframe_transforms.append(
                [self._read_transform(f) for _ in range(animation.keyframe_count)])
                
        self._animations_processed += 1
        
        looked_up_value = self._hasher.lookup_hash(hashed_string, "animations")
        
        if (looked_up_value != None):
            animation.name = looked_up_value
            
        return animation
    
    def _read_socket(self, f):
        socket = Socket()
        f.seek(4, 1)
        socket.rotation = self._read_quaternion(f)
        socket.location = self._read_vector(f)
        f.seek(4, 1)
        socket.node_index = unpack('I', f)[0]
        hashed_string = unpack('I', f)[0]
        
        f.seek(4, 1)
        
        socket.name = "Socket" + str(self._socket_counter)
        self._socket_counter += 1
        
        looked_up_value = self._hasher.lookup_hash(hashed_string, "sockets")
        
        if (looked_up_value != None):
            socket.name = looked_up_value
            
        return socket

    def _read_anim_binding(self, f):
        anim_binding = AnimBinding()
        anim_binding.name = self._read_string(f)
        anim_binding.extents = self._read_vector(f)
        anim_binding.origin = self._read_vector(f)
        return anim_binding

    def _read_weight_set(self, f):
        weight_set = WeightSet()
        # First read the ID
        weight_set.id = unpack('I', f)[0]
        # Then read the node count
        node_count = unpack('I', f)[0]
        # Read weights for each node
        weight_set.node_weights = [unpack('f', f)[0] for _ in range(node_count)]
        return weight_set
        
        
    def _apply_piece_attachments(self, model):
        """
        Apply proper positioning for pieces based on their mesh type and node binding.
        Uses node_binding field to determine attachment behavior.
        """
        print("=== APPLYING PIECE POSITIONING ===")
        
        for piece_index, piece in enumerate(model.pieces):
            print(f"\nProcessing Piece {piece_index}: {piece.name}")
            
            # Only process LOD 0 (highest detail)
            if not piece.lods or len(piece.lods) == 0:
                print(f"  ⚠️  No LODs found for piece {piece_index}")
                continue
                
            lod = piece.lods[0]  # Use primary LOD
            
            if not hasattr(lod, 'node_binding'):
                print(f"  ⚠️  No node_binding found for piece {piece_index}")
                continue
            
            print(f"  LOD 0 - MeshType: {lod.mesh_type}, NodeBinding: {lod.node_binding}")
            
            success = False
            if lod.mesh_type == MT_RIGID:
                success = self._position_rigid_mesh(piece, lod, model, piece_index)
            elif lod.mesh_type == MT_SKELETAL:
                success = self._position_skeletal_mesh(piece, lod, model, piece_index)
            elif lod.node_binding == 0:
                success = self._position_world_object(piece, piece_index)
            else:
                print(f"  ⚠️  Unknown mesh type {lod.mesh_type} for piece {piece_index}")
            
            if success:
                print(f"  ✅ Successfully positioned piece {piece_index}")
            else:
                print(f"  ❌ Failed to position piece {piece_index}")

    def _position_rigid_mesh(self, piece, lod, model, piece_index):
        """
        Position rigid mesh pieces. For rigid meshes, node_binding = target node index.
        Vertices are stored in object space relative to the attachment bone.
        """
        node_index = lod.node_binding
        
        if not (0 <= node_index < len(model.nodes)):
            print(f"  ❌ Rigid mesh node index {node_index} out of range (max: {len(model.nodes)-1})")
            return False
        
        target_node = model.nodes[node_index]
        print(f"  ✅ Rigid mesh attached to Node {node_index} ({target_node.name})")
        
        # Store attachment info for both piece and LOD
        piece.mesh_type = MT_RIGID
        piece.attached_node_index = node_index
        piece.attachment_transform = target_node.bind_matrix
        piece.is_rigid_mesh = True
        
        # Extract and display position
        pos = target_node.bind_matrix.translation
        print(f"     Target Position: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")
        
        # PRESERVE ORIGINAL DATA: Store both original (object space) and transformed (world space) data
        print(f"     Preserving original object-space data for {len(lod.vertices)} vertices")
        for vertex in lod.vertices:
            # Store original position and normal (object space - what's in the LTB file)
            vertex.original_location = vertex.location.copy()
            vertex.original_normal = vertex.normal.copy()
            
            # Transform for Blender display (world space)
            vertex.location = target_node.bind_matrix @ vertex.location
            vertex.normal = target_node.bind_matrix.to_3x3() @ vertex.normal
            vertex.normal.normalize()
        
        # Set up weights for all vertices to attach to the single bone
        print(f"     Setting up rigid mesh weights for {len(lod.vertices)} vertices")
        for vertex in lod.vertices:
            # Clear any existing weights
            vertex.weights = []
            
            # Create single weight pointing to attachment bone
            weight = Weight()
            weight.node_index = node_index
            weight.location = vertex.original_location  # Use original object space location
            weight.bias = 1.0  # Full weight to attachment bone
            vertex.weights.append(weight)
        
        print(f"     ✅ Set rigid mesh weights: all vertices → Node {node_index}")
        return True

    # REMOVE the separate _transform_rigid_vertices method call since we do it inline now

    def _position_skeletal_mesh(self, piece, lod, model, piece_index):
        """
        Position skeletal mesh pieces. For skeletal meshes, node_binding = bone count.
        Vertices should already have bone weights assigned.
        """
        bone_count = lod.node_binding
        print(f"  ✅ Skeletal mesh with {bone_count} bones")
        
        # Verify vertices have bone weights
        if not hasattr(lod, 'vertices') or len(lod.vertices) == 0:
            print(f"     ⚠️  No vertices found")
            return False
        
        # Check if vertices have valid bone weights
        weighted_vertices = 0
        for vertex in lod.vertices:
            if hasattr(vertex, 'weights') and len(vertex.weights) > 0:
                weighted_vertices += 1
        
        if weighted_vertices > 0:
            print(f"     ✅ {weighted_vertices}/{len(lod.vertices)} vertices have bone weights")
            piece.attached_node_index = -1  # No single attachment
            piece.attachment_transform = Matrix.Identity(4)
            piece.is_rigid_mesh = False
            return True
        else:
            print(f"     ⚠️  No vertices have bone weights - may need manual binding")
            return False

    def _position_world_object(self, piece, piece_index):
        """
        Position world objects. These pieces have no bone attachment and remain at world coordinates.
        """
        print(f"  ✅ World object - no attachment needed")
        piece.attached_node_index = -1
        piece.attachment_transform = Matrix.Identity(4)
        piece.is_rigid_mesh = False
        return True

    # Modified main method to better follow BT structure and support LODs with individual mesh types
    def from_file(self, path):
        model = Model()
        #model.name = os.path.splitext(os.path.basename(path))[0]
        filename = os.path.basename(path)
        model.name = os.path.splitext(filename)[0]
        
        print(f"\n{'='*60}")
        print(f"LOADING MODEL: {filename}")
        print(f"Format: LithTech LTB (PS2)")
        print(f"{'='*60}\n")

        with open(path, 'rb') as f:
            # Header section
            self._file_type = unpack('i', f)[0]
            self._version = unpack('h', f)[0]
            
            reserved1 = unpack('h', f)[0]  # Reserved1
            reserved2 = unpack('i', f)[0]  # Reserved2
            reserved3 = unpack('i', f)[0]  # Reserved3
            reserved4 = unpack('i', f)[0]  # Reserved4
            
            print("Loading ltb version %d" % self._version)

            # Verify file type and version
            if self._file_type is not REQUESTED_FILE_TYPE:
                message = "LTB Importer only supports PS2 LTB files."
                raise Exception(message)
                
            if self._version is not REQUESTED_VERSION:
                message = "LTB Importer only supports version %d." % REQUESTED_VERSION
                raise Exception(message)

            # Read offsets from header
            offset_offset = unpack('i', f)[0]
            piece_offset = unpack('i', f)[0]
            node_offset = unpack('i', f)[0]
            child_model_offset = unpack('i', f)[0]
            animation_offset = unpack('i', f)[0]
            socket_offset = unpack('i', f)[0]
            file_size = unpack('i', f)[0]
            
            padding = unpack('i', f)[0]  # Additional padding/unknown

            # Read model info
            keyframe_count = unpack('i', f)[0]  # KeyframeCount
            animation_count = unpack('i', f)[0]  # AnimationCount
            self._node_count = unpack('i', f)[0]  # NodeCount
            piece_count = unpack('i', f)[0]  # PieceCount
            child_model_count = unpack('i', f)[0]  # ChildModelCount
            triangle_count = unpack('i', f)[0]  # TriangleCount
            vertex_count = unpack('i', f)[0]  # VertexCount
            weight_count = unpack('i', f)[0]  # WeightCount
            self._lod_count = unpack('i', f)[0]  # LODCount
            socket_count = unpack('i', f)[0]  # SocketCount
            weight_set_count = unpack('i', f)[0]  # WeightSetCount
            string_count = unpack('i', f)[0]  # StringCount
            string_length_count = unpack('i', f)[0]  # StringLengthCount
            model_info_unknown = unpack('i', f)[0]  # Unknown
            
            # Read command string
            model.command_string = self._read_string(f)
            model.internal_radius = unpack('f', f)[0]

            # Read ModelInfoExtended
            hash_magic_number = unpack('i', f)[0]  # HashValue
            model_info_unk1 = unpack('i', f)[0]  # Unk1
            model_info_unk2 = unpack('i', f)[0]  # Unk2

            # Setup hasher
            self._hasher = HashLookUp(hash_magic_number)
            
            # Navigate to piece section using offset
            f.seek(piece_offset)
            
            # Read PieceInfo structure
            piece_info_count = unpack('i', f)[0]
            print(f"Found {piece_info_count} pieces in PieceInfo")
            
            # Process each piece according to BT structure
            for piece_index in range(piece_info_count):
                print("------------------------------------")
                print(f"Processing Piece {piece_index}")
                
                # Read Piece structure
                hashed_piece_name = unpack('i', f)[0]  # HashedPieceName
                specular_power = unpack('f', f)[0]     # SpecularPower
                specular_scale = unpack('f', f)[0]     # SpecularScale
                lod_weight = unpack('f', f)[0]         # LODWeight
                
                # Skip floaty padding (9 floats)
                f.seek(4 * 9, 1)
                
                texture_index = unpack('i', f)[0]      # TextureIndex
                unknowns = unpack('i', f)[0] + unpack('i', f)[0]  # Unknown[2]
                four = unpack('i', f)[0]               # Four
                
                # Create piece object
                piece_object = Piece()
                piece_object.name = f"Piece {piece_index}"
                
                # Try to get actual name from hash
                looked_up_value = self._hasher.lookup_hash(hashed_piece_name, "pieces")
                if looked_up_value is not None:
                    piece_object.name = looked_up_value
                    
                piece_object.material_index = texture_index
                piece_object.lods = []
                
                # Process each LOD for this piece
                for lod_index in range(self._lod_count):
                    print(f"Processing LOD {lod_index} for Piece {piece_index}")
                    
                    # Read mesh_type for this specific LOD
                    mesh_type = unpack('i', f)[0]      # MeshType - moved to LOD level
                    
                    print(f"LOD {lod_index} Mesh Type: {mesh_type}")
                    if mesh_type == MT_RIGID:
                        print("Rigid Mesh")
                    elif mesh_type == MT_SKELETAL:
                        print("Skeletal Mesh")
                    elif mesh_type == MT_VERTEX_ANIMATED:
                        print("Vertex Animated Mesh")
                    
                    # Variables for processing mesh data
                    lod = LOD()
                    lod.mesh_type = mesh_type  # Store mesh type in the LOD
                    vertex_list = VertexList()
                    mesh_set_index = 1
                    mesh_index = 0
                    lod_skeletal_unk_sector_count = 0
                    
                    # Read SkeletalMeshData if this is a skeletal mesh
                    if mesh_type == MT_SKELETAL:
                        skel_unk = unpack('i', f)[0]  # SkelUnk
                        lod_skeletal_unk_sector_count = unpack('i', f)[0]  # UnknownSectorSize
                        print(f"Skeletal mesh with UnknownSectorSize: {lod_skeletal_unk_sector_count}")
                    
                    # Read GeometryBatchHeader
                    lod_vertex_count = unpack('i', f)[0]       # VertexCount
                    lod_node_binding = unpack('i', f)[0]  # target node index for ridgid or bone count for weighting in skeletal
                    lod.node_binding = lod_node_binding  # Store it in the LOD object
                    print(f"Geometry batch: {lod_vertex_count} vertices, {lod_node_binding} target node iondex/bone count")
                    
                    # Process LODs - potentially multiple batches
                    check_for_more_data = False
                    finished_lods = False
                    
                    # Process batch data for this LOD
                    while not finished_lods:
                        # Check if we need to look for more data
                        if check_for_more_data:
                            print("Checking for more data...")
                            
                            # SizeOf(BatchConnector)
                            peek_amount = 28
                            
                            f.seek(peek_amount, 1)
                            vif_cmd = VIFCommand()
                            vif_cmd.read(f)
                            
                            # Move back
                            f.seek(-(peek_amount + 4), 1)
                            
                            # Check if there's more data
                            if vif_cmd.constant != VIF_DIRECT or vif_cmd.code != VIF_UNPACK:
                                print("No more data found!")
                                finished_lods = True
                                break
                                
                            print("Found an additional batch of data!")
                            check_for_more_data = False
                        
                        # Read BatchConnector
                        unknown_command = VIFCommand()
                        unknown_command.read(f)
                        
                        # Skip unknown
                        f.seek(4, 1)
                        
                        # Read flush command
                        flush_command = VIFCommand()
                        flush_command.read(f)
                        
                        # Skip unknowns
                        f.seek(4 * 4, 1)
                        
                        # Read PS2VIFUnpack
                        unpack_command = VIFCommand()
                        unpack_command.read(f)
                        
                        mesh_set_count = unpack('i', f)[0]
                        mesh_data_count = unpack('i', f)[0]
                        
                        # Skip zeros
                        f.seek(4 * 2, 1)
                        
                        # Track size for batch size calculation
                        size_start = f.tell()
                        running_mesh_set_count = 0
                        
                        # Process MeshSets
                        while True:
                            # Read MeshSet header
                            data_count = int.from_bytes(unpack('c', f)[0], 'little')
                            unknown_flag = int.from_bytes(unpack('c', f)[0], 'little')
                            
                            # Skip padding
                            f.seek(2, 1)
                            
                            # Read render patch details
                            unknown_val_1 = unpack('I', f)[0]  # RenderPatchStart
                            face_winding_order = unpack('I', f)[0]  # WindingOrder
                            unknown_val_2 = unpack('I', f)[0]  # Unknown3
                            
                            # Process each vertex in this mesh set
                            for i in range(data_count):
                                # Check for 1.0f padding marker
                                f.seek(4 * 3, 1)
                                constant_one = unpack('f', f)[0]
                                
                                # If we found the marker, go back to read vertex data
                                if constant_one == 1.0:
                                    f.seek(-(4 * 4), 1)
                                
                                # Read vertex data
                                vertex = Vertex()
                                vertex.sublod_vertex_index = 0xCDCD
                                
                                vertex_data = self._read_vector(f)
                                vertex_padding = unpack('f', f)[0]
                                normal_data = self._read_vector(f)
                                normal_padding = unpack('f', f)[0]
                                
                                uv_data = Vector()
                                uv_data.x = unpack('f', f)[0]
                                uv_data.y = unpack('f', f)[0]
                                
                                vertex_index = unpack('f', f)[0]
                                unknown_padding = unpack('f', f)[0]
                                
                                # Create face vertex
                                face_vertex = FaceVertex()
                                face_vertex.texcoord = uv_data
                                face_vertex.vertex_index = mesh_index
                                face_vertex.reversed = face_winding_order == WO_REVERSED
                                
                                # Set vertex attributes
                                vertex.location = vertex_data
                                vertex.normal = normal_data
                                
                                # Add to vertex list
                                vertex_list.append(vertex, mesh_set_index, face_vertex, False)
                                
                                mesh_index += 1
                            
                            mesh_set_index += 1
                            running_mesh_set_count += 1
                            
                            # Exit loop if this was the last set (flagged with 0x80/128)
                            if unknown_flag == 128:
                                print("Found last mesh set (flag 128)")
                                break
                        
                        
                        # Check for extended data or end command
                        end_command_peek = [unpack('i', f)[0], unpack('i', f)[0], unpack('i', f)[0], unpack('i', f)[0]]
                        
                        # If end command, go back to read it properly
                        if end_command_peek[0] == 0 and end_command_peek[1] == 0 and end_command_peek[2] == 0 and end_command_peek[3] == VIF_MSCALF:
                            print("Found End Command")
                            f.seek(-(4*4), 1)
                        else:
                            print("Skipping extra data before end command")
                        
                        # Read end command
                        end_command = EndCommand()
                        end_command.read(f)
                        
                        # Calculate batch size
                        size_end = f.tell()
                        size = size_end - size_start
                        print(f"Batch size: {size} bytes")

                        # Instead of using a fixed size threshold that immediately ends processing,
                        # try to peek ahead to see if there's more data regardless of current batch size
                        vif_peek_pos = f.tell()
                        try:
                            # Peek ahead (BatchConnector structure size is 28 bytes)
                            peek_amount = 28
                            f.seek(peek_amount, 1)
                            
                            # Try to read a VIF command
                            vif_cmd = VIFCommand()
                            vif_cmd.read(f)
                            
                            # Go back to our original position
                            f.seek(vif_peek_pos)
                            
                            # Check if the peeked command looks like a valid VIF command for a new batch
                            if vif_cmd.constant == VIF_DIRECT and vif_cmd.code == VIF_UNPACK:
                                print(f"Found another batch following the current one (size: {size})")
                                check_for_more_data = True
                                finished_lods = False
                            else:
                                print(f"No more batches found after current batch (size: {size})")
                                check_for_more_data = False
                                finished_lods = True
                        except Exception as e:
                            # If an exception occurs during peeking (e.g., EOF), assume no more batches
                            print(f"Exception occurred during peek: {e}")
                            f.seek(vif_peek_pos)  # Make sure we're back at the right position
                            check_for_more_data = False
                            finished_lods = True
                    
                    # Process vertices and faces
                    lod.vertices += vertex_list.get_vertex_list()
                    vertex_list.generate_faces()
                    lod.faces += vertex_list.get_face_list()
                    
                    # Process skeletal mesh weights if needed
                    if mesh_type == MT_SKELETAL:
                        # Process the UnknownSector
                        unk_sector_start = f.tell()
                        unk_sector_finished = False
                        
                        # Step through variable-length entries
                        while True:
                            unk_amount_to_skip = unpack('H', f)[0]
                            f.seek(+(unk_amount_to_skip * 2), 1)
                            
                            current_total = (f.tell() - unk_sector_start) / 2
                            if current_total >= lod_skeletal_unk_sector_count:
                                # Look for 1.0f marker indicating start of vertex data
                                while True:
                                    test_values = unpack('4f', f)
                                    if test_values[3] == 1.0:
                                        unk_sector_finished = True
                                        f.seek(-4*4, 1)
                                        break
                                    f.seek(-14, 1)
                            
                            if unk_sector_finished:
                                break
                        
                        # Process ordered vertices
                        ordered_vertices = []
                        
                        class OrderedVertex(object):
                            def __init__(self):
                                self.location = Vector()
                                self.location_padding = 0
                                self.normal = Vector()
                                self.normal_padding = 1
                        
                        print(f"Reading {lod_vertex_count} ordered vertices at position {f.tell()}")
                        
                        for vi in range(lod_vertex_count):
                            ov = OrderedVertex()
                            ov.location = self._read_vector(f)
                            ov.location_padding = unpack('f', f)[0]
                            ov.normal = self._read_vector(f)
                            ov.normal_padding = unpack('f', f)[0]
                            ordered_vertices.append(ov)
                        
                        # Read node map
                        node_map = []
                        print(f"Reading {lod_node_binding} node map entries at position {f.tell()}")
                        
                        for wni in range(lod_node_binding):
                            node_map.append(unpack('i', f)[0])
                        
                        print(f"Node map: {node_map}")
                        
                        # Read and process vertex weights
                        for wi in range(lod_vertex_count):
                            weights = unpack('4h', f)
                            # This line has the wrong variable name
                            node_indices = unpack('4b', f)  # Changed from node_indexes to node_indices
                            
                            normalized_weights = []
                            
                            # Normalize weights
                            for weight in weights:
                                if weight == 0:
                                    continue
                                normalized_weights.append(float(weight) / 4096.0)
                            
                            processed_weights = []
                            
                            # Process node indices
                            for j in range(len(normalized_weights)):
                                weight = Weight()
                                weight.bias = normalized_weights[j]
                                weight.node_index = node_indices[j]  # Changed from node_indexes to node_indices
                                
                                if weight.node_index != 0:
                                    weight.node_index /= 4
                                    weight.node_index = int(weight.node_index)
                                
                                # Map to global node index
                                weight.node_index = node_map[weight.node_index]
                                processed_weights.append(weight)
                            
                            # Match weights to vertices by position
                            ordered_vertex = ordered_vertices[wi]
                            
                            for vi in range(len(lod.vertices)):
                                vertex = lod.vertices[vi]
                                
                                if ordered_vertex.location == vertex.location:
                                    lod.vertices[vi].weights = copy.copy(processed_weights)
                                    # Set weight locations
                                    for i in range(len(lod.vertices[vi].weights)):
                                        lod.vertices[vi].weights[i].location = (ordered_vertex.location @ Matrix())
                                    
                                    break
                    
                    # Add the LOD to the piece
                    piece_object.lods.append(lod)
                    
                    print(f"LOD {lod_index} Final vertices: {len(lod.vertices)}")
                    print(f"LOD {lod_index} Final faces: {len(lod.faces)}")
                
                
                 # AFTER the LOD loop, set piece-level mesh type from first LOD
                if piece_object.lods:
                    piece_object.mesh_type = piece_object.lods[0].mesh_type  # ADD THIS LINE
                    print(f"Set piece mesh type to: {piece_object.mesh_type}")  # ADD THIS LINE
                
                # Add the piece to the model
                model.pieces.append(piece_object)
                
                print(f"Added Piece {piece_index} with {len(piece_object.lods)} LODs")
        
            # Read Nodes section
            f.seek(node_offset)
            model.nodes = [self._read_node(f) for _ in range(self._node_count)]
            build_undirected_tree(model.nodes)
            
            # Apply positioning system after all pieces and nodes are loaded
            self._apply_piece_attachments(model)
            
            # Read WeightSet data (follows node data)
            try:
                weight_set_count = unpack('I', f)[0]
                print(f"Found {weight_set_count} weight sets")
    
                if 0 <= weight_set_count < 1000:  # Reasonable sanity check
                    weight_sets = []
                    for i in range(weight_set_count):
                        try:
                            # Read the ID
                            weight_set_id = unpack('I', f)[0]
                            # Read node count
                            node_count = unpack('I', f)[0]
                            # Read weights
                            node_weights = []
                            for j in range(node_count):
                                node_weights.append(unpack('f', f)[0])
                
                            # Create and add the weight set
                            weight_set = WeightSet()
                            weight_set.id = weight_set_id
                            weight_set.node_weights = node_weights
                            weight_sets.append(weight_set)
                        except Exception as e:
                            print(f"Error reading weight set {i+1}/{weight_set_count}: {e}")
                            break
                    print(f"Successfully read {len(weight_sets)} weight sets")
                    model.weight_sets = weight_sets
                else:
                    print(f"Skipping weight sets: count {weight_set_count} seems invalid")
                    model.weight_sets = []
            except Exception as e:
                print(f"Error reading weight sets section: {e}")
                model.weight_sets = []
 
            # Then for child models, subtract 1 from the count for character models
            try:
                f.seek(child_model_offset)
                child_model_count = unpack('I', f)[0]
    
                # Subtract 1 from child model count for character models
                if child_model_count > 0:
                    actual_child_model_count = child_model_count - 1
                    print(f"Character model: adjusting child model count from {child_model_count} to {actual_child_model_count}")
                else:
                    actual_child_model_count = child_model_count
    
                model.child_models = []
                for i in range(actual_child_model_count):
                    model.child_models.append(self._read_child_model(f))
        
            except Exception as e:
                print(f"Error reading child models: {e}")
                model.child_models = []

            # Read Animations
            try:
                f.seek(animation_offset)
                local_animation_count = unpack('I', f)[0]
                if local_animation_count > 0 and local_animation_count < 1000:  # Sanity check
                    model.animations = [self._read_animation(f) for _ in range(local_animation_count)]
                else:
                    print(f"Skipping animations: count {local_animation_count} seems invalid")
            except Exception as e:
                print(f"Error reading animations: {e}")
                model.animations = []

            # Read Sockets
            try:
                f.seek(socket_offset)
                if socket_count > 0 and socket_count < 50:  # Sanity check
                    model.sockets = [self._read_socket(f) for _ in range(socket_count)]
                else:
                    print(f"Skipping sockets: count {socket_count} seems invalid")
            except Exception as e:
                print(f"Error reading sockets: {e}")
                model.sockets = []
            
        return model