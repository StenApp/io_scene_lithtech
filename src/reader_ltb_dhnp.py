# -*- coding: utf-8 -*-
"""
reader_ltb_dhnp.py -- Die Hard: Nakatomi Plaza (DHNP) specific LTB variant.

DHNP wraps its model data in an outer LTB container that is NOT used by any
other LithTech game we support (normal ABC, normal LTB-PC, LTB-PS2). The
container is tiny:

    uint16 len; char text[len];   // "LTBHeader"
    uint32 MeshVersion;
    uint16 FileType;              // 4 = ABC body, 1 = D3D-hybrid body
    uint16 Version;               // LTB container version (observed: 3)
    uint32 Reserved1..4;          // always 0 in every sample seen

Inside that wrapper the body is either:

  * FileType == 4 (ABC): structurally IDENTICAL to a normal standalone
    .abc file (same Header/Pieces/Nodes/ChildModels/Animation/Sockets/
    AnimBindings sections, same Piece->LOD->Face/Vertex layout, same
    Weight-list-per-vertex skinning). The section "next offset" pointers
    stored in the file are absolute from byte 0 of the WHOLE wrapped file
    (wrapper included) -- confirmed by hex/byte comparison against two real
    DHNP files (VASE2.LTB, SANTA_STATUE.LTB), both of which round-trip
    byte-perfectly this way. So this case is handled by simply skipping the
    wrapper and calling into the existing, unmodified reader_abc_pc.py
    section-scan loop (see ABCModelReader.from_file's start_offset param).

  * FileType == 1 (D3D-hybrid): a genuinely different, more complex per-LOD
    vertex-format-negotiation scheme (this file's DHNPD3DModelReader).
    The Header/Nodes/ChildModels/Animation/Sockets/AnimBindings sections are
    IDENTICAL to the ABC case (same field layout) -- only the Pieces
    section differs, so those section readers are duplicated here rather
    than shared, matching this codebase's existing convention of each
    reader_*.py being self-contained (reader_abc_pc.py and reader_ltb_pc.py
    already duplicate the same small helpers).

Everything below was verified byte-for-byte ("Perfect fit": bytes consumed
== ObjSize for every single LOD) against 7 real DHNP D3D-hybrid LTBs
(ALEXANDER, TERRORIST_1, POWEL2, ULI, BUILDING, LOGO, V_ZIPPO) covering
every MeshType/NewVertFormat/MaxBonesPerVert/MaxBonesPerTri combination
present in those files -- see LTB_D3D_MODEL_FILE_DHNP.bt (updated 2026-09)
for the matching 010 Editor template this was cross-checked against.

Two fields remain genuinely unresolved and are read-but-unused here:
  * LODHeader.UnknownFloats[7] -- probably material colour (Vermutung, not
    confirmed): constant (1,1,1,1,1,1,1) for an untextured default material,
    (0.8,0.8,0.8,1.0,0.2,0.2,0.2) for one with a texture, in the 2 files
    checked. Not needed for geometry/skinning, so not mapped to anything.
  * LODHeader.Unknown3 -- constant 2 in every single piece tested (both ABC
    and D3D bodies). Meaning unknown; harmless to ignore.
"""

import os
import struct
from .abc import *
from .io import unpack
from mathutils import Vector, Matrix, Quaternion

try:
    from .reader_abc_pc import ABCModelReader
except ImportError:
    from reader_abc_pc import ABCModelReader


DHNP_ABC_FILE_TYPE = 4
DHNP_D3D_FILE_TYPE = 1

Invalid_Bone = 255


def _read_dhnp_wrapper(f):
    """Read the LTBPrefix+LTB_Header wrapper at the current file position
    (expected to be 0). Returns (file_type, version, offset_after_wrapper).
    """
    text_len = unpack('H', f)[0]
    text = f.read(text_len).decode('ascii', errors='replace')
    if text != 'LTBHeader':
        raise Exception("Not a DHNP-wrapped LTB (expected 'LTBHeader', got '%s')" % text)
    unpack('I', f)  # MeshVersion, unused
    file_type = unpack('H', f)[0]
    version = unpack('H', f)[0]
    f.seek(16, 1)  # Reserved1..4
    return file_type, version, f.tell()


def detect_dhnp(path):
    """Return 'ltb_dhnp_abc', 'ltb_dhnp_d3d', or None (not a DHNP-wrapped file)."""
    try:
        with open(path, 'rb') as f:
            if unpack('H', f)[0] != 9:
                return None
            f.seek(0)
            file_type, _version, _offset = _read_dhnp_wrapper(f)
    except Exception:
        return None
    if file_type == DHNP_ABC_FILE_TYPE:
        return 'ltb_dhnp_abc'
    if file_type == DHNP_D3D_FILE_TYPE:
        return 'ltb_dhnp_d3d'
    return None


class DHNPABCModelReader(object):
    """DHNP's ABC-bodied LTB variant. The body is a normal ABC stream once
    the outer LTB wrapper is skipped, so this just delegates to the
    existing, unmodified ABCModelReader."""

    def from_file(self, path):
        with open(path, 'rb') as f:
            file_type, _version, offset = _read_dhnp_wrapper(f)
        if file_type != DHNP_ABC_FILE_TYPE:
            raise Exception('Not a DHNP ABC-bodied LTB (FileType=%d, expected %d).' % (
                file_type, DHNP_ABC_FILE_TYPE))
        return ABCModelReader().from_file(path, start_offset=offset)


class DHNPD3DModelReader(object):
    """DHNP's D3D-hybrid LTB variant. Genuinely different Piece/LOD/Vertex
    layout from both normal LTB-PC and ABC; everything else (Header, Nodes,
    ChildModels, Animation, Sockets, AnimBindings) matches the ABC body
    layout exactly, so those readers are copied unchanged from
    reader_abc_pc.py."""

    LOD_HEADER_SIZE = 120

    def __init__(self):
        self._version = 0
        self._node_count = 0
        self._lod_count = 0

    # -- shared small helpers (identical to reader_abc_pc.py) --------------

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
        return f.read(unpack('H', f)[0]).decode('ascii')

    # -- Pieces / LOD / Vertex (the genuinely new part) --------------------

    def _read_lod_header(self, f):
        mesh_type, obj_size = unpack('2I', f)
        unpack('7f', f)               # UnknownFloats[7] -- Vermutung: material colour, unused
        unpack('10i', f)              # UnknownInt[10] -- confirmed uninitialized padding, unused
        texture_index = unpack('I', f)[0]
        unpack('3i', f)               # efef[3] -- confirmed constant (-1,-1,-1), unused
        specular_power, specular_scale = unpack('2f', f)
        unpack('I', f)                # Unknown3 -- confirmed constant 2, unused
        vert_count, face_count, max_bones_per_tri, max_bones_per_vert = unpack('4I', f)
        return {
            'mesh_type': mesh_type,
            'obj_size': obj_size,
            'texture_index': texture_index,
            'specular_power': specular_power,
            'specular_scale': specular_scale,
            'vert_count': vert_count,
            'face_count': face_count,
            'max_bones_per_tri': max_bones_per_tri,
            'max_bones_per_vert': max_bones_per_vert,
        }

    def _read_uv_and_faces(self, f, lod, vert_count, face_count):
        uvs = [unpack('2f', f) for _ in range(vert_count)]
        for _ in range(face_count):
            face = Face()
            for _ in range(3):
                vertex_index = unpack('H', f)[0]
                face_vertex = FaceVertex()
                face_vertex.vertex_index = vertex_index
                face_vertex.texcoord.xy = uvs[vertex_index]
                face.vertices.append(face_vertex)
            lod.faces.append(face)

    def _read_rigid_lod(self, f, lod, hdr):
        # BESTAETIGT per Hex-Abgleich: ein uint32 Bone-Index direkt vor den
        # Vertices (siehe LTB_D3D_MODEL_FILE_DHNP.bt). Beobachteter Wert:
        # ein einzelner, fuer das ganze Piece geltender Bone-Index.
        bone_index = unpack('I', f)[0]
        for _ in range(hdr['vert_count']):
            vertex = Vertex()
            vertex.location = self._read_vector(f)
            vertex.normal = self._read_vector(f)
            weight = Weight()
            weight.node_index = bone_index
            weight.bias = 1.0
            vertex.weights = [weight]
            lod.vertices.append(vertex)
        self._read_uv_and_faces(f, lod, hdr['vert_count'], hdr['face_count'])

    def _read_skeletal_lod_fmt1(self, f, lod, hdr):
        # NewVertFormat == 1: packed multi-bone blend, stride depends on
        # MaxBonesPerVert (2/3/4 -> 32/36/40 bytes; anything else falls back
        # to 36, matching the .bt template's own fallback).
        max_bones = hdr['max_bones_per_vert']
        n_blend_floats = {2: 1, 3: 2, 4: 3}.get(max_bones, 2)
        unpack('2I', f)  # Streamdata, unused
        for _ in range(hdr['vert_count']):
            vertex = Vertex()
            vertex.location = self._read_vector(f)
            blend_weights = [unpack('f', f)[0] for _ in range(n_blend_floats)]
            bone_indices = unpack('4B', f)  # packed [idx0..idx(n-1), 0-padded]
            vertex.normal = self._read_vector(f)

            n = max_bones if max_bones in (2, 3, 4) else 3
            weights = []
            remaining_bias = 1.0
            for i in range(n - 1):
                w = Weight()
                w.node_index = bone_indices[i]
                w.bias = blend_weights[i]
                remaining_bias -= blend_weights[i]
                weights.append(w)
            last = Weight()
            last.node_index = bone_indices[n - 1]
            last.bias = remaining_bias
            weights.append(last)
            vertex.weights = weights

            lod.vertices.append(vertex)
        self._read_uv_and_faces(f, lod, hdr['vert_count'], hdr['face_count'])

    def _read_skeletal_lod_fmt0(self, f, lod, hdr, lod_start, f_obj):
        # NewVertFormat == 0: one weight per vertex (BoneWeight is always
        # 1.0 per the .bt template's own comment), real bone index comes
        # from the BoneSet table read after the faces. Stride depends on
        # MaxBonesPerTri (1/2 -> 24/28 bytes; anything else -> 32).
        max_bones_per_tri = hdr['max_bones_per_tri']
        for _ in range(hdr['vert_count']):
            vertex = Vertex()
            vertex.location = self._read_vector(f)
            if max_bones_per_tri == 1:
                pass  # SkeletalVertex_Fmt0_24: no BoneWeight/BoneIndex field
            elif max_bones_per_tri == 2:
                unpack('f', f)  # BoneWeight, always 1.0
            else:
                unpack('f', f)  # BoneWeight, always 1.0
                unpack('I', f)  # BoneIndex, placeholder (real one via BoneSet)
            vertex.normal = self._read_vector(f)
            weight = Weight()
            weight.node_index = 0
            weight.bias = 1.0
            vertex.weights = [weight]
            lod.vertices.append(vertex)
        self._read_uv_and_faces(f, lod, hdr['vert_count'], hdr['face_count'])

        # BoneSet table: present when the leftover bytes before ObjSize's
        # end look like "count + N*12-byte entries" (confirmed pattern,
        # identical 12-byte BoneSet struct as normal LTB-PC).
        bytes_read = f.tell() - (lod_start + 8)
        padding_needed = hdr['obj_size'] - bytes_read
        if padding_needed > 4 and (padding_needed - 4) % 12 == 0:
            bone_set_count = unpack('I', f)[0]
            for _ in range(bone_set_count):
                index_start, index_count = unpack('2H', f)
                bone_list = unpack('4B', f)
                unpack('I', f)  # IndexBufferIndex, unused
                for vertex_index in range(index_start, index_start + index_count):
                    real_bone = next((b for b in bone_list if b != Invalid_Bone), 0)
                    lod.vertices[vertex_index].weights[0].node_index = real_bone
        elif padding_needed > 0:
            f.seek(padding_needed, 1)

    def _read_lod(self, f):
        lod = LOD()
        lod_start = f.tell()
        hdr = self._read_lod_header(f)
        lod.vert_count = hdr['vert_count']
        lod.face_count = hdr['face_count']
        lod.max_bones_per_face = hdr['max_bones_per_tri']
        lod.max_bones_per_vert = hdr['max_bones_per_vert']
        lod.type = hdr['mesh_type']
        lod.textures = (hdr['texture_index'],)
        # Stashed here (not part of abc.py's LOD field set) purely so
        # _read_piece below can lift the per-LOD SpecularPower/Scale up to
        # the Piece, mirroring how reader_ltb_pc.py lifts LOD[0].textures[0]
        # up to Piece.material_index -- DHNP's Piece struct has no material
        # fields of its own (see LTB_D3D_MODEL_FILE_DHNP.bt), only per-LOD
        # ones inside LODHeader.
        lod.specular_power = hdr['specular_power']
        lod.specular_scale = hdr['specular_scale']

        if hdr['mesh_type'] == 4:
            self._read_rigid_lod(f, lod, hdr)
        elif hdr['mesh_type'] == 5:
            new_vert_format = unpack('B', f)[0]
            if new_vert_format == 1:
                self._read_skeletal_lod_fmt1(f, lod, hdr)
            else:
                self._read_skeletal_lod_fmt0(f, lod, hdr, lod_start, f)
        else:
            print("  WARNING: unhandled DHNP D3D MeshType=%d, skipping LOD payload" % hdr['mesh_type'])

        # Safety net: regardless of how the payload above was interpreted,
        # ObjSize is the authoritative length of this LOD (confirmed exact
        # -- "Perfect fit" -- on every one of 15+ real pieces/LODs tested).
        # Forcing the cursor here means one misread field can't cascade into
        # misparsing every piece after it.
        f.seek(lod_start + 8 + hdr['obj_size'])
        return lod

    def _read_piece(self, f):
        piece = Piece()
        piece.name = self._read_string(f)
        piece.lods = [self._read_lod(f) for _ in range(self._lod_count)]
        # DHNP's D3D Piece struct carries no material_index/specular fields
        # of its own (unlike the ABC body's Piece) -- those live per-LOD
        # inside LODHeader instead. Lift LOD[0]'s values up to the Piece,
        # same pattern reader_ltb_pc.py uses for material_index.
        if piece.lods:
            first_lod = piece.lods[0]
            if first_lod.textures:
                piece.material_index = first_lod.textures[0]
            piece.specular_power = getattr(first_lod, 'specular_power', 0.0)
            piece.specular_scale = getattr(first_lod, 'specular_scale', 0.0)
        return piece

    # -- Node / ChildModel / Animation / Socket / AnimBinding --------------
    # (identical layout to reader_abc_pc.py -- copied, not shared, matching
    # this codebase's existing per-format-reader convention)

    def _read_node(self, f):
        node = Node()
        node.name = self._read_string(f)
        node.index = unpack('H', f)[0]
        node.flags = unpack('b', f)[0]
        node.bind_matrix = self._read_matrix(f)
        node.inverse_bind_matrix = node.bind_matrix.inverted()
        node.child_count = unpack('I', f)[0]
        return node

    def _read_transform(self, f):
        transform = Animation.Keyframe.Transform()
        transform.location = self._read_vector(f)
        transform.rotation = self._read_quaternion(f)
        if self._version == 13:
            f.seek(8, 1)  # Two unknown floats (v13 only)
        return transform

    def _read_child_model(self, f):
        child_model = ChildModel()
        child_model.name = self._read_string(f)
        child_model.build_number = unpack('I', f)[0]
        child_model.transforms = [self._read_transform(f) for _ in range(self._node_count)]
        return child_model

    def _read_keyframe(self, f):
        keyframe = Animation.Keyframe()
        keyframe.time = unpack('I', f)[0]
        keyframe.string = self._read_string(f)
        return keyframe

    def _read_animation(self, f):
        animation = Animation()
        animation.extents = self._read_vector(f)
        animation.name = self._read_string(f)
        animation.unknown1 = unpack('i', f)[0]
        animation.interpolation_time = unpack('I', f)[0] if self._version >= 12 else 200
        animation.keyframe_count = unpack('I', f)[0]
        animation.keyframes = [self._read_keyframe(f) for _ in range(animation.keyframe_count)]
        animation.node_keyframe_transforms = []
        for _ in range(self._node_count):
            if self._version >= 13:
                f.seek(4, 1)  # -1 marker
            animation.node_keyframe_transforms.append(
                [self._read_transform(f) for _ in range(animation.keyframe_count)])
        return animation

    def _read_socket(self, f):
        socket = Socket()
        socket.node_index = unpack('I', f)[0]
        socket.name = self._read_string(f)
        socket.rotation = self._read_quaternion(f)
        socket.location = self._read_vector(f)
        return socket

    def _read_anim_binding(self, f):
        anim_binding = AnimBinding()
        anim_binding.name = self._read_string(f)
        anim_binding.extents = self._read_vector(f)
        anim_binding.origin = self._read_vector(f)
        return anim_binding

    def _read_weight_set(self, f):
        weight_set = WeightSet()
        weight_set.name = self._read_string(f)
        node_count = unpack('I', f)[0]
        weight_set.node_weights = [unpack('f', f)[0] for _ in range(node_count)]
        return weight_set

    def from_file(self, path):
        model = Model()
        model.name = os.path.splitext(os.path.basename(path))[0]

        filename = os.path.basename(path)
        print(f"\n{'='*60}")
        print(f"LOADING MODEL: {filename}")
        print(f"Format: LithTech LTB (DHNP D3D-hybrid)")
        print(f"{'='*60}\n")

        with open(path, 'rb') as f:
            file_type, _version, offset = _read_dhnp_wrapper(f)
            if file_type != DHNP_D3D_FILE_TYPE:
                raise Exception('Not a DHNP D3D-hybrid LTB (FileType=%d, expected %d).' % (
                    file_type, DHNP_D3D_FILE_TYPE))

            next_section_offset = offset
            while next_section_offset != -1:
                f.seek(next_section_offset)
                section_name = self._read_string(f)
                next_section_offset = unpack('i', f)[0]

                if section_name == 'Header':
                    self._version = unpack('I', f)[0]
                    model.version = self._version
                    f.seek(8, 1)  # KeyframeCount, AnimationCount
                    self._node_count = unpack('I', f)[0]
                    f.seek(4, 1)  # PieceCount (re-read from PieceHeader itself below)
                    f.seek(16, 1)  # ChildModelCount, FaceCount, VertexCount, WeightCount
                    self._lod_count = unpack('I', f)[0]
                    f.seek(4, 1)  # SocketCount
                    weight_set_count_hdr = unpack('I', f)[0]
                    # NB: on real DHNP files the v13 "Unknown" field's exact
                    # position relative to StringCount/StringLengthTotal makes
                    # no observable difference here -- both are skipped and
                    # neither value is used below.
                    if self._version >= 13:
                        f.seek(4, 1)
                    f.seek(8, 1)  # StringCount, StringLengthTotal
                    model.command_string = self._read_string(f)
                    model.internal_radius = unpack('f', f)[0]
                    lod_dist_count = unpack('I', f)[0]
                    f.seek(60, 1)  # Padding
                    model.lod_distances = [unpack('f', f)[0] for _ in range(lod_dist_count)]
                elif section_name == 'Pieces':
                    # DHNP D3D PieceHeader has NO WeightCount, just PieceCount
                    # (confirmed: differs from the ABC body's PieceHeader).
                    piece_count = unpack('I', f)[0]
                    model.pieces = [self._read_piece(f) for _ in range(piece_count)]
                elif section_name == 'Nodes':
                    model.nodes = [self._read_node(f) for _ in range(self._node_count)]
                    build_undirected_tree(model.nodes)
                    weight_set_count = unpack('I', f)[0]
                    model.weight_sets = [self._read_weight_set(f) for _ in range(weight_set_count)]
                elif section_name == 'ChildModels':
                    child_model_count = unpack('H', f)[0]
                    model.child_models = [self._read_child_model(f) for _ in range(child_model_count)]
                elif section_name == 'Animation':
                    animation_count = unpack('I', f)[0]
                    model.animations = [self._read_animation(f) for _ in range(animation_count)]
                elif section_name == 'Sockets':
                    socket_count = unpack('I', f)[0]
                    model.sockets = [self._read_socket(f) for _ in range(socket_count)]
                elif section_name == 'AnimBindings':
                    anim_binding_count = unpack('I', f)[0]
                    model.anim_bindings = [self._read_anim_binding(f) for _ in range(anim_binding_count)]
        return model
