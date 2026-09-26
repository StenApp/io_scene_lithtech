import os
from .abc import *
from .io import unpack
from mathutils import Vector, Matrix, Quaternion


class ABCModelReader(object):
    def __init__(self):
        self._version = 0
        self._node_count = 0
        self._lod_count = 0
        self._lod_dist_count = 0

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

    def _read_weight(self, f):
        weight = Weight()
        weight.node_index = unpack('I', f)[0]
        weight.location = self._read_vector(f)
        weight.bias = unpack('f', f)[0]
        return weight

    def _read_vertex(self, f):
        vertex = Vertex()

        if self._version != 108:
            weight_count = unpack('H', f)[0]
            vertex.sublod_vertex_index = unpack('H', f)[0]
        elif self._version == 108:
            weight_count = unpack('B', f)[0]
            vertex.sublod_vertex_index = unpack('B', f)[0]
            f.seek(2, 1)

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

        if self._version == 108:
            f.seek(6, 1)

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
        node.index = unpack('H', f)[0]
        node.flags = unpack('b', f)[0]

        if self._version == 108:
            f.seek(4, 1)

        node.bind_matrix = self._read_matrix(f)
        node.inverse_bind_matrix = node.bind_matrix.inverted()

        node.child_count = unpack('I', f)[0]
        return node

    def _read_transform(self, f):
        # Plain "Transform" struct (ABC_V9-134.bt): Location + Rotation,
        # NOTHING else, for ANY version. This is what ChildModel.Transforms
        # uses. It used to also get the v13 extra-8-bytes skip below
        # (copy-pasted from _read_anim_transform's needs) -- that was WRONG:
        # the .bt defines two DISTINCT structs, plain Transform (used here)
        # and AnimTransform (Location+Rotation+`if(g_Version>=13): float
        # Unknown[2]`, used only by Animation's KeyFrameTransform.Transforms).
        # Confirmed structural bug for any v13 ABC file with real
        # ChildModels: this function was consuming 8 extra bytes it should
        # never have, misaligning every ChildModel.Transform after the
        # first. Fixed by moving that skip into _read_anim_transform only,
        # which is the sole other caller of this method (2026-09, Sten,
        # against ABC_V9-132.bt/ABC_V9-134.bt).
        transform = Animation.Keyframe.Transform()
        transform.location = self._read_vector(f)
        transform.rotation = self._read_quaternion(f)
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

    def _read_anim_transform(self, f):
        # AnimTransform struct (ABC_V9-134.bt): plain Transform PLUS, only
        # for v13, two extra unknown floats (`if(g_Version>=13):
        # float Unknown[2]`). Used exclusively by Animation's
        # KeyFrameTransform.Transforms -- never by ChildModel.Transforms
        # (see _read_transform's comment). The v108 skip below is a
        # separate, pre-existing case (v108 isn't covered by the v9-13 .bt
        # at all and is documented elsewhere as a distinct structural
        # variant); kept as-is, unaffected by this fix.
        transform = self._read_transform(f)

        if self._version == 13:
            f.seek(8, 1)

        if self._version == 108:
            f.seek(8, 1)

        return transform

    def _read_animation(self, f):
        animation = Animation()
        animation.extents = self._read_vector(f)
        animation.name = self._read_string(f)
        animation.unknown1 = unpack('i', f)[0]  # "Val" per ABC_V9-134.bt

        # BESTAETIGT (2026-09, ABC_V9-132.bt/ABC_V9-134.bt, Sten): Val ist
        # ein ECHTER Discriminator, keine Konstante. Nur wenn Val==-1 folgt
        # (bei v12/v13 zusaetzlich ein UnkInt-Feld davor) die echte
        # KeyFrameCount als eigenes Feld. Wenn Val!=-1, IST Val selbst
        # bereits die KeyFrameCount -- dann folgen GAR KEINE weiteren
        # Felder hier. Vorher wurde IMMER so gelesen als waere Val==-1
        # (unconditional 4 oder 8 Extra-Bytes je nach Version) -- in allen
        # ~25 bisher geprueften echten Animationen (6 Testdateien v9-v13)
        # war Val zwar tatsaechlich immer -1 (dieser else-Zweig also nie
        # empirisch beobachtet), aber das war reines Glueck: bei einer
        # einzigen Animation mit Val!=-1 in einer echten Datei haette das
        # eine Fehlausrichtung fuer KeyFrames/Transforms dieser UND aller
        # nachfolgenden Animationen im selben Array verursacht.
        if animation.unknown1 == -1:
            animation.interpolation_time = unpack('I', f)[0] if self._version >= 12 else 200
            animation.keyframe_count = unpack('I', f)[0]
        else:
            animation.interpolation_time = 200
            animation.keyframe_count = animation.unknown1
        animation.keyframes = [self._read_keyframe(f) for _ in range(animation.keyframe_count)]
        animation.node_keyframe_transforms = []
        for _ in range(self._node_count):

            # Skip past -1
            if self._version >= 13:
                f.seek(4, 1)

            animation.node_keyframe_transforms.append(
                [self._read_anim_transform(f) for _ in range(animation.keyframe_count)])
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

    def from_file(self, path, start_offset=0):
        # start_offset: normal standalone .abc files start the section scan
        # at byte 0 ("Header" is the very first section). DHNP wraps an
        # otherwise-identical ABC body inside an outer LTB container
        # (LTBHeader-string + LTB_Header); reader_ltb_dhnp.py passes the
        # size of that wrapper here so the exact same section-scan loop
        # below can be reused unchanged for both cases.
        model = Model()
        model.name = os.path.splitext(os.path.basename(path))[0]

        filename = os.path.basename(path)

        print(f"\n{'='*60}")
        print(f"LOADING MODEL: {filename}")
        print(f"Format: Lithtech ABC (PC)")
        print(f"{'='*60}\n")


        with open(path, 'rb') as f:
            next_section_offset = start_offset
            while next_section_offset != -1:
                f.seek(next_section_offset)
                section_name = self._read_string(f)
                next_section_offset = unpack('i', f)[0]
                if section_name == 'Header':
                    self._version = unpack('I', f)[0]
                    if self._version not in [9, 10, 11, 12, 13, 108]:
                        raise Exception('Unsupported file version ({}).'.format(self._version))
                    model.version = self._version
                    f.seek(8, 1)
                    self._node_count = unpack('I', f)[0]
                    f.seek(20, 1)
                    self._lod_count = unpack('I', f)[0]
                    f.seek(4, 1)
                    self._weight_set_count = unpack('I', f)[0]
                    f.seek(8, 1)

                    # Unknown new value
                    if self._version >= 13:
                        f.seek(4, 1)

                    if self._version == 108:
                        f.seek(8, 1)

                    model.command_string = self._read_string(f)
                    model.internal_radius = unpack('f', f)[0]

                    # BESTAETIGT (2026-09, ABC_V9-13.bt byte-exakt gegen 8
                    # echte Dateien ueber v9/v11/v12/v13 geprueft, Sten):
                    # LODDistanceCount ist bei JEDER Version ein echtes,
                    # eigenstaendig im File stehendes Feld -- nicht aus
                    # LODCount ableitbar. Bisher wurde es fuer Version != 108
                    # per f.seek(4,1) uebersprungen und stattdessen
                    # self._lod_dist_count = self._lod_count (= LODCount)
                    # angenommen; der reale Wert entspricht in allen
                    # Testdateien LODCount-1 (Distanz-Schwellen liegen
                    # ZWISCHEN Stufen), war also off-by-one. Jetzt fuer alle
                    # Versionen gleich: echt lesen statt ableiten.
                    self._lod_dist_count = unpack('I', f)[0]

                    f.seek(60, 1)
                    model.lod_distances = [unpack('f', f)[0] for _ in range(self._lod_dist_count)]
                elif section_name == 'Pieces':
                    weight_count, pieces_count = unpack('2I', f)
                    model.pieces = [self._read_piece(f) for _ in range(pieces_count)]
                elif section_name == 'Nodes':
                    if self._version == 108:
                        weight_set_count = unpack('I', f)[0]
                        model.weight_sets = [self._read_weight_set(f) for _ in range(weight_set_count)]

                    model.nodes = [self._read_node(f) for _ in range(self._node_count)]
                    build_undirected_tree(model.nodes)

                    if self._version != 108:
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

                    # BESTAETIGT (2026-09, ABC_V9-134.bt v1.6, Sten -- byte-
                    # exakt bis Dateiende verifiziert an ARCHER.ABC v13
                    # (ChildModelCount=2 -> 1 Extra-Block/54 Eintraege) und
                    # hero_action.abc v12 (ChildModelCount=3 -> 2 Extra-
                    # Bloecke/336+16 Eintraege)): nach der "internen"
                    # AnimBindings-Sektion folgt -- OHNE eigenen Section-
                    # Marker, einfach direkt im Anschluss -- pro ECHTEM
                    # ChildModel (Index >=1; Index 0 ist immer der leere
                    # Platzhalter, der das Modell selbst repraesentiert)
                    # noch ein WEITERER Block im exakt gleichen Format wie
                    # oben (uint32 Count + Count*AnimBinding). Kommt nur
                    # vor wenn len(model.child_models) >= 2 -- bei <=1
                    # (kein oder nur der Platzhalter) endet die Datei
                    # direkt hier. Vorher komplett ungelesen (kein Bug im
                    # Sinne von Datenkorruption -- next_section_offset ist
                    # hier -1, die Sektions-Schleife endet ohnehin -- aber
                    # echte, bisher liegen gelassene Daten).
                    model.child_model_anim_bindings = []
                    extra_count = max(len(model.child_models) - 1, 0)
                    for _ in range(extra_count):
                        cm_binding_count = unpack('I', f)[0]
                        model.child_model_anim_bindings.append(
                            [self._read_anim_binding(f) for _ in range(cm_binding_count)])
                elif section_name == 'HitGroups' and self._version == 108:
                    hitgroups_count = unpack('I', f)[0]
                    #model.hitgroups = [self._read_hitgroups(f) for _ in range(hitgroups_count)]
        return model
