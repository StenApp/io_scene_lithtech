import bpy
import mathutils

def build_model_vertex_normals(obj):
    # Ensure we are working with a mesh
    if obj.type != 'MESH':
        print("Object is not a mesh")
        return

    # Ensure the mesh is updated
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.mode_set(mode='OBJECT')
    mesh = obj.data

    # Create a list to store the new vertex normals
    new_normals = [mathutils.Vector((0, 0, 0)) for _ in range(len(mesh.vertices))]

    # Create adjacency information (similar to the C++ VertPolyAdjacency structure)
    adjacency = {i: [] for i in range(len(mesh.vertices))}
    
    # Loop through faces (triangles) and update adjacency list
    for face in mesh.polygons:
        v1, v2, v3 = face.vertices
        normal = face.normal

        # Add this face to the adjacency list for each of its vertices
        adjacency[v1].append(normal)
        adjacency[v2].append(normal)
        adjacency[v3].append(normal)

    # Now calculate the vertex normals
    for i, adj_tris in adjacency.items():
        if len(adj_tris) == 0:  # Prevent division by zero
            continue  # Skip if no adjacent faces (unlikely in most meshes)

        normal_sum = mathutils.Vector((0, 0, 0))

        for tri_normal in adj_tris:
            normal_sum += tri_normal

        # Average the normals
        normal_sum /= len(adj_tris)

        # Normalize the resulting normal
        new_normals[i] = normal_sum.normalized()

        # Print the vertex normal for debugging
        print("New normal:", new_normals[i].x * 127, new_normals[i].y * 127, new_normals[i].z * 127)

    # Set custom normals to vertices
    mesh.normals_split_custom_set_from_vertices(new_normals)
    obj.data.update()

class LITHTECHPanel(bpy.types.Panel):
    """Creates a Panel in the LITHTECH category"""
    bl_label = "LITHTECH Helpers"
    bl_idname = "OBJECT_PT_vertex_normals"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'LITHTECH'  # Custom panel category

    def draw(self, context):
        layout = self.layout

        # Add a button to the panel
        layout.operator("object.build_vertex_normals_operator", text="Generate Vertex Normals")


class BuildVertexNormalsOperator(bpy.types.Operator):
    """Set vertex normals with same weighting as Lithtech ModelEdit"""
    bl_idname = "object.build_vertex_normals_operator"
    bl_label = "Generate Vertex Normals"
    
    def execute(self, context):
        obj = bpy.context.active_object
        if obj:
            build_model_vertex_normals(obj)
        return {'FINISHED'}


def register():
    bpy.utils.register_class(LITHTECHPanel)
    bpy.utils.register_class(BuildVertexNormalsOperator)


def unregister():
    bpy.utils.unregister_class(LITHTECHPanel)
    bpy.utils.unregister_class(BuildVertexNormalsOperator)


if __name__ == "__main__":
    register()
