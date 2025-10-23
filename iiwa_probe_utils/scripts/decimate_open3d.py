import sys, open3d as o3d
if len(sys.argv) < 4:
    print("Uso: python3 decimate_open3d.py <in.stl> <out.stl> <percento_es.0.10>")
    sys.exit(1)

inp, outp, perc = sys.argv[1], sys.argv[2], float(sys.argv[3])
mesh = o3d.io.read_triangle_mesh(inp)
mesh.remove_duplicated_vertices()
mesh.remove_duplicated_triangles()
mesh.remove_degenerate_triangles()
mesh.remove_unreferenced_vertices()
mesh.compute_vertex_normals()

orig_tris = len(mesh.triangles)
target = max(1000, int(orig_tris * perc))
print(f"Triangoli originali: {orig_tris}  ->  target: {target}")

simp = mesh.simplify_quadric_decimation(target)
simp.compute_vertex_normals()
ok = o3d.io.write_triangle_mesh(outp, simp)
print("Scritto:", ok, "Triangoli finali:", len(simp.triangles))
