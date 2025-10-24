import open3d as o3d
import numpy as np
import pandas as pd
from pathlib import Path
# questo codice serve per convertire la mesh.stl della superficie del paziente ottenuta da 3DSlicer in cloudpoints e per stimare la normale alla superficie (verso l'esterno) di ciascun punto. Ho scelto per ora 8000 punti.
def stl_to_pointcloud(stl_path: str,
                      n_points: int = 8000,
                      normal_nb_neighbors: int = 40,
                      seed: int = 123,
                      orient_outward: bool = True):
    stl_path = Path(stl_path)
    if not stl_path.exists():
        raise FileNotFoundError(stl_path)

    mesh = o3d.io.read_triangle_mesh(str(stl_path))
    if not mesh.has_triangles():
        raise ValueError("La mesh non contiene triangoli")

    # pulizia base
    mesh.remove_duplicated_vertices()
    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    mesh.remove_non_manifold_edges()
    mesh.remove_unreferenced_vertices()

    # Poisson-disk: sovracampioniamo e poi tagliamo a n_points
    pcd = mesh.sample_points_poisson_disk(
        number_of_points=int(n_points * 1.2),
        init_factor=5,
    )
    if len(pcd.points) > n_points:
        idx = np.random.default_rng(seed).choice(len(pcd.points), size=n_points, replace=False)
        pcd = pcd.select_by_index(idx)

    # stima normali (raggio proporzionale alla scala del modello)
    aabb = mesh.get_axis_aligned_bounding_box()
    diag = np.linalg.norm(aabb.get_max_bound() - aabb.get_min_bound())
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=float(0.02 * diag),  # ~2% della diagonale
            max_nn=int(normal_nb_neighbors)
        )
    )
    pcd.orient_normals_consistent_tangent_plane(k=normal_nb_neighbors)

    # (opzionale) orienta verso l’esterno
    if orient_outward and pcd.has_normals():
        center = pcd.get_center()
        pts = np.asarray(pcd.points)
        nrm = np.asarray(pcd.normals)
        flip_mask = (np.einsum("ij,ij->i", nrm, pts - center) < 0.0)
        nrm[flip_mask] *= -1.0
        pcd.normals = o3d.utility.Vector3dVector(nrm)

    # dataframe x,y,z,nx,ny,nz
    xyz = np.asarray(pcd.points)
    nrm = np.asarray(pcd.normals)
    df = pd.DataFrame(np.hstack([xyz, nrm]), columns=["x", "y", "z", "nx", "ny", "nz"])
    return pcd, df

if __name__ == "__main__":
    mesh_path = "/home/chiararipiemo/Documenti/Segmentation_decimated_better.stl"
    out_base  = "/home/chiararipiemo/iiwa_stack_ws/src/iiwa_probe_utils/Segmentation_decimated_better_points"

    pcd, df = stl_to_pointcloud(
        mesh_path,
        n_points=8000,
        normal_nb_neighbors=40,
        seed=123,
        orient_outward=True
    )

    df.to_csv(f"{out_base}_normals.csv", index=False)
    # PLY binario (più leggero)
    o3d.io.write_point_cloud(f"{out_base}.ply", pcd, write_ascii=False)
    # PCD: in ROS/PCL avrai campi x y z normal_x normal_y normal_z
    o3d.io.write_point_cloud(f"{out_base}.pcd", pcd)

    print(f"Punti: {len(pcd.points)}")

