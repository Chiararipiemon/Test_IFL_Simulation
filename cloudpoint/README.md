Here’s a brief description of the cloudpoint folder:
- stl_to_pointcloud.py --> main script: reads the .stl mesh, samples ~8,000 points, estimates outward surface normals, and saves results to CSV/PCD/PLY.
- Segmentation_decimated_better_points_normals.csv --> table with columns x, y, z, nx, ny, nz (coordinates and per-point normals)
- Segmentation_decimated_better_points.pcd --> binary point cloud (with normals), ready for ROS/PCL/RViz/MoveIt
- Segmentation_decimated_better_points.ply --> point cloud (with normals) in PLY format, useful for tools like MeshLab or Open3D

How to visualize:
```
python3 - <<'PY'
import open3d as o3d
pcd=o3d.io.read_point_cloud("Segmentation_decimated_better_points.pcd")
o3d.visualization.draw_geometries([pcd], point_show_normal=True)
PY
```
<img width="828" height="652" alt="immagine" src="https://github.com/user-attachments/assets/81fb06ab-e635-486d-928f-b012eaba8ad3" />
