#!/usr/bin/env python3
import rospy, math, numpy as np, open3d as o3d
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
# con questo codice si aggiunge il oint coud nella scena in moveit!
def rot_matrix_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]], float)
    Ry = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]], float)
    Rx = np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]], float)
    return Rz @ Ry @ Rx  # Rz*Ry*Rx (come nel tuo script mesh)

def unit(v):
    n = np.linalg.norm(v);  return v/n if n>1e-12 else v

def main():
    rospy.init_node("add_patient_cloud_on_table", anonymous=True)

    # --- input principali ---
    pcd_path = rospy.get_param("~pcd_path")
    frame_id = rospy.get_param("~frame_id", "world")
    x = float(rospy.get_param("~x", 0.90))
    y = float(rospy.get_param("~y", 0.00))

    roll_deg  = float(rospy.get_param("~roll_deg", 90.0))
    pitch_deg = float(rospy.get_param("~pitch_deg", 0.0))
    yaw_deg   = float(rospy.get_param("~yaw_deg",  0.0))
    align_yaw_with_table = bool(rospy.get_param("~align_mesh_yaw_with_table", True))
    z_lift = float(rospy.get_param("~z_lift", 0.01))

    topic_name = rospy.get_param("~topic_name", "/cloud_with_normals")

    # --- parametri ambiente (per calcolare la quota tavolo) ---
    pedestal_size    = float(rospy.get_param("~pedestal_size", 0.60))
    pedestal_z_top   = float(rospy.get_param("~pedestal_z_top", 0.0))
    table_height     = float(rospy.get_param("~table_height", 0.75))
    table_align_top  = bool(rospy.get_param("~table_align_with_pedestal_top", True))
    table_yaw_rad    = float(rospy.get_param("~table_yaw", 0.0))

    # quota top tavolo
    if table_align_top:
        table_top_z = pedestal_z_top
    else:
        floor_z = pedestal_z_top - pedestal_size
        table_top_z = floor_z + table_height

    # allinea yaw al tavolo, in realtà lo metto false per ora, non ci capisco un granchè ma va bene così
    if align_yaw_with_table:
        yaw_deg = math.degrees(table_yaw_rad)

    roll, pitch, yaw = map(math.radians, [roll_deg, pitch_deg, yaw_deg])
    R = rot_matrix_rpy(roll, pitch, yaw)

    # --- carica PCD ---
    pcd = o3d.io.read_point_cloud(pcd_path)
    if len(pcd.points)==0:
        rospy.logerr("Empty PCD: %s", pcd_path); return

    P = np.asarray(pcd.points, float)
    if pcd.has_normals():
        N = np.asarray(pcd.normals, float)
    else:
        aabb = pcd.get_axis_aligned_bounding_box()
        diag = np.linalg.norm(aabb.get_max_bound() - aabb.get_min_bound())
        pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.02*diag, max_nn=30))
        pcd.orient_normals_consistent_tangent_plane(k=30)
        N = np.asarray(pcd.normals, float)

    # --- applico RPY e posa ---
    P_rot = (R @ P.T).T
    N_rot = np.apply_along_axis(unit, 1, (R @ N.T).T)

    z_min = float(P_rot[:,2].min())
    z = table_top_z - z_min + z_lift
    T = np.array([x,y,z], float)

    P_world = P_rot + T

    # --- pubblica PointCloud2 (latched) ---
    header = Header(frame_id=frame_id)
    fields = [
        PointField('x',0,  PointField.FLOAT32,1),
        PointField('y',4,  PointField.FLOAT32,1),
        PointField('z',8,  PointField.FLOAT32,1),
        PointField('normal_x',12,PointField.FLOAT32,1),
        PointField('normal_y',16,PointField.FLOAT32,1),
        PointField('normal_z',20,PointField.FLOAT32,1),
    ]
    data = np.hstack([P_world.astype(np.float32), N_rot.astype(np.float32)])
    cloud = pc2.create_cloud(header, fields, data.tolist())

    pub = rospy.Publisher(topic_name, PointCloud2, queue_size=1, latch=True)
    rospy.sleep(0.3)
    pub.publish(cloud)
    rospy.loginfo("Published %d points on %s (frame=%s) roll=%.1f pitch=%.1f yaw=%.1f z_lift=%.3f",
                  P_world.shape[0], topic_name, frame_id, roll_deg, pitch_deg, yaw_deg, z_lift)
    rospy.spin()

if __name__ == "__main__":
    main()
