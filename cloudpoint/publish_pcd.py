#!/usr/bin/env python3
import rospy
import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
import sys

def to_pc2(o3d_pcd, frame_id="world"):
    pts = np.asarray(o3d_pcd.points, dtype=np.float32)
    nrm = np.asarray(o3d_pcd.normals, dtype=np.float32)
    data = np.hstack([pts, nrm]).astype(np.float32)
    fields = [
        PointField('x', 0,  PointField.FLOAT32, 1),
        PointField('y', 4,  PointField.FLOAT32, 1),
        PointField('z', 8,  PointField.FLOAT32, 1),
        PointField('normal_x', 12, PointField.FLOAT32, 1),
        PointField('normal_y', 16, PointField.FLOAT32, 1),
        PointField('normal_z', 20, PointField.FLOAT32, 1),
    ]
    header = Header(frame_id=frame_id)
    return pc2.create_cloud(header, fields, data.tolist())

if __name__ == "__main__":
    rospy.init_node("pcd_publisher")
    if len(sys.argv) < 2:
        rospy.logerr("Uso: rosrun <pkg> publish_pcd.py <file.pcd> [frame_id]")
        sys.exit(1)
    pcd_path = sys.argv[1]
    frame = sys.argv[2] if len(sys.argv) > 2 else "world"
    pcd = o3d.io.read_point_cloud(pcd_path)
    pub = rospy.Publisher("cloud_with_normals", PointCloud2, queue_size=1, latch=True)
    msg = to_pc2(pcd, frame_id=frame)
    pub.publish(msg)
    rospy.loginfo("Pubblicato %s con frame_id=%s su /cloud_with_normals", pcd_path, frame)
    rospy.spin()
