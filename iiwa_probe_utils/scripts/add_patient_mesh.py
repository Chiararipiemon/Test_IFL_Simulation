#!/usr/bin/env python3
import os, sys, math, rospy
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped
from moveit_commander import PlanningSceneInterface, roscpp_initialize

"""
Aggiunge la mesh STL del paziente nella planning scene.

Parametri ROS:
  _mesh_path : path allo STL (default: ~/Documenti/Segmentation.stl)
  _frame_id  : frame (default: world)
  _x,_y,_z   : posizione m (default: 0,0,0)
  _roll,_pitch,_yaw_deg : gradi (default: 0,0,0)
  _scale_x,_scale_y,_scale_z : (default: 1,1,1)
  _name : nome oggetto (default: patient_back)
"""

def main():
    roscpp_initialize(sys.argv)
    rospy.init_node("add_patient_mesh")

    mesh_path = os.path.expanduser(rospy.get_param("~mesh_path", "~/Documenti/Segmentation.stl"))
    frame_id  = rospy.get_param("~frame_id", "world")
    name      = rospy.get_param("~name", "patient_back") 

    # NEW: leggi namespace (default /iiwa)
    ns = rospy.get_param("~ns", "/iiwa")

    x = float(rospy.get_param("~x", 0.0))
    y = float(rospy.get_param("~y", 0.0))
    z = float(rospy.get_param("~z", 0.0))
    rr = math.radians(float(rospy.get_param("~roll", 0.0)))
    pp = math.radians(float(rospy.get_param("~pitch", 0.0)))
    yy = math.radians(float(rospy.get_param("~yaw_deg", 0.0)))

    sx = float(rospy.get_param("~scale_x", 1.0))
    sy = float(rospy.get_param("~scale_y", 1.0))
    sz = float(rospy.get_param("~scale_z", 1.0))

    if not os.path.isfile(mesh_path):
        rospy.logerr("Mesh file not found: %s", mesh_path); return

    psi = PlanningSceneInterface(ns =ns, synchronous=True)
    rospy.sleep(1.0)

    # rimuovi se già presente
    try: psi.remove_world_object(name)
    except Exception: pass
    rospy.sleep(0.3)

    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    q = tft.quaternion_from_euler(rr, pp, yy)
    pose.pose.orientation.x, pose.pose.orientation.y = q[0], q[1]
    pose.pose.orientation.z, pose.pose.orientation.w = q[2], q[3]

    psi.add_mesh(name, pose, mesh_path, size=(sx, sy, sz))
    rospy.loginfo("Added mesh '%s' from %s", name, mesh_path)
    rospy.spin()

if __name__ == "__main__":
    main()
