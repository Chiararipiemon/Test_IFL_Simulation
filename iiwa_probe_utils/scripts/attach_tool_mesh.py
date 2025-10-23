#!/usr/bin/env python3
import os, sys, math, rospy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_commander import (
    MoveGroupCommander, PlanningSceneInterface,
    RobotCommander, roscpp_initialize, roscpp_shutdown
)

def quat_from_rpy(roll, pitch, yaw):
    cy, sy = math.cos(yaw*0.5), math.sin(yaw*0.5)
    cp, sp = math.cos(pitch*0.5), math.sin(pitch*0.5)
    cr, sr = math.cos(roll*0.5), math.sin(roll*0.5)
    qw = cr*cp*cy + sr*sp*sy
    qx = sr*cp*cy - cr*sp*sy
    qy = cr*sp*cy + sr*cp*sy
    qz = cr*cp*sy - sr*sp*cy
    return (qx,qy,qz,qw)

def wait_for_state_update(scene, name, attached=True, timeout=5.0):
    start = rospy.get_time()
    while (rospy.get_time() - start) < timeout and not rospy.is_shutdown():
        attached_objects = scene.get_attached_objects([name])
        is_attached = len(attached_objects.keys()) > 0
        if attached and is_attached: return True
        if not attached and name not in scene.get_known_object_names(): return True
        rospy.sleep(0.1)
    return False

def main():
    roscpp_initialize(sys.argv)
    rospy.init_node("attach_tool_mesh", anonymous=True)

    mesh_path = os.path.expanduser(rospy.get_param("~mesh_path", ""))
    if not mesh_path or not os.path.exists(mesh_path):
        rospy.logerr("~mesh_path mancante o file inesistente: %s", mesh_path); sys.exit(1)

    tool_name  = rospy.get_param("~name", "probe_holder")
    link_name  = rospy.get_param("~link_name", "iiwa_link_ee")

    x = float(rospy.get_param("~x", 0.0))
    y = float(rospy.get_param("~y", 0.0))
    z = float(rospy.get_param("~z", 0.0))
    roll_deg  = float(rospy.get_param("~roll_deg", 0.0))
    pitch_deg = float(rospy.get_param("~pitch_deg", 0.0))
    yaw_deg   = float(rospy.get_param("~yaw_deg", 0.0))

    sx = float(rospy.get_param("~scale_x", 1.0))
    sy = float(rospy.get_param("~scale_y", 1.0))
    sz = float(rospy.get_param("~scale_z", 1.0))

    ns = rospy.get_namespace().rstrip('/') or "iiwa"
    scene = PlanningSceneInterface(ns="/"+ns)
    robot = RobotCommander(ns="/"+ns)
    group = MoveGroupCommander("manipulator", ns="/"+ns)
    rospy.sleep(1.0)

    # pulizia oggetti precedenti
    try:
        scene.remove_attached_object(link_name, name=tool_name)
        scene.remove_world_object(name=tool_name)
    except Exception:
        pass
    wait_for_state_update(scene, tool_name, attached=False, timeout=2.0)

    # Pose del tool relativa al link_name
    pose = PoseStamped()
    pose.header.frame_id = link_name          # <<< fondamentale
    pose.header.stamp = rospy.Time.now()

    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z

    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    yv = math.radians(yaw_deg)
    qx,qy,qz,qw = quat_from_rpy(r, p, yv)
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw

    rospy.loginfo("Attach mesh '%s' a %s (ns=/%s)", tool_name, link_name, ns)
    scene.attach_mesh(link_name, tool_name, pose, mesh_path, size=(sx,sy,sz))
    ok = wait_for_state_update(scene, tool_name, attached=True, timeout=5.0)
    if not ok:
        rospy.logwarn("Non ho potuto verificare l'attacco. Controlla in RViz.")
    rospy.loginfo("Done.")
    roscpp_shutdown()

if __name__ == "__main__":
    main()
