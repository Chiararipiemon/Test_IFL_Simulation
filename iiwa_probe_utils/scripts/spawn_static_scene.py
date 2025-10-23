#!/usr/bin/env python3
# Adds two static boxes to the MoveIt PlanningScene, supporting namespaced move_group (e.g., /iiwa).
# Objects:
#  - 'pedestal': cube under the base (top at z=0)
#  - 'patient_table': rectangular bed in front of robot (+X)

import sys
import time
import rospy
from geometry_msgs.msg import PoseStamped

# MoveIt commander (Python)
from moveit_commander import PlanningSceneInterface, RobotCommander, roscpp_initialize, roscpp_shutdown

# Yaw support
try:
    import tf.transformations as tft
    def quat_from_euler(roll, pitch, yaw):
        q = tft.quaternion_from_euler(roll, pitch, yaw)
        return q
except Exception:
    import math
    def quat_from_euler(roll, pitch, yaw):
        cy = math.cos(yaw * 0.5); sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5);  sr = math.sin(roll * 0.5)
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy
        return (qx, qy, qz, qw)


def _srv(ns, name):
    ns = (ns or '').strip('/')
    name = name.lstrip('/')
    return '/' + '/'.join([ns, name]) if ns else '/' + name


def wait_for_moveit_services(ns='', timeout=45.0):
    """Wait for core MoveIt services so PlanningSceneInterface will work."""
    start = time.time()
    services = [_srv(ns, 'get_planning_scene'), _srv(ns, 'apply_planning_scene')]
    for s in services:
        try:
            rospy.loginfo("Waiting for service %s ...", s)
            remaining = max(0.1, timeout - (time.time() - start))
            rospy.wait_for_service(s, timeout=remaining)
            rospy.loginfo("OK: %s is available.", s)
        except rospy.ROSException:
            rospy.logwarn("Timed out waiting for %s", s)
            return False
    return True


def add_box(scene, name, size_xyz, frame_id, xyz, rpy=(0.0, 0.0, 0.0)):
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = frame_id
    pose.pose.position.x = float(xyz[0])
    pose.pose.position.y = float(xyz[1])
    pose.pose.position.z = float(xyz[2])
    qx, qy, qz, qw = quat_from_euler(rpy[0], rpy[1], rpy[2])
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw

    rospy.loginfo("Adding box '%s' in frame '%s' at %s with size %s", name, frame_id, xyz, size_xyz)
    scene.add_box(name, pose, tuple(map(float, size_xyz)))

    # Wait until the object shows up in the scene
    for i in range(60):
        names = scene.get_known_object_names()
        if name in names:
            rospy.loginfo("Confirmed in scene: %s", name)
            return True
        rospy.sleep(0.25)
    rospy.logwarn("Object '%s' not found in scene after timeout.", name)
    return False


def main():
    roscpp_initialize(sys.argv)
    rospy.init_node("spawn_static_scene", anonymous=True)

    # Let RViz/MoveIt come up
    rospy.sleep(1.0)

    moveit_ns = rospy.get_param("~moveit_ns", "")
    rospy.loginfo("Using MoveIt namespace: '%s'", moveit_ns or "<none>")

    ok = wait_for_moveit_services(ns=moveit_ns, timeout=45.0)
    if not ok:
        rospy.logwarn("MoveIt services not ready yet under ns '%s'. Objects may not appear.", moveit_ns)

    # Namespaced MoveIt Commander objects
    robot = RobotCommander(ns=moveit_ns)  # uses /<ns>/robot_description if provided there
    scene = PlanningSceneInterface(ns=moveit_ns)

    planning_frame = robot.get_planning_frame()
    rospy.loginfo("MoveIt planning frame: %s", planning_frame)

    world_frame = rospy.get_param("~world_frame", "world")

    # ---- Pedestal (cube) ----
    pedestal_size = float(rospy.get_param("~pedestal_size", 0.60))  # cube side
    pedestal_z_top = float(rospy.get_param("~pedestal_z_top", 0.0))
    pedestal_center_z = pedestal_z_top - pedestal_size / 2.0
    add_box(scene,
            name="pedestal",
            size_xyz=(pedestal_size, pedestal_size, pedestal_size),
            frame_id=world_frame,
            xyz=(0.0, 0.0, pedestal_center_z),
            rpy=(0.0, 0.0, 0.0))

    # ---- Patient table (rectangular box) ----
    table_length = float(rospy.get_param("~table_length", 1.80))
    table_width  = float(rospy.get_param("~table_width",  0.60))
    table_height = float(rospy.get_param("~table_height", 0.75))
    table_x      = float(rospy.get_param("~table_x",      0.90))
    table_y      = float(rospy.get_param("~table_y",      0.00))
    table_yaw    = float(rospy.get_param("~table_yaw",    0.00))

    table_center_z = table_height / 2.0  # rests on the floor
    add_box(scene,
            name="patient_table",
            size_xyz=(table_length, table_width, table_height),
            frame_id=world_frame,
            xyz=(table_x, table_y, table_center_z),
            rpy=(0.0, 0.0, table_yaw))

    names = scene.get_known_object_names()
    rospy.loginfo("Known objects in scene now: %s", names)

    rospy.loginfo("If you don't see the boxes in RViz:")
    rospy.loginfo("- Set RViz 'Fixed Frame' to '%s'", world_frame)
    rospy.loginfo("- In MoveIt 'PlanningScene' display, enable 'Scene Geometry' (Collision).")
    rospy.spin()
    roscpp_shutdown()


if __name__ == "__main__":
    main()
