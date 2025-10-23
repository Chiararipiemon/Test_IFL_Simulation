#!/usr/bin/env python3
# Scene spawner for IFL-CAMP/iiwa_stack using only PlanningSceneInterface.
# Guarantees the table bottom sits on the SAME floor as the pedestal bottom when use_pedestal_floor:=true.

import sys
import time
import rospy
from geometry_msgs.msg import PoseStamped
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown

def _srv(ns, name):
    ns = (ns or '').strip('/')
    name = name.lstrip('/')
    return '/' + '/'.join([ns, name]) if ns else '/' + name

def wait_for_moveit_services(ns, timeout=45.0):
    start = time.time()
    needed = [_srv(ns, 'get_planning_scene'), _srv(ns, 'apply_planning_scene')]
    for s in needed:
        try:
            rospy.loginfo("Waiting for service %s ...", s)
            remaining = max(0.1, timeout - (time.time() - start))
            rospy.wait_for_service(s, timeout=remaining)
            rospy.loginfo("OK: %s is available.", s)
        except rospy.ROSException:
            rospy.logwarn("Timed out waiting for %s", s)
            return False
    return True

def quat_from_euler(roll, pitch, yaw):
    try:
        import tf.transformations as tft
        return tft.quaternion_from_euler(roll, pitch, yaw)
    except Exception:
        import math
        cy = math.cos(yaw * 0.5); sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5);  sr = math.sin(roll * 0.5)
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy
        return (qx, qy, qz, qw)

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

    rospy.loginfo("Adding box '%s' in frame '%s' at %s size %s", name, frame_id, xyz, size_xyz)
    scene.add_box(name, pose, tuple(map(float, size_xyz)))

    for _ in range(60):
        if name in scene.get_known_object_names():
            rospy.loginfo("Confirmed in scene: %s", name)
            return True
        rospy.sleep(0.2)
    rospy.logwarn("Object '%s' not confirmed after timeout.", name)
    return False

def main():
    roscpp_initialize(sys.argv)
    rospy.init_node("spawn_static_scene", anonymous=True)

    moveit_ns = rospy.get_param("~moveit_ns", "iiwa")
    world_frame = rospy.get_param("~world_frame", "world")

    wait_for_moveit_services(moveit_ns, timeout=45.0)

    scene = PlanningSceneInterface(ns=moveit_ns)
    rospy.sleep(1.0)

    # Read pedestal/table params
    pedestal_size = float(rospy.get_param("~pedestal_size", 0.60))
    pedestal_z_top = float(rospy.get_param("~pedestal_z_top", 0.0))

    use_ped_floor = rospy.get_param("~use_pedestal_floor", True)
    floor_z_param = rospy.get_param("~floor_z", 0.0)

    # Compute a single FLOOR level and place BOTH objects on it
    if use_ped_floor:
        floor_z = pedestal_z_top - pedestal_size  # bottom of the pedestal
        rospy.loginfo("Using pedestal floor. floor_z = %.3f (top=%.3f, size=%.3f)", floor_z, pedestal_z_top, pedestal_size)
    else:
        floor_z = float(floor_z_param)
        rospy.loginfo("Using provided floor_z = %.3f", floor_z)

    # Place the pedestal so its BOTTOM sits at floor_z
    pedestal_center_z = floor_z + pedestal_size / 2.0
    add_box(scene, "pedestal",
            (pedestal_size, pedestal_size, pedestal_size),
            world_frame, (0.0, 0.0, pedestal_center_z))

    # Table params
    table_length = float(rospy.get_param("~table_length", 1.80))
    table_width  = float(rospy.get_param("~table_width",  0.60))
    table_height = float(rospy.get_param("~table_height", 0.75))
    table_x      = float(rospy.get_param("~table_x",      0.90))
    table_y      = float(rospy.get_param("~table_y",      0.00))
    table_yaw    = float(rospy.get_param("~table_yaw",    0.00))

    # Place table so its BOTTOM sits at the SAME floor_z
    table_center_z = floor_z + table_height / 2.0
    add_box(scene, "patient_table",
            (table_length, table_width, table_height),
            world_frame, (table_x, table_y, table_center_z),
            (0.0, 0.0, table_yaw))

    rospy.loginfo("Placed with shared floor_z = %.3f. Objects: %s", floor_z, scene.get_known_object_names())
    rospy.spin()
    roscpp_shutdown()

if __name__ == "__main__":
    main()
