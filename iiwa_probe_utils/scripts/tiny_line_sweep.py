#!/usr/bin/env python3
import sys, math, time, rospy
from geometry_msgs.msg import Pose
from moveit_commander import (
    MoveGroupCommander, RobotCommander, PlanningSceneInterface, roscpp_initialize
)

def _has_valid_param(key, must_contain_tokens):
    try:
        val = rospy.get_param(key, "")
        if not isinstance(val, str) or not val:
            return False
        return all(tok in val for tok in must_contain_tokens)
    except Exception:
        return False

def _mirror_param(src, dst):
    try:
        if rospy.has_param(src):
            rospy.set_param(dst, rospy.get_param(src))
            rospy.loginfo("Param mirror: %s -> %s", src, dst)
    except Exception as e:
        rospy.logwarn("Param mirror failed %s -> %s: %s", src, dst, e)

def wait_iiwa_model(timeout=20.0):
    t0 = time.time()
    while time.time() - t0 < timeout and not rospy.is_shutdown():
        ok_urdf = _has_valid_param("/iiwa/robot_description", ["<robot"])
        ok_srdf = _has_valid_param("/iiwa/robot_description_semantic", ["<robot", "group"])
        if ok_urdf and ok_srdf:
            return True
        rospy.sleep(0.2)
    return False

def ensure_root_params_from_iiwa():
    pairs = [
        ("/iiwa/robot_description",            "/robot_description"),
        ("/iiwa/robot_description_semantic",   "/robot_description_semantic"),
        ("/iiwa/robot_description_planning",   "/robot_description_planning"),
        ("/iiwa/robot_description_kinematics", "/robot_description_kinematics"),
    ]
    for src, dst in pairs:
        if rospy.has_param(src):
            _mirror_param(src, dst)

def make_pose(x,y,z,orient):
    p = Pose()
    p.position.x, p.position.y, p.position.z = float(x), float(y), float(z)
    p.orientation = orient
    return p

def main():
    roscpp_initialize(sys.argv)
    rospy.init_node("tiny_line_sweep")

    # Parametri
    cx = float(rospy.get_param("~cx", 0.00))
    cy = float(rospy.get_param("~cy", 0.42))
    cz = float(rospy.get_param("~cz", 0.66))
    length = float(rospy.get_param("~length", 0.20))
    along  = rospy.get_param("~along", "y")      # "x" o "y"
    vel_scale = float(rospy.get_param("~vel_scale", 0.2))
    avoid_collisions = bool(rospy.get_param("~avoid_collisions", True))

    # Assicurati che URDF/SRDF in /iiwa ci siano e fai mirror in root
    if not wait_iiwa_model(timeout=25.0):
        rospy.logerr("URDF/SRDF in /iiwa non pronti. Avvia il launch MoveIt.")
        return
    ensure_root_params_from_iiwa()

    # MoveIt in ns /iiwa
    robot = RobotCommander(ns="/iiwa")
    scene = PlanningSceneInterface(ns="/iiwa", synchronous=True)
    group = MoveGroupCommander("manipulator", ns="/iiwa")
    group.set_end_effector_link("iiwa_link_ee")
    group.set_pose_reference_frame("world")
    group.set_planning_time(10.0)
    group.set_num_planning_attempts(5)
    group.allow_replanning(True)
    group.set_max_velocity_scaling_factor(vel_scale)
    group.set_max_acceleration_scaling_factor(vel_scale)

    # orientazione ATTUALE dell'EEF
    orient = group.get_current_pose("iiwa_link_ee").pose.orientation

    half = length/2.0
    if along.lower() == "y":
        ys = [cy - half, cy - half/2, cy, cy + half/2, cy + half]
        pts = [make_pose(cx, y, cz, orient) for y in ys]
    else:
        xs = [cx - half, cx - half/2, cx, cx + half/2, cx + half]
        pts = [make_pose(x, cy, cz, orient) for x in xs]

    # move-to-start (pre-lift)
    start = pts[0]
    pre = make_pose(start.position.x, start.position.y, start.position.z + 0.10, start.orientation)
    group.set_start_state_to_current_state()
    group.set_pose_target(pre); ok1 = group.go(wait=True); group.stop(); group.clear_pose_targets()
    group.set_pose_target(start); ok2 = group.go(wait=True); group.stop(); group.clear_pose_targets()
    if not (ok1 and ok2):
        rospy.logerr("Move-to-start failed (alza cz a 0.68 o accorcia length).")
        return

    # cartesiano
    eef_step = 0.01
    try:
        plan, fraction = group.compute_cartesian_path(pts, eef_step, avoid_collisions)
    except TypeError:
        plan, fraction = group.compute_cartesian_path(pts, eef_step, 0.0)

    rospy.loginfo("Cartesian fraction: %.1f%%", 100.0*float(fraction))
    if getattr(plan, "joint_trajectory", None) and len(plan.joint_trajectory.points)>1 and fraction>0.5:
        group.execute(plan, wait=True)
        rospy.loginfo("Line executed.")
    else:
        rospy.logerr("Line planning failed (alza cz, accorcia length, o set avoid_collisions:=false).")

if __name__ == "__main__":
    main()

