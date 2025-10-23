#!/usr/bin/env python3
import sys, math, rospy
import tf.transformations as tft
from geometry_msgs.msg import Pose
from moveit_commander import (
    MoveGroupCommander, PlanningSceneInterface, RobotCommander, roscpp_initialize
)

"""
Serpentina sopra la ROI usando iiwa_link_ee.
Step:
  1) Pre-approach sopra il centro ROI (cz + _preclear_z)
  2) Pre-lift sopra il primo punto (start.z + _prelift)
  3) Move-to-start
  4) Cartesian path sui waypoints

Parametri:
  _group=manipulator, _L_tip=0.12
  ROI: _roi_cx,_roi_cy,_roi_cz (m), _roi_dx,_roi_dy (m)
  _line_step=0.01 (m), _row_step=0.02 (m)
  _approach_angle_deg=10 (tilt X), _down_offset=0.00 (m)
  _preclear_z=0.15, _prelift=0.10 (m)
  _vel_scale=0.2
"""

def pose_from_xyzrpy(x, y, z, rr, pp, yy):
    q = tft.quaternion_from_euler(rr, pp, yy)
    p = Pose()
    p.position.x, p.position.y, p.position.z = x, y, z
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
    return p

def ee_from_tip(tip_pose, L_tip):
    pos = [tip_pose.position.x, tip_pose.position.y, tip_pose.position.z]
    q   = [tip_pose.orientation.x, tip_pose.orientation.y, tip_pose.orientation.z, tip_pose.orientation.w]
    T_tip = tft.compose_matrix(translate=pos) @ tft.quaternion_matrix(q)
    T_ee  = T_tip @ tft.compose_matrix(translate=[0, 0, -L_tip])  # arretra lungo -Z_tip
    trans = tft.translation_from_matrix(T_ee)
    quat  = tft.quaternion_from_matrix(T_ee)
    p = Pose()
    p.position.x, p.position.y, p.position.z = trans
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quat
    return p

def main():
    roscpp_initialize(sys.argv)
    rospy.init_node("plan_execute_sweep")

    # --- Parametri utente ---
    group_name = rospy.get_param("~group", "manipulator")
    L_tip      = float(rospy.get_param("~L_tip", 0.12))

    cx = float(rospy.get_param("~roi_cx", 0.0))
    cy = float(rospy.get_param("~roi_cy", 0.50))
    cz = float(rospy.get_param("~roi_cz", 0.10))
    dx = float(rospy.get_param("~roi_dx", 0.20))
    dy = float(rospy.get_param("~roi_dy", 0.30))

    line_step  = float(rospy.get_param("~line_step", 0.01))
    row_step   = float(rospy.get_param("~row_step",  0.02))
    approach_angle_deg = float(rospy.get_param("~approach_angle_deg", 10.0))
    down_offset = float(rospy.get_param("~down_offset", 0.0))
    preclear_z  = float(rospy.get_param("~preclear_z", 0.15))
    prelift     = float(rospy.get_param("~prelift", 0.10))
    vel_scale   = float(rospy.get_param("~vel_scale", 0.2))

    # --- MoveIt (namespace /iiwa) ---
    robot = RobotCommander(ns="/iiwa")
    scene = PlanningSceneInterface(ns="/iiwa", synchronous=True)
    group = MoveGroupCommander(group_name, ns="/iiwa")
    group.set_end_effector_link("iiwa_link_ee")
    group.set_pose_reference_frame("world")
    group.set_planning_time(10.0)
    group.set_num_planning_attempts(5)
    group.allow_replanning(True)
    group.set_goal_position_tolerance(0.004)    # 4 mm
    group.set_goal_orientation_tolerance(0.05)  # ~3°
    group.set_max_velocity_scaling_factor(vel_scale)
    group.set_max_acceleration_scaling_factor(vel_scale)
    try:
        group.set_planner_id("RRTConnectkConfigDefault")
    except Exception:
        pass
    rospy.sleep(1.0)

    # Orientazione del tip (tilt intorno a X)
    face_down = bool(rospy.get_param("~face_down", True))
    yaw_deg   = float(rospy.get_param("~yaw_deg", 0.0))
    base_roll = math.pi if face_down else 0.0
    rr = base_roll + math.radians(approach_angle_deg)  # 180° (+ eventuale tilt) intorno a X
    pp = 0.0                                           # niente pitch
    yy = math.radians(yaw_deg)                         # yaw sul piano lettino


    # --- Waypoints della serpentina (in TIP, poi convertiti in EE) ---
    x0 = cx - dx/2.0
    x1 = cx + dx/2.0
    y0 = cy - dy/2.0
    rows = int(dy / row_step) + 1

    waypoints = []
    direction = +1
    z_tip = cz - down_offset

    for r in range(rows):
        y = y0 + r * row_step
        xs = [x0, x1] if direction > 0 else [x1, x0]
        npts = max(2, int(abs(xs[1] - xs[0]) / line_step) + 1)
        for i in range(npts):
            x = xs[0] + (xs[1] - xs[0]) * (i / float(npts - 1))
            tip = pose_from_xyzrpy(x, y, z_tip, rr, pp, yy)
            ee  = ee_from_tip(tip, L_tip)
            waypoints.append(ee)
        direction *= -1

    if len(waypoints) < 2:
        rospy.logerr("Waypoints insufficienti; aumenta ROI o riduci _line_step.")
        return

    # --- 1) PRE-APPROACH: sopra il centro ROI (in TIP → converti a EE) ---
    tip_app = pose_from_xyzrpy(cx, cy, cz + preclear_z, rr, pp, yy)
    ee_app  = ee_from_tip(tip_app, L_tip)

    group.set_start_state_to_current_state()
    group.set_pose_target(ee_app)
    ok_app = group.go(wait=True)
    group.stop(); group.clear_pose_targets()
    if not ok_app:
        rospy.logwarn("Pre-approach fallito a cz+%.0fmm, ritento più alto...", preclear_z*1000.0)
        tip_app2 = pose_from_xyzrpy(cx, cy, cz + preclear_z + 0.10, rr, pp, yy)  # +10 cm
        ee_app2  = ee_from_tip(tip_app2, L_tip)
        group.set_pose_target(ee_app2)
        ok_app = group.go(wait=True)
        group.stop(); group.clear_pose_targets()
        if not ok_app:
            rospy.logerr("Pre-approach ancora fallito. Alza _roi_cz o riduci tilt.")
            return

    # --- 2) PRE-LIFT: sopra il primo punto (start + prelift) ---
    start_pose = waypoints[0]
    pre_start = Pose()
    pre_start.position.x = start_pose.position.x
    pre_start.position.y = start_pose.position.y
    pre_start.position.z = start_pose.position.z + prelift
    pre_start.orientation = start_pose.orientation

    group.set_pose_target(pre_start)
    ok_pre = group.go(wait=True)
    group.stop(); group.clear_pose_targets()
    if not ok_pre:
        rospy.logwarn("Pre-lift fallito a +%.0fmm, ritento +50mm...", prelift*1000.0)
        pre_start.position.z = start_pose.position.z + prelift + 0.05
        group.set_pose_target(pre_start)
        ok_pre = group.go(wait=True)
        group.stop(); group.clear_pose_targets()
        if not ok_pre:
            rospy.logerr("Pre-lift fallito. Alza _roi_cz o allarga ROI.")
            return

    # --- 3) MOVE-TO-START ---
    group.set_pose_target(start_pose)
    ok_start = group.go(wait=True)
    group.stop(); group.clear_pose_targets()
    if not ok_start:
        rospy.logerr("Move-to-start fallito. Suggerimenti: _roi_cz >= 0.70, _approach_angle_deg più basso, ROI più ampia.")
        return

    # --- 4) CARTESIAN PATH ---
    eef_step = 0.005  # 5 mm
    try:
        plan, fraction = group.compute_cartesian_path(waypoints, eef_step, True)  # avoid_collisions=True (Noetic)
    except TypeError:
        plan, fraction = group.compute_cartesian_path(waypoints, eef_step, 0.0)   # old API

    fraction = float(fraction)
    rospy.loginfo("Cartesian path fraction: %.1f%%", 100.0 * fraction)

    if getattr(plan, "joint_trajectory", None) and len(plan.joint_trajectory.points) > 1 and fraction > 0.1:
        group.execute(plan, wait=True)
        rospy.loginfo("Esecuzione completata.")
    else:
        rospy.logerr("Piano vuoto o frazione bassa. Alza _roi_cz (>=0.72), riduci _line_step (0.004), amplia ROI.")

if __name__ == "__main__":
    main()

