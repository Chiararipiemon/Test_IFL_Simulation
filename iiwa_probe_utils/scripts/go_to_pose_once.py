#!/usr/bin/env python3
import sys, math, time, rospy
import tf.transformations as tft
from geometry_msgs.msg import Pose
from moveit_commander import (
    MoveGroupCommander, RobotCommander, PlanningSceneInterface, roscpp_initialize
)

"""
go_to_pose_once.py (robusto)
- Attende URDF+SRDF validi nel ns /iiwa.
- Mirroring dei parametri in root: /robot_description*, così MoveIt Python non fallisce.
- Frame: world ; EEF: iiwa_link_ee ; pre-approach: z + preclear_z
Parametri:
  _x,_y,_z ; _use_current_orient (bool) ; _roll_deg,_pitch_deg,_yaw_deg (se use_current_orient=false)
  _preclear_z (0.15) ; _vel_scale (0.20)
"""

def pose_from_xyzrpy(x, y, z, rr, pp, yy):
    q = tft.quaternion_from_euler(rr, pp, yy)
    p = Pose(); p.position.x, p.position.y, p.position.z = x, y, z
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
    return p

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
    """Attende che /iiwa/robot_description e /iiwa/robot_description_semantic siano validi."""
    t0 = time.time()
    while time.time() - t0 < timeout and not rospy.is_shutdown():
        ok_urdf = _has_valid_param("/iiwa/robot_description", ["<robot"])
        ok_srdf = _has_valid_param("/iiwa/robot_description_semantic", ["<robot", "group"])
        if ok_urdf and ok_srdf:
            return True
        rospy.sleep(0.2)
    return False

def ensure_root_params_from_iiwa():
    """Copia i parametri chiave da /iiwa/* alle chiavi root equivalenti."""
    pairs = [
        ("/iiwa/robot_description",              "/robot_description"),
        ("/iiwa/robot_description_semantic",     "/robot_description_semantic"),
        ("/iiwa/robot_description_planning",     "/robot_description_planning"),
        ("/iiwa/robot_description_kinematics",   "/robot_description_kinematics"),
    ]
    for src, dst in pairs:
        if rospy.has_param(src):
            _mirror_param(src, dst)

def main():
    roscpp_initialize(sys.argv)
    rospy.init_node("go_to_pose_once")

    # --- parametri utente ---
    x = float(rospy.get_param("~x", 0.00))
    y = float(rospy.get_param("~y", 0.42))
    z = float(rospy.get_param("~z", 0.64))
    use_current_orient = bool(rospy.get_param("~use_current_orient", True))
    roll_deg  = float(rospy.get_param("~roll_deg", 0.0))
    pitch_deg = float(rospy.get_param("~pitch_deg", 0.0))
    yaw_deg   = float(rospy.get_param("~yaw_deg", 0.0))
    preclear_z = float(rospy.get_param("~preclear_z", 0.15))
    vel_scale  = float(rospy.get_param("~vel_scale", 0.20))

    # --- attende modello nel ns /iiwa ---
    if not wait_iiwa_model(timeout=25.0):
        rospy.logerr("URDF/SRDF in /iiwa non pronti. Assicurati che il launch MoveIt sia attivo.")
        return
    rospy.loginfo("URDF+SRDF /iiwa OK. Creo mirror root per MoveIt Python…")
    ensure_root_params_from_iiwa()

    # --- MoveIt in ns /iiwa ---
    ns = "/iiwa"
    robot = RobotCommander(ns=ns)
    scene = PlanningSceneInterface(ns=ns, synchronous=True)
    group = MoveGroupCommander("manipulator", ns=ns)
    group.set_end_effector_link("iiwa_link_ee")
    group.set_pose_reference_frame("world")
    group.set_planning_time(10.0)
    group.set_num_planning_attempts(5)
    group.allow_replanning(True)
    group.set_goal_position_tolerance(0.004)
    group.set_goal_orientation_tolerance(0.05)
    group.set_max_velocity_scaling_factor(vel_scale)
    group.set_max_acceleration_scaling_factor(vel_scale)
    try:
        group.set_planner_id("RRTConnectkConfigDefault")
    except Exception:
        pass

    # --- target pose ---
    if use_current_orient:
        orient = group.get_current_pose("iiwa_link_ee").pose.orientation
        target = Pose(); target.position.x, target.position.y, target.position.z = x, y, z
        target.orientation = orient
    else:
        rr, pp, yy = map(math.radians, [roll_deg, pitch_deg, yaw_deg])
        target = pose_from_xyzrpy(x, y, z, rr, pp, yy)

    # pre-approach sopra il target
    pre = Pose(); pre.position.x, pre.position.y, pre.position.z = x, y, z + preclear_z
    pre.orientation = target.orientation

    # --- esecuzione: pre → target ---
    group.set_start_state_to_current_state()
    group.set_pose_target(pre); ok_pre = group.go(wait=True)
    group.stop(); group.clear_pose_targets()

    group.set_pose_target(target); ok_tar = group.go(wait=True)
    group.stop(); group.clear_pose_targets()

    rospy.loginfo("Risultato: pre_ok=%s  target_ok=%s", ok_pre, ok_tar)
    print("DONE: pre_ok=%s target_ok=%s" % (ok_pre, ok_tar))

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

