#!/usr/bin/env python3
import rospy, math
from geometry_msgs.msg import Pose, Quaternion
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander
import tf.transformations as tft

def q_multiply(q1, q2):
    # both as [x,y,z,w] -> returns [x,y,z,w]
    return tft.quaternion_multiply(q1, q2)

def main():
    rospy.init_node("preposition_probe", anonymous=True)

    # ===== Parametri =====
    cx = rospy.get_param("~cx", 0.00)
    cy = rospy.get_param("~cy", 0.42)
    cz = rospy.get_param("~cz", 0.66)
    preclear_z = rospy.get_param("~preclear_z", 0.20)
    vel_scale = rospy.get_param("~vel_scale", 0.20)
    flip_axis = rospy.get_param("~flip_axis", "x")  # "x", "y", "z" o "" (nessun flip)

    # ===== MoveIt setup =====
    # robot_description può essere in / o /iiwa
    desc_ns = "/iiwa" if rospy.has_param("/iiwa/robot_description") else ""
    group_ns = "/iiwa"

    robot = RobotCommander(ns=desc_ns)
    scene = PlanningSceneInterface(ns=group_ns, synchronous=True)
    group = MoveGroupCommander("manipulator", ns=group_ns)
    group.set_pose_reference_frame("world")

    eef_link = group.get_end_effector_link() or "tool0"
    group.set_end_effector_link(eef_link)
    group.set_max_velocity_scaling_factor(vel_scale)
    group.set_max_acceleration_scaling_factor(vel_scale)
    group.allow_replanning(True)
    group.set_planning_time(20.0)

    rospy.sleep(0.5)
    rospy.loginfo("EEF link: %s", eef_link)

    # ===== Orientazione desiderata =====
    # Prende l'orientazione attuale e (opzionale) applica un flip di 180° intorno ad un asse
    current_q = group.get_current_pose(eef_link).pose.orientation
    qc = [current_q.x, current_q.y, current_q.z, current_q.w]

    if flip_axis in ("x", "y", "z"):
        axis_map = {"x": (1,0,0), "y": (0,1,0), "z": (0,0,1)}
        ax = axis_map[flip_axis]
        qf = tft.quaternion_about_axis(math.pi, ax)  # 180°
        qn = q_multiply(qc, qf)
    else:
        qn = qc

    q = Quaternion(x=qn[0], y=qn[1], z=qn[2], w=qn[3])

    # ===== Pose target: sopra il centro della raster (quota di preclear) =====
    p = Pose()
    p.position.x = cx
    p.position.y = cy
    p.position.z = cz + preclear_z  # sopra al paziente
    p.orientation = q

    rospy.loginfo("Moving to preclear pose at (%.3f, %.3f, %.3f) with flipped axis: %s",
                  p.position.x, p.position.y, p.position.z, flip_axis)

    group.set_start_state_to_current_state()
    group.set_pose_target(p, eef_link)
    plan = group.plan()
    ok = group.execute(plan, wait=True)
    group.stop()
    group.clear_pose_targets()

    if ok:
        rospy.loginfo("Pre-posizionamento COMPLETATO.")
    else:
        rospy.logerr("Pre-posizionamento FALLITO (prova altro flip_axis o alza preclear_z).")

if __name__ == "__main__":
    main()
