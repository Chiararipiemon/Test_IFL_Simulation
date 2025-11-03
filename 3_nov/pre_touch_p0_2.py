#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import numpy as np

import rospy
import tf2_ros
import tf2_geometry_msgs
import tf.transformations as tft

from geometry_msgs.msg import PointStamped, PoseStamped, Pose
from moveit_commander import (
    roscpp_initialize, roscpp_shutdown, MoveGroupCommander
)

def norm(v, eps=1e-12):
    n = np.linalg.norm(v)
    return v / n if n > eps else np.zeros_like(v)

def quat_from_z(z_axis, x_hint=None):
    """
    Crea una terna destra (X,Y,Z) con Z allineato a z_axis e restituisce il quaternione (x,y,z,w).
    """
    z = norm(np.array(z_axis, dtype=float))
    if x_hint is None:
        up = np.array([0.0, 0.0, 1.0])
    else:
        up = np.array(x_hint, dtype=float)
    x = np.cross(up, z)
    if np.linalg.norm(x) < 1e-6:
        up = np.array([1.0, 0.0, 0.0])
        x = np.cross(up, z)
    x = norm(x)
    y = norm(np.cross(z, x))
    # re-ortho
    x = norm(np.cross(y, z))
    R = np.column_stack((x, y, z))
    T = np.eye(4)
    T[:3, :3] = R
    qx, qy, qz, qw = tft.quaternion_from_matrix(T)
    return (qx, qy, qz, qw)

def to_dict(joint_names, values):
    assert len(joint_names) == len(values)
    return {name: float(val) for name, val in zip(joint_names, values)}

class PreTouchP0(object):
    def __init__(self):
        roscpp_initialize(sys.argv)
        rospy.init_node("pre_touch_p0")

        # Parametri
        self.group_name         = rospy.get_param("~group_name", "manipulator")
        self.ee_link            = rospy.get_param("~ee_link", "iiwa_link_ee")
        self.clicked_topic      = rospy.get_param("~clicked_topic", "/clicked_point")
        self.ref_frame          = rospy.get_param("~ref_frame", "world")
        self.speed_scale        = float(rospy.get_param("~speed_scale", 0.2))
        self.planning_time      = float(rospy.get_param("~planning_time", 20.0))
        self.planning_attempts  = int(rospy.get_param("~planning_attempts", 20))
        self.tip_to_contact     = float(rospy.get_param("~tip_to_contact", 0.00))  # m, punta avanti a +Z_tool
        self.approach_dist      = float(rospy.get_param("~approach_dist", 0.08))   # m, distanza del pre-approach
        self.eef_step           = float(rospy.get_param("~eef_step", 0.003))
        self.do_home_zero       = bool(rospy.get_param("~do_home_zero", True))
        self.point_tool_to_p0   = bool(rospy.get_param("~point_tool_to_p0", True))  # orienta Z_tool verso P0

        # Config pre-pre-approach (rad)
        self.pre_joints = [-2.529, 0.271, -0.268, 1.141, 2.932, 1.581, 0.174]

        # MoveIt
        ns = rospy.get_namespace()  # es. "/iiwa/"
        robot_description = (ns if ns != "/" else "") + "robot_description"
        self.group = MoveGroupCommander(self.group_name, robot_description=robot_description, ns=ns)
        self.group.set_end_effector_link(self.ee_link)
        self.group.set_pose_reference_frame(self.ref_frame)
        self.group.set_max_velocity_scaling_factor(self.speed_scale)
        self.group.set_max_acceleration_scaling_factor(self.speed_scale)
        self.group.set_planning_time(self.planning_time)
        self.group.set_num_planning_attempts(self.planning_attempts)
        self.group.allow_replanning(True)
        # Tolleranze più lasche per aiutare l'IK
        self.group.set_goal_position_tolerance(0.002)       # 2 mm
        self.group.set_goal_orientation_tolerance(0.20)     # ~11.5°

        # TF
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(60.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Sequenza: home -> pre-pre-approach
        self.move_home_and_pre()

        # Sub per P0
        rospy.Subscriber(self.clicked_topic, PointStamped, self.clicked_cb, queue_size=10)
        rospy.loginfo("Pronto: clicca un punto P0 con RViz (Publish Point). Pre-approach + avvicinamento cartesiano fino a P0.")
        rospy.spin()

    # ---- Movimenti base ----
    def go_joint_dict(self, target_dict, msg=""):
        self.group.set_start_state_to_current_state()
        self.group.set_joint_value_target(target_dict)
        ok = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
        if not ok:
            rospy.logerr("Pianificazione a joint-target fallita: %s", msg)
        return ok

    def move_home_and_pre(self):
        joint_names = self.group.get_active_joints()
        if self.do_home_zero:
            zeros = [0.0]*len(joint_names)
            home = to_dict(joint_names, zeros)
            rospy.loginfo("Vado in home (tutti i joint a 0).")
            self.go_joint_dict(home, "home_zero")
        # Pre-pre-approach
        current = dict(zip(joint_names, self.group.get_current_joint_values()))
        desired = current.copy()
        for i in range(7):
            name = f"iiwa_joint_{i+1}"
            if name in desired:
                desired[name] = self.pre_joints[i]
        rospy.loginfo("Vado in pre-pre-approach (joint set specificati).")
        self.go_joint_dict(desired, "pre_pre_approach")

    # ---- Callback su P0 ----
    def clicked_cb(self, msg):
        # Porta P0 nel ref_frame
        if msg.header.frame_id and msg.header.frame_id != self.ref_frame:
            try:
                tfm = self.tf_buffer.lookup_transform(self.ref_frame, msg.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
                p = tf2_ros.do_transform_point(msg, tfm)
                target = np.array([p.point.x, p.point.y, p.point.z], dtype=float)
            except Exception as e:
                rospy.logwarn("TF fallita %s -> %s: %s", msg.header.frame_id, self.ref_frame, str(e))
                return
        else:
            target = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=float)

        # Pose corrente
        cur = self.group.get_current_pose(self.ee_link)
        p_now = np.array([cur.pose.position.x, cur.pose.position.y, cur.pose.position.z], dtype=float)
        q_now = np.array([cur.pose.orientation.x, cur.pose.orientation.y, cur.pose.orientation.z, cur.pose.orientation.w], dtype=float)

        # Orientazione desiderata
        if self.point_tool_to_p0:
            z_tool = norm(target - p_now)  # punta verso P0
            qx, qy, qz, qw = quat_from_z(z_tool)
        else:
            qx, qy, qz, qw = q_now.tolist()

        # Pre-approach e finale (considerando l'offset punta)
        # ee_link da posizionare: a distanza (tip_to_contact + approach_dist) prima di P0 lungo Z_tool
        z_tool_vec = tft.quaternion_matrix([qx, qy, qz, qw])[:3, 2]
        pre_pos = target - (self.tip_to_contact + self.approach_dist) * z_tool_vec
        final_pos = target - self.tip_to_contact * z_tool_vec

        # 1) Vai al pre-approach con go()
        pre_ps = PoseStamped()
        pre_ps.header.frame_id = self.ref_frame
        pre_ps.pose = Pose()
        pre_ps.pose.position.x, pre_ps.pose.position.y, pre_ps.pose.position.z = pre_pos.tolist()
        pre_ps.pose.orientation.x, pre_ps.pose.orientation.y, pre_ps.pose.orientation.z, pre_ps.pose.orientation.w = (qx, qy, qz, qw)

        rospy.loginfo("Pre-approach verso P0 (dist=%.3f m).", self.approach_dist)
        self.group.set_start_state_to_current_state()
        self.group.set_pose_target(pre_ps)
        ok = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
        if not ok:
            rospy.logerr("Pre-approach fallito (go()=False). Prova a ridurre approach_dist o aumentare planning_time.")
            return

        # 2) Avvicinamento cartesiano rettilineo fino a P0
        rospy.loginfo("Avvicinamento cartesiano fino a P0...")
        waypoints = []

        final_pose = Pose()
        final_pose.position.x, final_pose.position.y, final_pose.position.z = final_pos.tolist()
        final_pose.orientation.x, final_pose.orientation.y, final_pose.orientation.z, final_pose.orientation.w = (qx, qy, qz, qw)
        waypoints.append(final_pose)

        traj, fraction = self.group.compute_cartesian_path(
            waypoints, eef_step=self.eef_step, jump_threshold=0.0, avoid_collisions=True
        )
        rospy.loginfo("Frazione cartesian path: %.1f%%", 100.0*fraction)
        if getattr(traj, "joint_trajectory", None) and len(traj.joint_trajectory.points) > 0:
            self.group.execute(traj, wait=True)
            self.group.stop()
            rospy.loginfo("P0 raggiunto (considerando tip_to_contact=%.3f m).", self.tip_to_contact)
        else:
            rospy.logerr("Traiettoria cartesiana vuota. Prova a ridurre eef_step o approach_dist.")

if __name__ == "__main__":
    try:
        node = PreTouchP0()
    except rospy.ROSInterruptException:
        pass
    finally:
        roscpp_shutdown()

