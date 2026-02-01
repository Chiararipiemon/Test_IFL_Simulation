#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import numpy as np

import rospy
import tf2_ros
from geometry_msgs.msg import PointStamped, PoseStamped, Pose
from moveit_commander import (
    roscpp_initialize, roscpp_shutdown, MoveGroupCommander
)

def to_dict(joint_names, values):
    """Crea un dict {name: value} da lista nomi e lista valori (stessa lunghezza)."""
    assert len(joint_names) == len(values), "Lunghezze joint_names/values diverse"
    return {name: float(val) for name, val in zip(joint_names, values)}

class PreTouchP0(object):
    def __init__(self):
        roscpp_initialize(sys.argv)
        rospy.init_node("pre_touch_p0")

        # Parametri
        self.group_name     = rospy.get_param("~group_name", "manipulator")
        self.ee_link        = rospy.get_param("~ee_link", "iiwa_link_ee")
        self.clicked_topic  = rospy.get_param("~clicked_topic", "/clicked_point")
        self.ref_frame      = rospy.get_param("~ref_frame", "world")
        self.speed_scale    = float(rospy.get_param("~speed_scale", 0.2))
        self.planning_time  = float(rospy.get_param("~planning_time", 15.0))
        self.planning_attempts = int(rospy.get_param("~planning_attempts", 15))
        # Offset tra origine TCP (ee_link) e punto di contatto sonda lungo +Z_tool (m).
        # Se il tuo ee_link è già la punta della sonda, lascia 0.0
        self.tip_to_contact = float(rospy.get_param("~tip_to_contact", 0.0))
        # Vuoi passare dalla home a zero prima di andare al pre-preapproach?
        self.do_home_zero   = bool(rospy.get_param("~do_home_zero", True))

        # Config pre-pre-approach (radiani)
        # Ho scelto questi valori muovendo il robot manualmente nella gui di Moveit!
        self.pre_joints = [
            -2.529,  # iiwa_joint_1
             0.271,  # iiwa_joint_2
            -0.268,  # iiwa_joint_3
             1.141,  # iiwa_joint_4
             2.932,  # iiwa_joint_5
             1.581,  # iiwa_joint_6
             0.174   # iiwa_joint_7
        ]

        # MoveIt connection
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

        # TF buffer (per sicurezza, se /clicked_point non è nel ref_frame)
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(60.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Sequenza iniziale: home(0) -> pre-pre-approach 
        self.move_home_and_pre()

        # Subscriber per il click di P0: praticamente qui clicco un punto io
        rospy.Subscriber(self.clicked_topic, PointStamped, self.clicked_cb, queue_size=10)
        rospy.loginfo("Pronto: clicca un punto P0 con RViz (Publish Point). Il robot porterà la sonda a toccare P0.")
        rospy.spin()

    # ----------------- Helpers di movimento -----------------
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
        # Per iiwa di solito: ['iiwa_joint_1', ..., 'iiwa_joint_7']
        if len(joint_names) < 7:
            rospy.logwarn("Attenzione: numero di giunti attivi inatteso (%d).", len(joint_names))

        # 1) Home a zero (opzionale)
        if self.do_home_zero:
            zeros = [0.0]*len(joint_names)
            home = to_dict(joint_names, zeros)
            rospy.loginfo("Vado in home (tutti i joint a 0).")
            if not self.go_joint_dict(home, "home_zero"):
                rospy.logwarn("Home a zero non riuscita. Proseguo comunque.")

        # 2) Pre-pre-approach: imposto i 7 valori dati, lasciando invariati eventuali giunti extra
        current = dict(zip(joint_names, self.group.get_current_joint_values()))
        desired = current.copy()
        # Prova a mappare per nome iiwa_joint_1..7
        for i in range(7):
            name = f"iiwa_joint_{i+1}"
            if name in desired:
                desired[name] = self.pre_joints[i]
        rospy.loginfo("Vado in pre-pre-approach (joint set specificati).")
        self.go_joint_dict(desired, "pre_pre_approach")

    # ----------------- Callback click P0 -----------------
    def clicked_cb(self, msg):
        # Porta il punto nel frame di riferimento (self.ref_frame), se necessario
        if msg.header.frame_id and msg.header.frame_id != self.ref_frame:
            try:
                tfm = self.tf_buffer.lookup_transform(self.ref_frame, msg.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
                p = tf2_ros.do_transform_point(msg, tfm)
                x, y, z = p.point.x, p.point.y, p.point.z
            except Exception as e:
                rospy.logwarn("TF fallita %s -> %s: %s", msg.header.frame_id, self.ref_frame, str(e))
                return
        else:
            x, y, z = msg.point.x, msg.point.y, msg.point.z

        # Orientazione: mantengo quella corrente della sonda (pre-pre-approach dovrebbe già "guardare" il paziente)
        cur = self.group.get_current_pose(self.ee_link)
        qx, qy, qz, qw = cur.pose.orientation.x, cur.pose.orientation.y, cur.pose.orientation.z, cur.pose.orientation.w

        # Se la punta fisica è a +Z_tool rispetto all'origine ee_link, arretra di tip_to_contact
        # Ricavo Z_tool dal quaternione attuale
        import tf.transformations as tft
        R = tft.quaternion_matrix([qx, qy, qz, qw])[:3, :3]
        z_tool = R[:, 2]  # +Z del tool

        px = x - self.tip_to_contact * z_tool[0]
        py = y - self.tip_to_contact * z_tool[1]
        pz = z - self.tip_to_contact * z_tool[2]

        target = PoseStamped()
        target.header.frame_id = self.ref_frame
        target.pose = Pose()
        target.pose.position.x = px
        target.pose.position.y = py
        target.pose.position.z = pz
        target.pose.orientation.x = qx
        target.pose.orientation.y = qy
        target.pose.orientation.z = qz
        target.pose.orientation.w = qw

        rospy.loginfo("Vado a toccare P0: (%.3f, %.3f, %.3f) mantenendo orientazione corrente.", x, y, z)
        self.group.set_start_state_to_current_state()
        self.group.set_pose_target(target)
        ok = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
        if not ok:
            rospy.logerr("Pianificazione verso P0 fallita. (Prova ad aumentare planning_time o a spostare leggermente P0)")

if __name__ == "__main__":
    try:
        node = PreTouchP0()
    except rospy.ROSInterruptException:
        pass
    finally:
        roscpp_shutdown()
