#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import numpy as np

import rospy
from moveit_commander import (
    roscpp_initialize, roscpp_shutdown, MoveGroupCommander, RobotCommander
)
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def to_dict(names, vals):
    return {n: float(v) for n, v in zip(names, vals)}

class MovePreToPose(object):
    def __init__(self):
        roscpp_initialize(sys.argv)
        rospy.init_node("move_pre_to_pose")

        # Parametri base
        self.group_name  = rospy.get_param("~group_name", "manipulator")
        self.ee_link     = rospy.get_param("~ee_link", "iiwa_link_ee")
        self.speed_scale = float(rospy.get_param("~speed_scale", 0.20))

        # Joint set di default (puoi sovrascrivere via parametri ~pre_joints e ~target_joints)
        self.pre_joints_default    = [-2.529, 0.271, -0.268, 1.141, 2.932, 1.581, 0.174]
        self.target_joints_default = [-0.176, 0.675, 0.008, -0.789, -0.004, 1.669, -0.169]

        self.pre_joints    = rospy.get_param("~pre_joints",    self.pre_joints_default)
        self.target_joints = rospy.get_param("~target_joints", self.target_joints_default)

        # Fallback di traiettoria diretta
        self.fallback_steps = int(rospy.get_param("~fallback_steps", 40))
        self.fallback_dt    = float(rospy.get_param("~fallback_dt", 0.20))  # secondi tra i punti

        # MoveIt
        ns = rospy.get_namespace()
        robot_description = (ns if ns != "/" else "") + "robot_description"
        self.robot = RobotCommander(robot_description=robot_description, ns=ns)
        self.group = MoveGroupCommander(self.group_name, robot_description=robot_description, ns=ns)
        self.group.set_end_effector_link(self.ee_link)
        self.group.set_max_velocity_scaling_factor(self.speed_scale)
        self.group.set_max_acceleration_scaling_factor(self.speed_scale)
        self.joint_names = self.group.get_active_joints()

        # Esegui sequenza
        ok1 = self.go_or_fallback(self.pre_joints, tag="pre_approach")
        if not ok1:
            rospy.logerr("Non sono riuscita a raggiungere il pre-approach. Esco.")
            return
        ok2 = self.go_or_fallback(self.target_joints, tag="target_pose")
        if not ok2:
            rospy.logerr("Non sono riuscita a raggiungere la posa target.")
            return
        rospy.loginfo("Sequenza completata: pre-approach -> target.")

        rospy.signal_shutdown("done")

    # ---- helpers ----
    def go_or_fallback(self, joint_vals, tag=""):
        """
        Prova MoveIt go(); se fallisce, usa traiettoria diretta interpolata.
        joint_vals: lista di 7 valori per iiwa_joint_1..7
        """
        # mappa valori sui nomi dei joint presenti
        jd = {}
        for i in range(7):
            name = f"iiwa_joint_{i+1}"
            if name in self.joint_names:
                jd[name] = float(joint_vals[i])

        # tentativo via planner
        self.group.set_start_state_to_current_state()
        self.group.set_joint_value_target(jd)
        ok = self.group.go(wait=True)
        self.group.stop(); self.group.clear_pose_targets()

        if ok:
            rospy.loginfo("Raggiunto %s via planner.", tag)
            return True

        rospy.logwarn("Planner fallito per %s. Avvio fallback diretto (interpolazione).", tag)
        return self.exec_direct_interp(jd, tag)

    def exec_direct_interp(self, target_dict, tag="direct"):
        cur_vals = self.group.get_current_joint_values()
        cur = np.array(cur_vals, dtype=float)
        tgt = np.array([target_dict[n] for n in self.joint_names], dtype=float)

        jt = JointTrajectory()
        jt.joint_names = list(self.joint_names)

        t = 0.0
        steps = max(2, int(self.fallback_steps))
        for s in np.linspace(0.0, 1.0, steps):
            pt = JointTrajectoryPoint()
            pos = (1.0 - s) * cur + s * tgt
            pt.positions = pos.tolist()
            pt.velocities = [0.0]*len(self.joint_names)
            pt.accelerations = [0.0]*len(self.joint_names)
            t += max(self.fallback_dt, 0.05)
            pt.time_from_start = rospy.Duration(t)
            jt.points.append(pt)

        traj = RobotTrajectory()
        traj.joint_trajectory = jt
        ok = self.group.execute(traj, wait=True)
        self.group.stop()
        if ok:
            rospy.loginfo("Raggiunto %s con fallback diretto.", tag)
        else:
            rospy.logerr("Fallback diretto fallito su %s.", tag)
        return bool(ok)

if __name__ == "__main__":
    try:
        MovePreToPose()
    except rospy.ROSInterruptException:
        pass
    finally:
        roscpp_shutdown()
