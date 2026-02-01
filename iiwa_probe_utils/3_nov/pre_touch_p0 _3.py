#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import numpy as np

import rospy
import tf2_ros
import tf2_geometry_msgs
import tf.transformations as tft

from geometry_msgs.msg import PointStamped, PoseStamped, Pose
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

from moveit_commander import (
    roscpp_initialize, roscpp_shutdown, MoveGroupCommander
)

def norm(v, eps=1e-12):
    n = np.linalg.norm(v)
    return v / n if n > eps else np.zeros_like(v)

def quat_from_z(z_axis, x_hint=None):
    """Restituisce un quaternione (x,y,z,w) con Z_tool allineato a z_axis."""
    z = norm(np.asarray(z_axis, dtype=float))
    up = np.array([0.0, 0.0, 1.0]) if x_hint is None else np.asarray(x_hint, float)
    x = np.cross(up, z)
    if np.linalg.norm(x) < 1e-6:
        up = np.array([1.0, 0.0, 0.0])
        x = np.cross(up, z)
    x = norm(x)
    y = norm(np.cross(z, x))
    x = norm(np.cross(y, z))
    R = np.column_stack((x, y, z))
    T = np.eye(4); T[:3, :3] = R
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
        self.cloud_topic        = rospy.get_param("~cloud_topic", "/cloud_with_normals")
        self.speed_scale        = float(rospy.get_param("~speed_scale", 0.2))
        self.planning_time      = float(rospy.get_param("~planning_time", 30.0))
        self.planning_attempts  = int(rospy.get_param("~planning_attempts", 30))
        self.tip_to_contact     = float(rospy.get_param("~tip_to_contact", 0.00))   # m
        self.approach_dist      = float(rospy.get_param("~approach_dist", 0.08))    # m
        self.contact_margin     = float(rospy.get_param("~contact_margin", 0.005))  # m
        self.eef_step           = float(rospy.get_param("~eef_step", 0.003))
        self.do_home_zero       = bool(rospy.get_param("~do_home_zero", True))
        self.auto_pick_p0       = bool(rospy.get_param("~auto_pick_p0", True))

        # Config pre-pre-approach (rad)
        self.pre_joints = [-2.529, 0.271, -0.268, 1.141, 2.932, 1.581, 0.174]

        # MoveIt
        ns = rospy.get_namespace()
        robot_description = (ns if ns != "/" else "") + "robot_description"
        self.group = MoveGroupCommander(self.group_name, robot_description=robot_description, ns=ns)
        self.group.set_end_effector_link(self.ee_link)
        self.group.set_pose_reference_frame(self.ref_frame)
        self.group.set_max_velocity_scaling_factor(self.speed_scale)
        self.group.set_max_acceleration_scaling_factor(self.speed_scale)
        self.group.set_planning_time(self.planning_time)
        self.group.set_num_planning_attempts(self.planning_attempts)
        self.group.allow_replanning(True)
        # Tolleranze un po' più permissive per facilitare l'IK
        self.group.set_goal_position_tolerance(0.002)       # 2 mm
        self.group.set_goal_orientation_tolerance(0.20)     # ~11.5°

        # TF
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(60.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Cloud
        self.points = None   # Nx3
        self.normals = None  # Nx3 (opzionale)
        rospy.Subscriber(self.cloud_topic, PointCloud2, self.cloud_cb, queue_size=1)

        # Sequenza iniziale: home -> pre-pre-approach
        self.move_home_and_pre()

        # Flusso: auto pick P0 oppure attesa click
        if self.auto_pick_p0:
            rospy.loginfo("auto_pick_p0 = True: scelgo P0 dalla cloud e vado in pre-approach + tocco.")
            self.pick_from_cloud_and_touch()
        else:
            rospy.Subscriber(self.clicked_topic, PointStamped, self.clicked_cb, queue_size=10)
            rospy.loginfo("Pronto: clicca un P0 (Publish Point).")

        rospy.spin()

    # --------- Cloud callback ----------
    def cloud_cb(self, msg):
        fields = [f.name for f in msg.fields]
        has_normals = ("normal_x" in fields or "nx" in fields) and \
                      ("normal_y" in fields or "ny" in fields) and \
                      ("normal_z" in fields or "nz" in fields)
        nx = "normal_x" if "normal_x" in fields else ("nx" if "nx" in fields else None)
        ny = "normal_y" if "normal_y" in fields else ("ny" if "ny" in fields else None)
        nz = "normal_z" if "normal_z" in fields else ("nz" if "nz" in fields else None)

        pts = []
        nors = [] if has_normals else None

        if has_normals:
            for p in pc2.read_points(msg, field_names=("x","y","z",nx,ny,nz), skip_nans=True):
                pts.append([p[0], p[1], p[2]])
                nors.append([p[3], p[4], p[5]])
        else:
            for p in pc2.read_points(msg, field_names=("x","y","z"), skip_nans=True):
                pts.append([p[0], p[1], p[2]])

        self.points = np.asarray(pts, dtype=float) if len(pts) else None
        if has_normals:
            self.normals = np.asarray(nors, dtype=float)
            nrm = np.linalg.norm(self.normals, axis=1); nrm[nrm==0]=1.0
            self.normals = self.normals / nrm[:,None]

    # --------- Movimenti base ----------
    def go_joint_dict(self, target_dict, msg=""):
        self.group.set_start_state_to_current_state()
        self.group.set_joint_value_target(target_dict)
        ok = self.group.go(wait=True)
        self.group.stop(); self.group.clear_pose_targets()
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

        # Pre-pre-approach (imposta i 7 valori noti lasciando invariati eventuali giunti extra)
        current = dict(zip(joint_names, self.group.get_current_joint_values()))
        desired = current.copy()
        for i in range(7):
            name = f"iiwa_joint_{i+1}"
            if name in desired:
                desired[name] = self.pre_joints[i]
        rospy.loginfo("Vado in pre-pre-approach (joint set specificati).")
        self.go_joint_dict(desired, "pre_pre_approach")

    # --------- Scelta automatica P0 dalla cloud ----------
    def pick_from_cloud_and_touch(self):
        # Attende la cloud
        for _ in range(100):
            if self.points is not None and len(self.points) > 0:
                break
            rospy.sleep(0.05)
        if self.points is None or len(self.points) == 0:
            rospy.logerr("Cloud non disponibile su %s.", self.cloud_topic); return

        cur = self.group.get_current_pose(self.ee_link)
        p_now = np.array([cur.pose.position.x, cur.pose.position.y, cur.pose.position.z], dtype=float)
        q_now = [cur.pose.orientation.x, cur.pose.orientation.y, cur.pose.orientation.z, cur.pose.orientation.w]
        z_now = tft.quaternion_matrix(q_now)[:3,2]
        z_now = norm(z_now if np.linalg.norm(z_now)>0 else np.array([0,0,1.0]))

        # Trova il punto della cloud più vicino al raggio p_now + t*z_now, t>=0
        r = self.points - p_now[None,:]
        t = np.einsum('ij,j->i', r, z_now)
        mask = t > 0.0
        cand_idx = np.where(mask)[0]
        if cand_idx.size == 0:
            cand_idx = np.arange(len(self.points))
            t = np.maximum(t, 0.0)
        d2 = np.einsum('ij,ij->i', r, r) - t*t
        idx = cand_idx[np.argmin(d2[cand_idx])] if cand_idx.size>0 else int(np.argmin(d2))
        P0 = self.points[idx]
        n0 = self.normals[idx] if self.normals is not None and idx < len(self.normals) else None

        rospy.loginfo("P0 auto-selezionato: [%.3f, %.3f, %.3f] (idx %d).", P0[0], P0[1], P0[2], idx)

        # Orienta Z_tool: usa normale se disponibile, altrimenti punta a P0
        if n0 is not None:
            z_des = -norm(n0)  # Z_tool verso la superficie (opposto alla normale)
        else:
            z_des = norm(P0 - p_now) if np.linalg.norm(P0 - p_now)>1e-6 else z_now
        qx, qy, qz, qw = quat_from_z(z_des)

        # Posizioni: pre-approach e “quasi contatto” (fermati prima della superficie)
        z_vec = tft.quaternion_matrix([qx,qy,qz,qw])[:3,2]
        pre_pos   = P0 - (self.tip_to_contact + self.approach_dist) * z_vec
        final_pos = P0 - (self.tip_to_contact + self.contact_margin) * z_vec

        # 1) Pre-approach CARTESIAN (con fallback a go())
        pre_pose = Pose()
        pre_pose.position.x, pre_pose.position.y, pre_pose.position.z = pre_pos.tolist()
        pre_pose.orientation.x, pre_pose.orientation.y, pre_pose.orientation.z, pre_pose.orientation.w = (qx,qy,qz,qw)

        self.group.set_start_state_to_current_state()
        traj, frac = self.group.compute_cartesian_path([pre_pose], self.eef_step, True)  # avoid_collisions=True
        rospy.loginfo("Cartesian pre-approach fraction: %.1f%%", 100.0*frac)
        if getattr(traj, "joint_trajectory", None) and len(traj.joint_trajectory.points)>0 and frac>0.9:
            self.group.execute(traj, wait=True); self.group.stop()
        else:
            rospy.logwarn("Cartesian pre-approach parziale/assente. Provo con go().")
            pre_ps = PoseStamped(); pre_ps.header.frame_id = self.ref_frame; pre_ps.pose = pre_pose
            self.group.set_start_state_to_current_state()
            self.group.set_pose_target(pre_ps)
            ok = self.group.go(wait=True)
            self.group.stop(); self.group.clear_pose_targets()
            if not ok:
                rospy.logerr("Pre-approach fallito."); return

        # 2) Avvicinamento cartesiano fino a prima della superficie
        final_pose = Pose()
        final_pose.position.x, final_pose.position.y, final_pose.position.z = final_pos.tolist()
        final_pose.orientation.x, final_pose.orientation.y, final_pose.orientation.z, final_pose.orientation.w = (qx,qy,qw,qw) if False else (qx,qy,qz,qw)  # keep orientation

        self.group.set_start_state_to_current_state()
        traj2, frac2 = self.group.compute_cartesian_path([final_pose], self.eef_step, True)
        rospy.loginfo("Cartesian approach fraction: %.1f%%", 100.0*frac2)
        if getattr(traj2, "joint_trajectory", None) and len(traj2.joint_trajectory.points)>0:
            self.group.execute(traj2, wait=True); self.group.stop()
            rospy.loginfo("Arrivato vicino a P0 (margine %.3f m).", self.contact_margin)
        else:
            rospy.logerr("Traiettoria cartesiana vuota. Riduci approach_dist o eef_step.")

    # --------- Callback manuale su P0 (se auto_pick_p0=False) ----------
    def clicked_cb(self, msg):
        # Porta il punto nel ref_frame, se necessario
        if msg.header.frame_id and msg.header.frame_id != self.ref_frame:
            try:
                tfm = self.tf_buffer.lookup_transform(self.ref_frame, msg.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
                p = tf2_geometry_msgs.do_transform_point(msg, tfm)
                target = np.array([p.point.x, p.point.y, p.point.z], dtype=float)
            except Exception as e:
                rospy.logwarn("TF fallita %s -> %s: %s", msg.header.frame_id, self.ref_frame, str(e))
                return
        else:
            target = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=float)

        cur = self.group.get_current_pose(self.ee_link)
        p_now = np.array([cur.pose.position.x, cur.pose.position.y, cur.pose.position.z], dtype=float)
        z_des = norm(target - p_now)
        qx, qy, qz, qw = quat_from_z(z_des)

        z_vec = tft.quaternion_matrix([qx,qy,qz,qw])[:3,2]
        pre_pos   = target - (self.tip_to_contact + self.approach_dist) * z_vec
        final_pos = target - (self.tip_to_contact + self.contact_margin) * z_vec

        # Pre-approach cartesiano (fallback a go())
        pre_pose = Pose()
        pre_pose.position.x, pre_pose.position.y, pre_pose.position.z = pre_pos.tolist()
        pre_pose.orientation.x, pre_pose.orientation.y, pre_pose.orientation.z, pre_pose.orientation.w = (qx,qy,qz,qw)

        self.group.set_start_state_to_current_state()
        traj, frac = self.group.compute_cartesian_path([pre_pose], self.eef_step, True)
        rospy.loginfo("Cartesian pre-approach fraction: %.1f%%", 100.0*frac)
        if getattr(traj, "joint_trajectory", None) and len(traj.joint_trajectory.points)>0 and frac>0.9:
            self.group.execute(traj, wait=True); self.group.stop()
        else:
            pre_ps = PoseStamped(); pre_ps.header.frame_id = self.ref_frame; pre_ps.pose = pre_pose
            self.group.set_start_state_to_current_state()
            self.group.set_pose_target(pre_ps)
            ok = self.group.go(wait=True)
            self.group.stop(); self.group.clear_pose_targets()
            if not ok:
                rospy.logerr("Pre-approach fallito."); return

        # Avvicinamento cartesiano fino a prima della superficie
        final_pose = Pose()
        final_pose.position.x, final_pose.position.y, final_pose.position.z = final_pos.tolist()
        final_pose.orientation.x, final_pose.orientation.y, final_pose.orientation.z, final_pose.orientation.w = (qx,qy,qz,qw)

        self.group.set_start_state_to_current_state()
        traj2, frac2 = self.group.compute_cartesian_path([final_pose], self.eef_step, True)
        rospy.loginfo("Cartesian approach fraction: %.1f%%", 100.0*frac2)
        if getattr(traj2, "joint_trajectory", None) and len(traj2.joint_trajectory.points)>0:
            self.group.execute(traj2, wait=True); self.group.stop()
            rospy.loginfo("Arrivato vicino a P0 (margine %.3f m).", self.contact_margin)
        else:
            rospy.logerr("Traiettoria cartesiana vuota.")

if __name__ == "__main__":
    try:
        node = PreTouchP0()
    except rospy.ROSInterruptException:
        pass
    finally:
        roscpp_shutdown()

