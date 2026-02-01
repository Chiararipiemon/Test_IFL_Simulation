#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# fixedZ version — keep probe Z_tool aligned to a fixed axis in ref_frame
# ci sono un po' di miei deliri da togliere nei commenti 
author_doc = """
Questo nodo ROS estende la pipeline touch→sweep→retract ma con orientazione
DELIBERATAMENTE FISSA del probe perchè mi sembra pù imile alla realtà: Z_tool viene allineato ad un vettore fisso nel
~ref_frame (default [0,0,-1], cioè "verso il basso" del mondo), ignorando le
normali della point cloud che per ora non considero

Fasi:
1) Move-to pre-approach (joints).
2) Move-to target (joints).
3) Touch su P0 con Z_tool fisso (con flip opzionale per "guardare" verso P0).
4) Sweep lineare sulla superficie mantenendo la stessa orientazione fissa.
5) Retract (allontanamento lungo +normale al paziente = −Z_tool) e rientro pre.

rispetto alla versione precedente in cui ci sono tutti i commenti principali, i nuovi parametri sono:
- ~use_fixed_z (bool, default True): usa orientazione fissa
- ~fixed_z_dir ([x,y,z], default [0,0,-1]): Z_tool desiderato nel ref_frame
- ~fixed_flip_to_face_target (bool, default True): se Z_tool non "guarda"
  verso il target (P0) lo inverte una sola volta per il touch/sweep
- Il resto (sweep_length, pdes_hint, ecc.) invariato

Nota: se ~fixed_z_dir=[0,0,-1], Z_tool punta verso il paziente; il retract
si muove lungo −Z_tool (cioè verso l’alto) per staccarsi dalla superficie del paziente
"""

import sys
import numpy as np
import rospy
import tf2_ros
import tf2_geometry_msgs  # noqa: F401
import tf.transformations as tft

from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

from moveit_commander import (
    roscpp_initialize,
    roscpp_shutdown,
    MoveGroupCommander,
    RobotCommander
)
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# --- opzionale KD-Tree ---
try:
    from scipy.spatial import cKDTree
except Exception:
    cKDTree = None


# ---------------- utils ----------------
def norm(v, eps=1e-12):
    v = np.asarray(v, float)
    n = np.linalg.norm(v)
    return v / n if n > eps else np.zeros_like(v)


def quat_from_axes(x, y, z):
    R = np.column_stack((norm(x), norm(y), norm(z)))
    T = np.eye(4)
    T[:3, :3] = R
    qx, qy, qz, qw = tft.quaternion_from_matrix(T)
    return np.array([qx, qy, qz, qw], float)


def quat_from_z_min_yaw(q_prev, z_des):
    q_prev = np.asarray(q_prev, float)
    Rprev = tft.quaternion_matrix(q_prev)[:3, :3]
    x_prev = Rprev[:, 0]
    z = norm(z_des)
    x_proj = x_prev - np.dot(x_prev, z) * z
    if np.linalg.norm(x_proj) < 1e-6:
        up = np.array([0, 0, 1.0]); x_proj = up - np.dot(up, z) * z
        if np.linalg.norm(x_proj) < 1e-6:
            up = np.array([1, 0, 0.0]); x_proj = up - np.dot(up, z) * z
    x = norm(x_proj)
    y = norm(np.cross(z, x))
    x = norm(np.cross(y, z))
    return quat_from_axes(x, y, z)


def slerp(q0, q1, s):
    return np.array(tft.quaternion_slerp(q0, q1, s), float)


def pose_from(p, q):
    ps = Pose()
    ps.position.x, ps.position.y, ps.position.z = float(p[0]), float(p[1]), float(p[2])
    ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w = q.tolist()
    return ps


def to_dict(names, vals):
    return {n: float(v) for n, v in zip(names, vals)}


# ---------------- main node ----------------
class PreToPoseAndTouchFixedZ(object):
    def __init__(self):
        roscpp_initialize(sys.argv)
        rospy.init_node("pre_to_pose_and_sweep_fixedZ")

        # MoveIt / group configuration
        self.group_name = rospy.get_param("~group_name", "manipulator")
        self.ee_link = rospy.get_param("~ee_link", "iiwa_link_ee")
        self.ref_frame = rospy.get_param("~ref_frame", "world")
        self.speed_scale = float(rospy.get_param("~speed_scale", 0.20))

        # Step 1–2 joints
        self.pre_joints = rospy.get_param("~pre_joints",
                                          [-2.529, 0.271, -0.268, 1.141, 2.932, 1.581, 0.174])
        self.target_joints = rospy.get_param("~target_joints",
                                             [-0.176, 0.675, 0.008, -0.789, -0.004, 1.669, -0.169])
        self.fallback_steps_pre = int(rospy.get_param("~fallback_steps_pre", 30))
        self.fallback_steps_tgt = int(rospy.get_param("~fallback_steps_tgt", 40))
        self.fallback_dt = float(rospy.get_param("~fallback_dt", 0.20))

        # Touch params
        self.cloud_topic = rospy.get_param("~cloud_topic", "/cloud_with_normals")
        self.contact_margin = float(rospy.get_param("~contact_margin", 0.006))
        self.approach_dist = float(rospy.get_param("~approach_dist", 0.06))
        self.retreat_dist = float(rospy.get_param("~retreat_dist", 0.18))

        self.far_steps = int(rospy.get_param("~far_steps", 25))
        self.pre_steps = int(rospy.get_param("~pre_steps", 18))
        self.approach_steps = int(rospy.get_param("~approach_steps", 22))
        self.step_time = float(rospy.get_param("~step_time", 0.25))

        self.ik_timeout = float(rospy.get_param("~ik_timeout", 1.2))
        self.ik_service_param = rospy.get_param("~ik_service", "")
        self.allow_partial = bool(rospy.get_param("~allow_partial", True))
        self.min_partial_fraction = float(rospy.get_param("~min_partial_fraction", 0.20))
        self.pos_tol_final = float(rospy.get_param("~pos_tol_final", 0.02))

        # Tip frames
        self.tip_frame = rospy.get_param("~tip_frame", "probe_tip")
        self.tip_to_contact = float(rospy.get_param("~tip_to_contact", 0.0))
        self.tip_vec_ee = None

        # Sweep params
        self.sweep_length = float(rospy.get_param("~sweep_length", 0.12))
        self.sweep_samples = int(rospy.get_param("~sweep_samples", 50))
        self.sweep_step_time = float(rospy.get_param("~sweep_step_time", 0.20))
        self.sweep_contact_margin = float(rospy.get_param("~sweep_contact_margin", self.contact_margin))
        self.sweep_pref_dir = np.asarray(rospy.get_param("~sweep_pref_dir", [1.0, 0.0, 0.0]), float)
        pdes_hint_param = rospy.get_param("~pdes_hint", [])
        self.pdes_hint = (np.asarray(pdes_hint_param, float)
                          if isinstance(pdes_hint_param, (list, tuple)) and len(pdes_hint_param) == 3
                          else None)

        # **NUOVI**: orientazione fissa
        self.use_fixed_z = bool(rospy.get_param("~use_fixed_z", True))
        self.fixed_z_dir = np.asarray(rospy.get_param("~fixed_z_dir", [0.0, 0.0, -1.0]), float)
        self.fixed_flip_to_face_target = bool(rospy.get_param("~fixed_flip_to_face_target", True))

        # MoveIt setup
        ns = rospy.get_namespace()
        robot_description = (ns if ns != "/" else "") + "robot_description"
        self.robot = RobotCommander(robot_description=robot_description, ns=ns)
        self.group = MoveGroupCommander(self.group_name, robot_description=robot_description, ns=ns)
        self.group.set_end_effector_link(self.ee_link)
        self.group.set_pose_reference_frame(self.ref_frame)
        self.group.set_max_velocity_scaling_factor(self.speed_scale)
        self.group.set_max_acceleration_scaling_factor(self.speed_scale)
        self.joint_names = self.group.get_active_joints()

        # TF / cloud
        self.tf_buf = tf2_ros.Buffer(rospy.Duration(60.0))
        self.tf_lis = tf2_ros.TransformListener(self.tf_buf)
        self.points = None
        self.normals = None
        self.kdt = None
        rospy.Subscriber(self.cloud_topic, PointCloud2, self.cloud_cb, queue_size=1)

        # IK service per la cinematica inversa 
        self.ik_srv = None
        self.ik_name = None

        # tip vector
        self.compute_tip_vector()

        # run
        self.step_to_joints(self.pre_joints, self.fallback_steps_pre, "pre_approach")
        self.step_to_joints(self.target_joints, self.fallback_steps_tgt, "target_pose")
        self.touch_and_sweep_fixedZ()
        rospy.signal_shutdown("done")

    # ---------- cloud ----------
    def cloud_cb(self, msg):
        fields = [f.name for f in msg.fields]
        has_norm = ("normal_x" in fields or "nx" in fields) and \
                   ("normal_y" in fields or "ny" in fields) and \
                   ("normal_z" in fields or "nz" in fields)
        nx = "normal_x" if "normal_x" in fields else ("nx" if "nx" in fields else None)
        ny = "normal_y" if "normal_y" in fields else ("ny" if "ny" in fields else None)
        nz = "normal_z" if "normal_z" in fields else ("nz" if "nz" in fields else None)

        pts = []
        nors = [] if has_norm else None
        if has_norm:
            for p in pc2.read_points(msg, field_names=("x", "y", "z", nx, ny, nz), skip_nans=True):
                pts.append([p[0], p[1], p[2]])
                nors.append([p[3], p[4], p[5]])
        else:
            for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                pts.append([p[0], p[1], p[2]])

        self.points = np.asarray(pts, float) if len(pts) else None
        if has_norm:
            self.normals = np.asarray(nors, float)
            nrm = np.linalg.norm(self.normals, axis=1)
            nrm[nrm == 0] = 1.0
            self.normals = self.normals / nrm[:, None]
        else:
            self.normals = None

        if self.points is not None and len(self.points) > 0 and cKDTree is not None:
            try:
                self.kdt = cKDTree(self.points)
            except Exception:
                self.kdt = None

    # ---------- tip vector ----------
    def compute_tip_vector(self):
        try:
            tfm = self.tf_buf.lookup_transform(self.ee_link, self.tip_frame,
                                               rospy.Time(0), rospy.Duration(2.0))
            tx = tfm.transform.translation.x
            ty = tfm.transform.translation.y
            tz = tfm.transform.translation.z
            self.tip_vec_ee = np.array([tx, ty, tz], float)
            rospy.loginfo("tip_frame=%s -> vettore EE->tip = [%.3f, %.3f, %.3f] (frame %s).",
                          self.tip_frame, tx, ty, tz, self.ee_link)
        except Exception as e:
            self.tip_vec_ee = None
            rospy.logwarn("lookup_transform(%s->%s) fallita: %s. Fallback tip_to_contact=%.3f.",
                          self.ee_link, self.tip_frame, str(e), self.tip_to_contact)

    # ---------- joint helpers ----------
    def exec_joint_traj(self, joints_seq, dt=None):
        if not joints_seq or len(joints_seq) < 2:
            return False
        jt = JointTrajectory(); jt.joint_names = list(self.joint_names)
        t = 0.0; step = self.fallback_dt if dt is None else dt
        for jd in joints_seq:
            pt = JointTrajectoryPoint()
            pt.positions = [jd[n] for n in jt.joint_names]
            pt.velocities = [0.0] * len(jt.joint_names)
            pt.accelerations = [0.0] * len(jt.joint_names)
            t += max(step, 0.05)
            pt.time_from_start = rospy.Duration(t)
            jt.points.append(pt)
        traj = RobotTrajectory(); traj.joint_trajectory = jt
        ok = self.group.execute(traj, wait=True); self.group.stop()
        return bool(ok)

    def step_to_joints(self, joint_vals, steps, tag):
        jd = {}
        for i in range(7):
            name = f"iiwa_joint_{i+1}"
            if name in self.joint_names:
                jd[name] = float(joint_vals[i])
        self.group.set_start_state_to_current_state()
        self.group.set_joint_value_target(jd)
        ok = self.group.go(wait=True); self.group.stop(); self.group.clear_pose_targets()
        if ok:
            rospy.loginfo("Raggiunto %s via planner.", tag); return True
        cur = np.array(self.group.get_current_joint_values(), float)
        tgt = np.array([jd[n] for n in self.joint_names], float)
        seq = []
        for s in np.linspace(0.0, 1.0, max(2, int(steps))):
            pos = (1.0 - s) * cur + s * tgt
            seq.append({n: float(v) for n, v in zip(self.joint_names, pos)})
        rospy.logwarn("Planner fallito per %s. Fallback diretto (%d step).", tag, len(seq))
        return self.exec_joint_traj(seq, self.fallback_dt)

    # ---------- IK service ----------
    def ensure_ik(self, timeout=3.0):
        if self.ik_srv is not None:
            return True
        candidates = []
        if self.ik_service_param:
            candidates.append(self.ik_service_param)
        ns = rospy.get_namespace()
        if ns != "/":
            candidates.append(ns + "compute_ik")
        candidates.append("/compute_ik")
        for name in candidates:
            try:
                rospy.wait_for_service(name, timeout=timeout)
                self.ik_srv = rospy.ServiceProxy(name, GetPositionIK)
                self.ik_name = name
                rospy.loginfo("Uso servizio IK: %s", name)
                return True
            except Exception:
                pass
        rospy.logerr("Nessun servizio IK disponibile: %s", ", ".join(candidates))
        return False

    def ik_solve(self, pose_stamped, avoid=True):
        if not self.ensure_ik(3.0):
            return None
        req = GetPositionIKRequest()
        req.ik_request.group_name = self.group_name
        req.ik_request.ik_link_name = self.ee_link
        req.ik_request.pose_stamped = pose_stamped
        req.ik_request.timeout = rospy.Duration(self.ik_timeout)
        req.ik_request.avoid_collisions = bool(avoid)
        req.ik_request.robot_state = self.robot.get_current_state()
        try:
            resp = self.ik_srv(req)
        except rospy.ServiceException as e:
            rospy.logwarn("IK service failed: %s", str(e)); return None
        if not hasattr(resp, "error_code") or resp.error_code.val != 1:
            return None
        js = resp.solution.joint_state
        name_to_pos = {n: p for n, p in zip(js.name, js.position)}
        target = {n: name_to_pos[n] for n in self.joint_names if n in name_to_pos}
        return target if len(target) == len(self.joint_names) else None

    # ---------- helpers: nearest / normali (solo per sampling traccia) ----------
    def nearest_index(self, p):
        if self.points is None or len(self.points) == 0:
            return None
        p = np.asarray(p, float)
        if self.kdt is not None:
            _, idx = self.kdt.query(p); return int(idx)
        d2 = np.sum((self.points - p[None, :]) ** 2, axis=1)
        return int(np.argmin(d2))

    def normal_at(self, idx_or_point, k=25):
        # Ancora disponibile se vuoi stimare normali per diagnostica, ma NON
        # viene usata per l'orientazione in questa versione fixed-Z.
        if self.points is None or len(self.points) == 0:
            return np.array([0, 0, 1.0], float)
        if isinstance(idx_or_point, int):
            if self.normals is not None and idx_or_point < len(self.normals):
                return norm(self.normals[idx_or_point])
            p = self.points[idx_or_point]
        else:
            p = np.asarray(idx_or_point, float)
        if self.kdt is not None:
            k_ = min(k, len(self.points))
            _, idxs = self.kdt.query(p, k=k_)
            neigh = self.points[idxs if k_ > 1 else [idxs]]
        else:
            d2 = np.sum((self.points - p[None, :]) ** 2, axis=1)
            idxs = np.argsort(d2)[:min(k, len(self.points))]
            neigh = self.points[idxs]
        c = neigh.mean(axis=0); A = neigh - c
        try:
            _, _, vh = np.linalg.svd(A, full_matrices=False)
            n = vh[-1, :]
        except Exception:
            n = np.array([0, 0, 1.0], float)
        return norm(n)

    # ---------- orientation helpers (FIXED Z) ----------
    def z_fixed_facing_target(self, q_ref, tip_pos_now, target_point):
        """
        Restituisce la Z fissa (eventualmente flippata) e il quaternion desiderato
        che mantiene lo yaw vicino a q_ref.
        """
        zfix = norm(self.fixed_z_dir)
        if self.fixed_flip_to_face_target:
            to_tgt = norm(np.asarray(target_point, float) - np.asarray(tip_pos_now, float))
            if np.dot(zfix, to_tgt) < 0.0:
                zfix = -zfix
        q_des = quat_from_z_min_yaw(q_ref, zfix)
        return zfix, q_des

    # ---------- touch waypoints (usa Z fissa) ----------
    def make_waypoints_fixedZ(self, p_now, q_now, P0):
        R_now = tft.quaternion_matrix(q_now)[:3, :3]
        r_tip = self.tip_vec_ee if self.tip_vec_ee is not None else \
            np.array([0.0, 0.0, float(self.tip_to_contact)], float)
        p_tip_now = p_now + R_now.dot(r_tip)

        z_vec, q_des = self.z_fixed_facing_target(q_now, p_tip_now, P0)
        R_des = tft.quaternion_matrix(q_des)[:3, :3]

        tip_final = P0 - self.contact_margin * z_vec
        ee_final = tip_final - R_des.dot(r_tip)

        tip_pre = P0 - (self.contact_margin + self.approach_dist) * z_vec
        tip_far = P0 - (self.contact_margin + self.approach_dist + self.retreat_dist) * z_vec
        ee_pre = tip_pre - R_des.dot(r_tip)
        ee_far = tip_far - R_des.dot(r_tip)

        way_far = [pose_from((1.0 - s) * p_now + s * ee_far, slerp(q_now, q_des, s))
                   for s in np.linspace(0.0, 1.0, max(2, self.far_steps))]
        way_pre = [pose_from((1.0 - s) * ee_far + s * ee_pre, q_des)
                   for s in np.linspace(0.0, 1.0, max(2, self.pre_steps))]
        way_app = [pose_from((1.0 - s) * ee_pre + s * ee_final, q_des)
                   for s in np.linspace(0.0, 1.0, max(2, self.approach_steps))]

        return way_far + way_pre + way_app, tip_final

    def ik_for_all(self, poses, frame_id):
        ps = PoseStamped(); ps.header.frame_id = frame_id
        joints_seq = []; solved = 0
        cur = self.group.get_current_joint_values()
        joints_seq.append(to_dict(self.joint_names, cur))
        for i, pose in enumerate(poses, 1):
            ps.header.stamp = rospy.Time.now(); ps.pose = pose
            tgt = self.ik_solve(ps, True) or self.ik_solve(ps, False)
            if tgt is None:
                frac = solved / float(len(poses))
                if self.allow_partial and frac >= self.min_partial_fraction:
                    rospy.logwarn("IK fallita al waypoint %d/%d: accetto parziale (frac=%.2f).",
                                  i, len(poses), frac)
                    return joints_seq, solved, True
                rospy.logerr("IK fallita al waypoint %d/%d e parziale non accettabile (frac=%.2f).",
                             i, len(poses), frac)
                return None, solved, False
            joints_seq.append(tgt); solved += 1
        return joints_seq, solved, False

    # ---------- selezione P0 ----------
    def pick_P0_ahead_of_tip(self):
        for _ in range(200):
            if self.points is not None and len(self.points) > 0:
                break
            rospy.sleep(0.05)
        if self.points is None or len(self.points) == 0:
            rospy.logerr("Cloud non disponibile su %s", self.cloud_topic)
            return None, None

        cur = self.group.get_current_pose(self.ee_link).pose
        q_now = np.array([cur.orientation.x, cur.orientation.y, cur.orientation.z, cur.orientation.w], float)
        R_now = tft.quaternion_matrix(q_now)[:3, :3]
        r_tip = self.tip_vec_ee if self.tip_vec_ee is not None else \
            np.array([0.0, 0.0, float(self.tip_to_contact)], float)
        p_tip = np.array([cur.position.x, cur.position.y, cur.position.z], float) + R_now.dot(r_tip)

        # Direzione "avanti" = -Z_tool corrente (come prima, per selezionare un punto davanti)
        z_now = R_now[:, 2]; dirn = -norm(z_now)
        r = self.points - p_tip[None, :]
        t = np.einsum('ij,j->i', r, dirn)
        idxs = np.where(t > 0.0)[0]
        if idxs.size == 0:
            idxs = np.arange(len(self.points)); t = np.maximum(t, 0.0)
        d2 = np.einsum('ij,ij->i', r, r) - t * t
        best = idxs[np.argmin(d2[idxs])] if idxs.size > 0 else int(np.argmin(d2))
        P0 = self.points[best]
        n0 = self.normals[best] if (self.normals is not None and best < len(self.normals)) else None
        rospy.loginfo("P0 scelto: [%.3f, %.3f, %.3f] (idx %d).", P0[0], P0[1], P0[2], best)
        return (P0, n0)

    # ---------- sweep planning (stessa logica di traiettoria spaziale) ----------
    def tangent_from_normal(self, n, pref=None):
        prefv = np.array(pref if pref is not None else [1.0, 0.0, 0.0], float)
        n = norm(n)
        t = prefv - np.dot(prefv, n) * n
        if np.linalg.norm(t) < 1e-6:
            pref2 = np.array([0.0, 1.0, 0.0], float)
            t = pref2 - np.dot(pref2, n) * n
            if np.linalg.norm(t) < 1e-6:
                pref2 = np.array([0.0, 0.0, 1.0], float)
                t = pref2 - np.dot(pref2, n) * n
        return norm(t)

    def plan_linear_surface_sweep(self, P0, n0):
        P0 = np.asarray(P0, float)
        if self.pdes_hint is not None:
            Pdes_wish = np.asarray(self.pdes_hint, float)
        else:
            # Se non ci sono normali, stimiamo solo per costruire una tangente (non per orientazione)
            n = n0 if n0 is not None else self.normal_at(self.nearest_index(P0))
            tdir = self.tangent_from_normal(n, self.sweep_pref_dir)
            Pdes_wish = P0 + self.sweep_length * tdir

        idx_end = self.nearest_index(Pdes_wish)
        if idx_end is None:
            rospy.logerr("Impossibile trovare P_des (cloud vuota).")
            return None, None, None
        Pdes_surf = self.points[idx_end]

        pts = []; nors = []
        nS = max(2, int(self.sweep_samples)); last_added = None
        for s in np.linspace(0.0, 1.0, nS):
            Pw = (1.0 - s) * P0 + s * Pdes_wish
            idx = self.nearest_index(Pw)
            if idx is None:
                continue
            Pi = self.points[idx]
            if last_added is None or np.linalg.norm(Pi - last_added) > 1e-4:
                ni = self.normal_at(idx)
                pts.append(Pi); nors.append(ni); last_added = Pi

        if len(pts) < 2:
            rospy.logerr("Traccia sweep troppo corta o cloud insufficiente.")
            return None, None, None
        return np.asarray(pts, float), np.asarray(nors, float), Pdes_surf

    def poses_from_surface_track_fixedZ(self, q_start, track_pts):
        """
        Converte la traccia in pose con Z_tool FISSO.
        Usa un unico quaternion q_fixed (con eventuale flip rispetto al primo punto).
        """
        # ricava tip attuale
        cur = self.group.get_current_pose(self.ee_link).pose
        p_now = np.array([cur.position.x, cur.position.y, cur.position.z], float)
        q_now = np.array([cur.orientation.x, cur.orientation.y, cur.orientation.z, cur.orientation.w], float)
        R_now = tft.quaternion_matrix(q_now)[:3, :3]
        r_tip = self.tip_vec_ee if self.tip_vec_ee is not None else \
            np.array([0.0, 0.0, float(self.tip_to_contact)], float)
        p_tip_now = p_now + R_now.dot(r_tip)

        # fissa Z con eventuale flip verso il primo punto della traccia
        z_vec, q_fixed = self.z_fixed_facing_target(q_start, p_tip_now, track_pts[0])
        R_fixed = tft.quaternion_matrix(q_fixed)[:3, :3]

        poses = []
        for Pi in track_pts:
            tip_target = Pi - self.sweep_contact_margin * z_vec
            ee_target = tip_target - R_fixed.dot(r_tip)
            poses.append(pose_from(ee_target, q_fixed))
        return poses, z_vec, q_fixed

    def retract_from_surface(self, current_q, current_tip, lift_dist=None, steps=12):
        """
        Retract lungo +normale al paziente, cioè −Z_tool (dato che Z_tool punta verso il paziente).
        """
        if lift_dist is None:
            lift_dist = self.approach_dist + self.retreat_dist
        q = np.asarray(current_q, float)
        R = tft.quaternion_matrix(q)[:3, :3]
        z = R[:, 2]          # Z_tool
        r_tip = self.tip_vec_ee if self.tip_vec_ee is not None else \
            np.array([0.0, 0.0, float(self.tip_to_contact)], float)

        tip_up = current_tip - lift_dist * z   # **IMPORTANTE**: verso opposto a Z_tool
        ee_up = tip_up - R.dot(r_tip)

        ee_now_pose = self.group.get_current_pose(self.ee_link).pose
        p_now = np.array([ee_now_pose.position.x, ee_now_pose.position.y, ee_now_pose.position.z], float)
        poses = [pose_from((1.0 - s) * p_now + s * ee_up, q)
                 for s in np.linspace(0.0, 1.0, max(2, steps))]
        return poses

    # ---------- pipeline completa ----------
    def touch_and_sweep_fixedZ(self):
        P0, n0 = self.pick_P0_ahead_of_tip()
        if P0 is None:
            rospy.logerr("Impossibile selezionare P0 dalla cloud."); return

        # --- Touch con Z fissa ---
        cur = self.group.get_current_pose(self.ee_link).pose
        p_now = np.array([cur.position.x, cur.position.y, cur.position.z], float)
        q_now = np.array([cur.orientation.x, cur.orientation.y, cur.orientation.z, cur.orientation.w], float)
        poses_touch, tip_final = self.make_waypoints_fixedZ(p_now, q_now, P0)

        joints_seq, solved, partial = self.ik_for_all(poses_touch, self.ref_frame)
        if joints_seq is None:
            rospy.logerr("IK insufficiente per la fase touch."); return
        rospy.loginfo("Eseguo traiettoria touch con %d/%d punti.", solved, len(poses_touch))
        if not self.exec_joint_traj(joints_seq, self.step_time):
            rospy.logerr("Esecuzione touch fallita."); return

        # orientazione dopo il touch (sarà vicina a q_fixed)
        cur = self.group.get_current_pose(self.ee_link).pose
        q_touch = np.array([cur.orientation.x, cur.orientation.y, cur.orientation.z, cur.orientation.w], float)

        # --- Sweep (orientazione fissa) ---
        track_pts, track_norms, Pdes_surf = self.plan_linear_surface_sweep(P0, n0)
        if track_pts is None:
            rospy.logerr("Pianificazione sweep fallita."); return

        poses_sweep, z_vec, q_fixed = self.poses_from_surface_track_fixedZ(q_touch, track_pts)
        joints_seq2, solved2, partial2 = self.ik_for_all(poses_sweep, self.ref_frame)
        if joints_seq2 is None:
            rospy.logerr("IK insufficiente per sweep (risolti %d/%d).", solved2, len(poses_sweep)); return
        rospy.loginfo("Eseguo sweep con %d/%d punti (Z_tool fisso).", solved2, len(poses_sweep))
        if not self.exec_joint_traj(joints_seq2, self.sweep_step_time):
            rospy.logerr("Esecuzione sweep fallita."); return

        # --- Retract dal punto finale ---
        last_pose = self.group.get_current_pose(self.ee_link).pose
        q_last = np.array([last_pose.orientation.x, last_pose.orientation.y,
                           last_pose.orientation.z, last_pose.orientation.w], float)
        R_last = tft.quaternion_matrix(q_last)[:3, :3]
        z_last = R_last[:, 2]
        tip_last = track_pts[-1] - self.sweep_contact_margin * z_last

        poses_up = self.retract_from_surface(q_last, tip_last,
                                             lift_dist=self.retreat_dist + self.approach_dist,
                                             steps=14)
        joints_seq3, solved3, partial3 = self.ik_for_all(poses_up, self.ref_frame)
        if joints_seq3 is not None:
            rospy.loginfo("Eseguo retract con %d/%d punti.", solved3, len(poses_up))
            self.exec_joint_traj(joints_seq3, self.step_time)
        else:
            rospy.logwarn("Retract non pianificabile, salto.")

        # --- ritorno pre-approach ---
        self.step_to_joints(self.pre_joints, self.fallback_steps_pre, "pre_approach")
        rospy.loginfo("Sequenza fixedZ completata (touch → sweep → retract → pre-approach).")


if __name__ == "__main__":
    try:
        PreToPoseAndTouchFixedZ()
    except rospy.ROSInterruptException:
        pass
    finally:
        roscpp_shutdown()
