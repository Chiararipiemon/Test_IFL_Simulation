#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 4_nov_sweep
# Code for touching a point P0 on a point-cloud surface using MoveIt and IK,
# then performing a linear surface sweep to P_des with normal alignment
# note personali e pensieri
author_doc = """
Questo nodo ROS cerca di far muovere il manipolatore in tre fasi principali + sweep lineare:

1) Move-to pre-approach (joints).
2) Move-to target (joints).
3) Seleziona un punto P0 davanti al tip dalla cloud, orienta Z_tool verso la normale locale e
   esegue far→pre→approach per un "touch" con margine di sicurezza, difatti mi sembra che i contatto su p0 è abbastanza soft e dolce, non lo so non sono un medico
4) Esegue una scansione lineare (sweep) sulla superficie mantenendo il probe normale ai punti
   della traccia e si ferma a P_des (derivato ~sweep_length che per ora scrivo direttamente qundo lancio il codice e per ora è settato a 0.20).
5) Si solleva (retract) da P_des e rientra alla pre-approach.

Parametri principali
- ~group_name: nome gruppo MoveIt (default "manipulator")
- ~ee_link: end-effector link (default "iiwa_link_ee")
- ~ref_frame: frame di riferimento pose (default "world")
- ~speed_scale: scala velocità/accelerazione MoveIt (default 0.20)
- ~pre_joints, ~target_joints: array di 7 joint per fasi 1–2
- ~fallback_steps_pre, ~fallback_steps_tgt: steps di interpolazione se il planner fallisce
- ~fallback_dt: tempo tra punti interpolati
- ~cloud_topic: topic PointCloud2 con eventuali normali (default "/cloud_with_normals")
- ~contact_margin: offset di sicurezza prima del contatto lungo Z_tool (default 0.006 m), da valtare se toglierlo 
- ~approach_dist, ~retreat_dist: distanze per waypoints
- ~far_steps, ~pre_steps, ~approach_steps: numero waypoints per segmenti
- ~step_time: tempo tra punti joint durante il touch
- ~ik_timeout: timeout per query IK
- ~ik_service: nome esplicito del servizio IK (opzionale)
- ~allow_partial, ~min_partial_fraction: accetta soluzioni parziali per IK altrimenti si bagga tutto
- ~pos_tol_final
- ~tip_frame: frame fisico del tip che coicide con la punta del probe circa 
- ~tip_to_contact: fallback scalare EE→tip lungo +Z_EE
- ~flip_check: flip di Z_tool se punta "all'indietro" rispetto a P0 perchè gni tanto si scorda e penetra il cloudpint, boh 

Parametri aggiuntivi per SWEEP
- ~sweep_length (0.20): lunghezza desiderata se non si specifica ~pdes_hint
- ~sweep_samples (40): numero di campioni lungo la traccia
- ~sweep_step_time (0.20): tempo tra punti joint durante lo sweep
- ~sweep_contact_margin (=~contact_margin): offset di sicurezza in contatto durante lo sweep
- ~sweep_pref_dir ([1,0,0]): direzione preferita (nel ~ref_frame) proiettata sul piano tangente
- ~pdes_hint: [x,y,z] opzionale; P_des è lo snap sulla cloud del punto più vicino a questo hint
"""

import sys
import numpy as np
import rospy
import tf2_ros
import tf2_geometry_msgs  
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

# --- opzionale KD-Tree per nearest neighbor ---
try:
    from scipy.spatial import cKDTree
except Exception:
    cKDTree = None


# ---------------- utils ----------------
def norm(v, eps=1e-12):
    """Normalizzazione di un vettore, eps è una tolleranza."""
    v = np.asarray(v, float)
    n = np.linalg.norm(v)
    return v / n if n > eps else np.zeros_like(v)


def quat_from_axes(x, y, z):
    """
    Costruire un quaternione (x, y, z, w) a partire da tre assi cartesiani
    (x, y, z) che descrivono l’orientazione desiderata dello strumento
    """
    R = np.column_stack((norm(x), norm(y), norm(z)))
    T = np.eye(4)
    T[:3, :3] = R
    qx, qy, qz, qw = tft.quaternion_from_matrix(T)
    return np.array([qx, qy, qz, qw], float)


def quat_from_z_min_yaw(q_prev, z_des):
    """
    Trova un quaternion che allinei Z_tool a z_des cambiando il meno possibile lo yaw
    rispetto a q_prev. Proietta X_prev sul piano ortogonale a z_des.
    """
    q_prev = np.asarray(q_prev, float)
    Rprev = tft.quaternion_matrix(q_prev)[:3, :3]
    x_prev = Rprev[:, 0]
    z = norm(z_des)

    x_proj = x_prev - np.dot(x_prev, z) * z
    if np.linalg.norm(x_proj) < 1e-6:
        up = np.array([0, 0, 1.0])
        x_proj = up - np.dot(up, z) * z
        if np.linalg.norm(x_proj) < 1e-6:
            up = np.array([1, 0, 0.0])
            x_proj = up - np.dot(up, z) * z

    x = norm(x_proj)
    y = norm(np.cross(z, x))
    x = norm(np.cross(y, z))
    return quat_from_axes(x, y, z)


def slerp(q0, q1, s):
    """Interpolazione sferica tra q0 e q1 alla frazione s∈[0,1]."""
    return np.array(tft.quaternion_slerp(q0, q1, s), float)


def pose_from(p, q):
    """Costruisce un geometry_msgs/Pose da p(3,) e q(4,)."""
    ps = Pose()
    ps.position.x, ps.position.y, ps.position.z = float(p[0]), float(p[1]), float(p[2])
    ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w = q.tolist()
    return ps


def to_dict(names, vals):
    """Zip nomi giunti e valori in un dict {name: float(value)}."""
    return {n: float(v) for n, v in zip(names, vals)}


# ---------------- nodo principales ----------------
class PreToPoseAndTouch(object):
    """
    Muove un manipolatore per toccare un punto P0 sulla superficie point cloud,
    poi esegue uno sweep lineare mantenendo la normale fino a P_des,
    quindi si ritrae e torna in pre-approach.
    """

    def __init__(self):
        roscpp_initialize(sys.argv)
        rospy.init_node("pre_to_pose_and_sweep")

        # MoveIt / group configuration
        self.group_name = rospy.get_param("~group_name", "manipulator")
        self.ee_link = rospy.get_param("~ee_link", "iiwa_link_ee")
        self.ref_frame = rospy.get_param("~ref_frame", "world")
        self.speed_scale = float(rospy.get_param("~speed_scale", 0.20))

        # Step 1–2: nominal joint positions and fallback interpolation
        self.pre_joints = rospy.get_param("~pre_joints",
                                          [-2.529, 0.271, -0.268, 1.141, 2.932, 1.581, 0.174])
        self.target_joints = rospy.get_param("~target_joints",
                                             [-0.176, 0.675, 0.008, -0.789, -0.004, 1.669, -0.169])
        self.fallback_steps_pre = int(rospy.get_param("~fallback_steps_pre", 30))
        self.fallback_steps_tgt = int(rospy.get_param("~fallback_steps_tgt", 40))
        self.fallback_dt = float(rospy.get_param("~fallback_dt", 0.20))

        # Touch parameters
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

        # Tip: EE→tip via TF o scalar fallback lungo +Z_EE
        self.tip_frame = rospy.get_param("~tip_frame", "probe_tip")
        self.tip_to_contact = float(rospy.get_param("~tip_to_contact", 0.0))
        self.tip_vec_ee = None

        # Flip Z_tool se necessario boh non ho capito erchè così funziona
        self.flip_check = bool(rospy.get_param("~flip_check", True))

        # --- Sweep / scansione ---
        self.sweep_length = float(rospy.get_param("~sweep_length", 0.12))
        self.sweep_samples = int(rospy.get_param("~sweep_samples", 50))
        self.sweep_step_time = float(rospy.get_param("~sweep_step_time", 0.20))
        self.sweep_contact_margin = float(rospy.get_param("~sweep_contact_margin", self.contact_margin))
        self.sweep_pref_dir = np.asarray(rospy.get_param("~sweep_pref_dir", [1.0, 0.0, 0.0]), float)

        pdes_hint_param = rospy.get_param("~pdes_hint", [])
        self.pdes_hint = (np.asarray(pdes_hint_param, float)
                          if isinstance(pdes_hint_param, (list, tuple)) and len(pdes_hint_param) == 3
                          else None)

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

        # TF / Point cloud
        self.tf_buf = tf2_ros.Buffer(rospy.Duration(60.0))
        self.tf_lis = tf2_ros.TransformListener(self.tf_buf)

        self.points = None
        self.normals = None
        self.kdt = None
        rospy.Subscriber(self.cloud_topic, PointCloud2, self.cloud_cb, queue_size=1)

        # IK service handle
        self.ik_srv = None
        self.ik_name = None

        # Calcola vettore del tip
        self.compute_tip_vector()

        # --- Sequenza principale ---
        self.step_to_joints(self.pre_joints, self.fallback_steps_pre, "pre_approach")
        self.step_to_joints(self.target_joints, self.fallback_steps_tgt, "target_pose")
        self.touch_and_sweep_to_pdes()

        rospy.signal_shutdown("done")

    # ---------- cloud ----------
    def cloud_cb(self, msg):
        """
        Parsing della PointCloud2
        """
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

        # Costruzione KD-Tree opzionale, in teoria non mi serfe e sta qui soolo per safety 
        if self.points is not None and len(self.points) > 0 and cKDTree is not None:
            try:
                self.kdt = cKDTree(self.points)
            except Exception:
                self.kdt = None

    # ---------- tip vector ----------
    def compute_tip_vector(self):
        """
        Ricava EE→tip via TF (preferito); altrimenti usa offset scalare lungo +Z_EE.
        """
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
        """
        Esegue una traiettoria in joint-space (lista di dict {joint:pos}) con timing uniforme.
        """
        if not joints_seq or len(joints_seq) < 2:
            return False

        jt = JointTrajectory()
        jt.joint_names = list(self.joint_names)

        t = 0.0
        step = self.fallback_dt if dt is None else dt
        for jd in joints_seq:
            pt = JointTrajectoryPoint()
            pt.positions = [jd[n] for n in jt.joint_names]
            pt.velocities = [0.0] * len(jt.joint_names)
            pt.accelerations = [0.0] * len(jt.joint_names)
            t += max(step, 0.05)
            pt.time_from_start = rospy.Duration(t)
            jt.points.append(pt)

        traj = RobotTrajectory()
        traj.joint_trajectory = jt
        ok = self.group.execute(traj, wait=True)
        self.group.stop()
        return bool(ok)

    def step_to_joints(self, joint_vals, steps, tag):
        """
        Prova il planner verso joint_vals; se fallisce, interpola linearmente in joint space.
        """
        jd = {}
        # mappa i primi 7 joint iiwa_joint_1..7 se presenti
        for i in range(7):
            name = f"iiwa_joint_{i+1}"
            if name in self.joint_names:
                jd[name] = float(joint_vals[i])

        # Tentativo planner
        self.group.set_start_state_to_current_state()
        self.group.set_joint_value_target(jd)
        ok = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
        if ok:
            rospy.loginfo("Raggiunto %s via planner.", tag)
            return True

        # Fallback: retta in joint space
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
        """Risolve un servizio IK disponibile (cerca nomi comuni e opzionale ~ik_service)."""
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
        """
        Chiama il servizio IK per una PoseStamped; ritorna {joint:pos} o None.
        Primo tentativo con collision avoidance, poi senza.
        """
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
            rospy.logwarn("IK service failed: %s", str(e))
            return None

        if not hasattr(resp, "error_code") or resp.error_code.val != 1:
            return None

        js = resp.solution.joint_state
        name_to_pos = {n: p for n, p in zip(js.name, js.position)}
        target = {}
        for n in self.joint_names:
            if n in name_to_pos:
                target[n] = name_to_pos[n]
        return target if len(target) == len(self.joint_names) else None

    # ---------- waypoints (touch) ----------
    def make_waypoints(self, p_now, q_now, P0, n0):
        """
        Costruisce i waypoints far→pre→approach per il touch su P0.
        Ritorna (poses, tip_final)
        """
        R_now = tft.quaternion_matrix(q_now)[:3, :3]
        r_tip = self.tip_vec_ee if self.tip_vec_ee is not None else \
            np.array([0.0, 0.0, float(self.tip_to_contact)], float)
        p_tip_now = p_now + R_now.dot(r_tip)

        if n0 is not None:
            z_guess = -norm(n0)  # punta Z_tool verso la superficie
        else:
            z_guess = norm(P0 - p_tip_now)  # senza normali, punta a P0

        to_P0 = norm(P0 - p_tip_now)
        if self.flip_check and np.dot(z_guess, to_P0) < 0.0:
            z_guess = -z_guess
            rospy.loginfo("Flip Z_tool per coerenza con direzione verso P0.")

        q_des = quat_from_z_min_yaw(q_now, z_guess)
        R_des = tft.quaternion_matrix(q_des)[:3, :3]
        z_vec = R_des[:, 2]

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
        """
        Risolve l'IK su una sequenza di pose; accetta opzionalmente soluzione parziale altrimenti impazzisce
        Ritorna (joints_seq, solved, partial)
        """
        ps = PoseStamped()
        ps.header.frame_id = frame_id

        joints_seq = []
        solved = 0

        cur = self.group.get_current_joint_values()
        joints_seq.append(to_dict(self.joint_names, cur))

        for i, pose in enumerate(poses, 1):
            ps.header.stamp = rospy.Time.now()
            ps.pose = pose
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
            joints_seq.append(tgt)
            solved += 1

        return joints_seq, solved, False

    # ---------- selezione P0 ----------
    def pick_P0_ahead_of_tip(self):
        """
        Sceglie un punto della cloud davanti alla punta lungo −Z_tool, vicino al raggio
        Ritorna (P0, n0) con n0 opzionale.
        """
        # attende brevemente la cloud
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
        z_now = R_now[:, 2]
        dirn = -norm(z_now)

        r = self.points - p_tip[None, :]
        t = np.einsum('ij,j->i', r, dirn)
        idxs = np.where(t > 0.0)[0]
        if idxs.size == 0:
            idxs = np.arange(len(self.points))
            t = np.maximum(t, 0.0)
        d2 = np.einsum('ij,ij->i', r, r) - t * t
        best = idxs[np.argmin(d2[idxs])] if idxs.size > 0 else int(np.argmin(d2))

        P0 = self.points[best]
        n0 = self.normals[best] if (self.normals is not None and best < len(self.normals)) else None
        rospy.loginfo("P0 scelto: [%.3f, %.3f, %.3f] (idx %d).", P0[0], P0[1], P0[2], best)
        return (P0, n0)

    # ---------- helpers sweep ----------
    def nearest_index(self, p):
        """Indice del punto della cloud più vicino a p (usa KD-Tree se disponibile)."""
        if self.points is None or len(self.points) == 0:
            return None
        p = np.asarray(p, float)
        if self.kdt is not None:
            _, idx = self.kdt.query(p)
            return int(idx)
        d2 = np.sum((self.points - p[None, :]) ** 2, axis=1)
        return int(np.argmin(d2))

    def normal_at(self, idx_or_point, k=25):
        """
        Normale al punto. Se la cloud non fornisce normali, stima via PCA (SVD) sui k vicini ma in teoria la cloud ha le normali quindi tutto ok
        """
        if self.points is None or len(self.points) == 0:
            return np.array([0, 0, 1.0], float)

        if isinstance(idx_or_point, int):
            if self.normals is not None and idx_or_point < len(self.normals):
                return norm(self.normals[idx_or_point])
            p = self.points[idx_or_point]
        else:
            p = np.asarray(idx_or_point, float)

        # trova vicini
        if self.kdt is not None:
            k_ = min(k, len(self.points))
            _, idxs = self.kdt.query(p, k=k_)
            neigh = self.points[idxs if k_ > 1 else [idxs]]
        else:
            d2 = np.sum((self.points - p[None, :]) ** 2, axis=1)
            idxs = np.argsort(d2)[:min(k, len(self.points))]
            neigh = self.points[idxs]

        c = neigh.mean(axis=0)
        A = neigh - c
        try:
            _, _, vh = np.linalg.svd(A, full_matrices=False)
            n = vh[-1, :]
        except Exception:
            n = np.array([0, 0, 1.0], float)
        return norm(n)

    def tangent_from_normal(self, n, pref=None):
        """Direzione tangente ottenuta proiettando 'pref' sul piano ortogonale a n."""
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
        """
        Costruisce una traccia (punti superficie e normali) da P0 verso P_des:
        - Se ~pdes_hint è dato, P_des è lo snap al punto cloud più vicino a quell'hint.
        - Altrimenti, P_des = P0 + ~sweep_length * tangente(pref_dir,n).
        Ritorna: (track_pts, track_norms, Pdes_surf)
        """
        P0 = np.asarray(P0, float)

        if self.pdes_hint is not None:
            Pdes_wish = np.asarray(self.pdes_hint, float)
        else:
            n = n0 if n0 is not None else self.normal_at(self.nearest_index(P0))
            tdir = self.tangent_from_normal(n, self.sweep_pref_dir)
            Pdes_wish = P0 + self.sweep_length * tdir

        idx_end = self.nearest_index(Pdes_wish)
        if idx_end is None:
            rospy.logerr("Impossibile trovare P_des (cloud vuota).")
            return None, None, None
        Pdes_surf = self.points[idx_end]

        # campionamento lungo il segmento P0→Pdes_wish con snap alla superficie
        pts = []
        nors = []
        nS = max(2, int(self.sweep_samples))
        last_added = None
        for s in np.linspace(0.0, 1.0, nS):
            Pw = (1.0 - s) * P0 + s * Pdes_wish
            idx = self.nearest_index(Pw)
            if idx is None:
                continue
            Pi = self.points[idx]
            if last_added is None or np.linalg.norm(Pi - last_added) > 1e-4:
                ni = self.normal_at(idx)
                pts.append(Pi)
                nors.append(ni)
                last_added = Pi

        if len(pts) < 2:
            rospy.logerr("Traccia sweep troppo corta o cloud insufficiente.")
            return None, None, None

        return np.asarray(pts, float), np.asarray(nors, float), Pdes_surf

    def poses_from_surface_track(self, q_start, track_pts, track_norms):
        """
        Converte (punti superficie, normali) in pose dell'EE con Z_tool allineato a -normale.
        Mantiene lo yaw vicino a quello precedente (usa quat_from_z_min_yaw).
        """
        r_tip = self.tip_vec_ee if self.tip_vec_ee is not None else \
            np.array([0.0, 0.0, float(self.tip_to_contact)], float)
        q_prev = np.asarray(q_start, float)
        poses = []

        for Pi, ni in zip(track_pts, track_norms):
            z_guess = -norm(ni)
            q_des = quat_from_z_min_yaw(q_prev, z_guess)
            R_des = tft.quaternion_matrix(q_des)[:3, :3]
            tip_target = Pi - self.sweep_contact_margin * R_des[:, 2]
            ee_target = tip_target - R_des.dot(r_tip)
            poses.append(pose_from(ee_target, q_des))
            q_prev = q_des

        return poses

    def retract_from_surface(self, current_q, current_tip, lift_dist=None, steps=12):
        """
        Si alza di 'lift_dist' lungo +Z_tool e genera pose di retrazione.
        """
        if lift_dist is None:
            lift_dist = self.approach_dist + self.retreat_dist

        q = np.asarray(current_q, float)
        R = tft.quaternion_matrix(q)[:3, :3]
        z = R[:, 2]
        r_tip = self.tip_vec_ee if self.tip_vec_ee is not None else \
            np.array([0.0, 0.0, float(self.tip_to_contact)], float)

        tip_up = current_tip - lift_dist * z
        ee_up = tip_up - R.dot(r_tip)

        ee_now_pose = self.group.get_current_pose(self.ee_link).pose
        p_now = np.array([ee_now_pose.position.x, ee_now_pose.position.y, ee_now_pose.position.z], float)

        poses = [pose_from((1.0 - s) * p_now + s * ee_up, q)
                 for s in np.linspace(0.0, 1.0, max(2, steps))]
        return poses

    # ---------- pipeline completa totale ufffff----------
    def touch_and_sweep_to_pdes(self):
        """
        1) Seleziona P0 e normale
        2) Touch (far→pre→approach)
        3) Sweep lineare su superficie fino a P_des
        4) Retract e ritorno alla pre-approach
        """
        P0, n0 = self.pick_P0_ahead_of_tip()
        if P0 is None:
            rospy.logerr("Impossibile selezionare P0 dalla cloud.")
            return

        # --- Touch ---
        cur = self.group.get_current_pose(self.ee_link).pose
        p_now = np.array([cur.position.x, cur.position.y, cur.position.z], float)
        q_now = np.array([cur.orientation.x, cur.orientation.y, cur.orientation.z, cur.orientation.w], float)
        poses_touch, _tip_final = self.make_waypoints(p_now, q_now, P0, n0)

        joints_seq, solved, partial = self.ik_for_all(poses_touch, self.ref_frame)
        if joints_seq is None:
            rospy.logerr("IK insufficiente per la fase touch.")
            return
        rospy.loginfo("Eseguo traiettoria touch con %d/%d punti.", solved, len(poses_touch))
        ok_touch = self.exec_joint_traj(joints_seq, self.step_time)
        if not ok_touch:
            rospy.logerr("Esecuzione touch fallita.")
            return

        # Orientazione dopo il touch
        cur = self.group.get_current_pose(self.ee_link).pose
        q_touch = np.array([cur.orientation.x, cur.orientation.y, cur.orientation.z, cur.orientation.w], float)

        # --- Pianifica sweep ---
        track_pts, track_norms, Pdes_surf = self.plan_linear_surface_sweep(P0, n0)
        if track_pts is None:
            rospy.logerr("Pianificazione sweep fallita.")
            return

        # EE è già vicino al primo punto; costruiamo le pose dello sweep
        poses_sweep = self.poses_from_surface_track(q_touch, track_pts, track_norms)

        joints_seq2, solved2, partial2 = self.ik_for_all(poses_sweep, self.ref_frame)
        if joints_seq2 is None:
            rospy.logerr("IK insufficiente per sweep (risolti %d/%d).", solved2, len(poses_sweep))
            return

        rospy.loginfo("Eseguo sweep con %d/%d punti verso P_des.", solved2, len(poses_sweep))
        ok_sweep = self.exec_joint_traj(joints_seq2, self.sweep_step_time)
        if not ok_sweep:
            rospy.logerr("Esecuzione sweep fallita.")
            return

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

        # --- Rientro alla pre-approach ---
        self.step_to_joints(self.pre_joints, self.fallback_steps_pre, "pre_approach")
        rospy.loginfo("Sequenza touch → sweep → retract → pre-approach completata.")

    # ---------- (opzionale) touch only ----------
    def touch_cloud(self):
        """Esegue solo il touch su P0 (utility/retrocompatibilità) ma in reltà ho il codice a parte di questa cosa"""
        P0, n0 = self.pick_P0_ahead_of_tip()
        if P0 is None:
            rospy.logerr("Impossibile selezionare P0 dalla cloud.")
            return

        cur = self.group.get_current_pose(self.ee_link).pose
        p_now = np.array([cur.position.x, cur.position.y, cur.position.z], float)
        q_now = np.array([cur.orientation.x, cur.orientation.y, cur.orientation.z, cur.orientation.w], float)
        poses, _ = self.make_waypoints(p_now, q_now, P0, n0)
        joints_seq, solved, partial = self.ik_for_all(poses, self.ref_frame)
        if joints_seq is None:
            rospy.logerr("IK insufficiente per la fase touch.")
            return
        rospy.loginfo("Eseguo traiettoria touch (joint) con %d punti risolti su %d.",
                      solved, len(poses))
        ok = self.exec_joint_traj(joints_seq, self.step_time)
        if ok:
            rospy.loginfo("Touch eseguito%s.", " (parziale)" if partial else "")
        else:
            rospy.logerr("Esecuzione traiettoria touch fallita.")


if __name__ == "__main__":
    try:
        PreToPoseAndTouch()
    except rospy.ROSInterruptException:
        pass
    finally:
        roscpp_shutdown()

