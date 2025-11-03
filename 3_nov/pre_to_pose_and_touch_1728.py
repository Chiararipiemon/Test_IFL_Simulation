#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Non ci siamo ancora, oltre al fatto che rileva pure collisioni dove non dovrebbe. In generale il planner funziona ma non è consapevole della geometria del probe holder.
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
    roscpp_initialize, roscpp_shutdown, MoveGroupCommander, RobotCommander
)
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# ---------------- utils ----------------
def norm(v, eps=1e-12):
    n = np.linalg.norm(v)
    return v / n if n > eps else np.zeros_like(v)

def quat_from_axes(x, y, z):
    R = np.column_stack((norm(x), norm(y), norm(z)))
    T = np.eye(4); T[:3,:3] = R
    qx, qy, qz, qw = tft.quaternion_from_matrix(T)
    return np.array([qx, qy, qz, qw], float)

def quat_from_z_min_yaw(q_prev, z_des):
    q_prev = np.asarray(q_prev, float)
    Rprev = tft.quaternion_matrix(q_prev)[:3,:3]
    x_prev = Rprev[:,0]; z = norm(z_des)
    x_proj = x_prev - np.dot(x_prev, z) * z
    if np.linalg.norm(x_proj) < 1e-6:
        up = np.array([0,0,1.0]); x_proj = up - np.dot(up, z) * z
        if np.linalg.norm(x_proj) < 1e-6:
            up = np.array([1,0,0.0]); x_proj = up - np.dot(up, z) * z
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
class PreToPoseAndTouch(object):
    def __init__(self):
        roscpp_initialize(sys.argv)
        rospy.init_node("pre_to_pose_and_touch")

        # MoveIt / group
        self.group_name   = rospy.get_param("~group_name", "manipulator")
        self.ee_link      = rospy.get_param("~ee_link", "iiwa_link_ee")
        self.ref_frame    = rospy.get_param("~ref_frame", "world")
        self.speed_scale  = float(rospy.get_param("~speed_scale", 0.20))

        # Step 1-2 joint
        self.pre_joints    = rospy.get_param("~pre_joints",    [-2.529,  0.271, -0.268,  1.141,  2.932,  1.581,  0.174])
        self.target_joints = rospy.get_param("~target_joints", [-0.176,  0.675,  0.008, -0.789, -0.004,  1.669, -0.169])
        self.fallback_steps_pre  = int(rospy.get_param("~fallback_steps_pre",  30))
        self.fallback_steps_tgt  = int(rospy.get_param("~fallback_steps_tgt",  40))
        self.fallback_dt         = float(rospy.get_param("~fallback_dt", 0.20))

        # Step 3 touch
        self.cloud_topic     = rospy.get_param("~cloud_topic", "/cloud_with_normals")
        self.contact_margin  = float(rospy.get_param("~contact_margin", 0.006))
        self.approach_dist   = float(rospy.get_param("~approach_dist", 0.06))
        self.retreat_dist    = float(rospy.get_param("~retreat_dist", 0.18))
        self.far_steps       = int(rospy.get_param("~far_steps", 25))
        self.pre_steps       = int(rospy.get_param("~pre_steps", 18))
        self.approach_steps  = int(rospy.get_param("~approach_steps", 22))
        self.step_time       = float(rospy.get_param("~step_time", 0.25))
        self.ik_timeout      = float(rospy.get_param("~ik_timeout", 1.2))
        self.ik_service_param= rospy.get_param("~ik_service", "")

        self.allow_partial        = bool(rospy.get_param("~allow_partial", True))
        self.min_partial_fraction = float(rospy.get_param("~min_partial_fraction", 0.20))
        self.pos_tol_final        = float(rospy.get_param("~pos_tol_final", 0.02))

        # Punta: vettore EE->punta via TF (preferito) o fallback scalare
        self.tip_frame     = rospy.get_param("~tip_frame", "probe_tip")  # usa il tuo frame di punta
        self.tip_to_contact= float(rospy.get_param("~tip_to_contact", 0.0))  # fallback
        self.tip_vec_ee    = None

        # Flip automatico della normale se punta “andrebbe all’indietro”
        self.flip_check    = bool(rospy.get_param("~flip_check", True))

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

        # TF / Cloud
        self.tf_buf = tf2_ros.Buffer(rospy.Duration(60.0))
        self.tf_lis = tf2_ros.TransformListener(self.tf_buf)
        self.points = None; self.normals = None
        rospy.Subscriber(self.cloud_topic, PointCloud2, self.cloud_cb, queue_size=1)

        # IK
        self.ik_srv = None; self.ik_name = None

        # Calcola vettore punta
        self.compute_tip_vector()

        # Sequenza
        self.step_to_joints(self.pre_joints,  self.fallback_steps_pre,  "pre_approach")
        self.step_to_joints(self.target_joints, self.fallback_steps_tgt, "target_pose")
        self.touch_cloud()

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

        pts = []; nors = [] if has_norm else None
        if has_norm:
            for p in pc2.read_points(msg, field_names=("x","y","z",nx,ny,nz), skip_nans=True):
                pts.append([p[0], p[1], p[2]]); nors.append([p[3], p[4], p[5]])
        else:
            for p in pc2.read_points(msg, field_names=("x","y","z"), skip_nans=True):
                pts.append([p[0], p[1], p[2]])

        self.points = np.asarray(pts, float) if len(pts) else None
        if has_norm:
            self.normals = np.asarray(nors, float)
            nrm = np.linalg.norm(self.normals, axis=1); nrm[nrm==0]=1.0
            self.normals = self.normals / nrm[:,None]

    # ---------- tip vector ----------
    def compute_tip_vector(self):
        try:
            tfm = self.tf_buf.lookup_transform(self.ee_link, self.tip_frame, rospy.Time(0), rospy.Duration(2.0))
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
        jt = JointTrajectory(); jt.joint_names = list(self.joint_names)
        t = 0.0; step = self.fallback_dt if dt is None else dt
        for jd in joints_seq:
            pt = JointTrajectoryPoint()
            pt.positions = [jd[n] for n in jt.joint_names]
            pt.velocities = [0.0]*len(jt.joint_names)
            pt.accelerations = [0.0]*len(jt.joint_names)
            t += max(step, 0.05)
            pt.time_from_start = rospy.Duration(t)
            jt.points.append(pt)
        traj = RobotTrajectory(); traj.joint_trajectory = jt
        ok = self.group.execute(traj, wait=True)
        self.group.stop()
        return bool(ok)

    def step_to_joints(self, joint_vals, steps, tag):
        jd = {}
        for i in range(7):
            name = f"iiwa_joint_{i+1}"
            if name in self.joint_names:
                jd[name] = float(joint_vals[i])

        self.group.set_start_state_to_current_state()
        self.group.set_joint_value_target(jd)
        ok = self.group.go(wait=True)
        self.group.stop(); self.group.clear_pose_targets()
        if ok:
            rospy.loginfo("Raggiunto %s via planner.", tag)
            return True

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
        if self.ik_service_param: candidates.append(self.ik_service_param)
        ns = rospy.get_namespace()
        if ns != "/": candidates.append(ns + "compute_ik")
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
        if not self.ensure_ik(3.0): return None
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
        target = {}
        for n in self.joint_names:
            if n in name_to_pos:
                target[n] = name_to_pos[n]
        return target if len(target) == len(self.joint_names) else None

    # ---------- waypoints (con correzione verso Z_tool) ----------
    def make_waypoints(self, p_now, q_now, P0, n0):
        R_now = tft.quaternion_matrix(q_now)[:3,:3]
        r_tip = self.tip_vec_ee if self.tip_vec_ee is not None else np.array([0.0, 0.0, float(self.tip_to_contact)], float)
        p_tip_now = p_now + R_now.dot(r_tip)

        # 1) direzione desiderata per Z_tool
        if n0 is not None:
            z_guess = -norm(n0)  # Z_tool verso la superficie
        else:
            z_guess = norm(P0 - p_tip_now)  # in assenza di normali, punta P0

        # 2) check del verso: vogliamo che Z_tool punti **verso** P0 dalla punta corrente
        to_P0 = norm(P0 - p_tip_now)
        if self.flip_check and np.dot(z_guess, to_P0) < 0.0:
            z_guess = -z_guess
            rospy.loginfo("Flip Z_tool per coerenza con direzione verso P0.")

        # 3) orientazione finale
        q_des = quat_from_z_min_yaw(q_now, z_guess)
        R_des = tft.quaternion_matrix(q_des)[:3,:3]
        z_vec = R_des[:,2]

        # 4) target della PUNTA e dell'EE
        tip_final = P0 - self.contact_margin * z_vec
        ee_final  = tip_final - R_des.dot(r_tip)

        tip_pre   = P0 - (self.contact_margin + self.approach_dist) * z_vec
        tip_far   = P0 - (self.contact_margin + self.approach_dist + self.retreat_dist) * z_vec
        ee_pre    = tip_pre - R_des.dot(r_tip)
        ee_far    = tip_far - R_des.dot(r_tip)

        # 5) waypoints
        way_far = [pose_from((1.0 - s)*p_now + s*ee_far,  slerp(q_now, q_des, s))
                   for s in np.linspace(0.0, 1.0, max(2, self.far_steps))]
        way_pre = [pose_from((1.0 - s)*ee_far + s*ee_pre,  q_des)
                   for s in np.linspace(0.0, 1.0, max(2, self.pre_steps))]
        way_app = [pose_from((1.0 - s)*ee_pre + s*ee_final, q_des)
                   for s in np.linspace(0.0, 1.0, max(2, self.approach_steps))]

        return way_far + way_pre + way_app, tip_final

    def ik_for_all(self, poses, frame_id):
        ps = PoseStamped(); ps.header.frame_id = frame_id
        joints_seq = []; solved = 0
        cur = self.group.get_current_joint_values()
        joints_seq.append(to_dict(self.joint_names, cur))

        for i, pose in enumerate(poses, 1):
            ps.header.stamp = rospy.Time.now()
            ps.pose = pose
            tgt = self.ik_solve(ps, True) or self.ik_solve(ps, False)
            if tgt is None:
                frac = solved / float(len(poses))
                if self.allow_partial and frac >= self.min_partial_fraction:
                    rospy.logwarn("IK fallita al waypoint %d/%d: accetto parziale (frac=%.2f).", i, len(poses), frac)
                    return joints_seq, solved, True
                rospy.logerr("IK fallita al waypoint %d/%d e parziale non accettabile (frac=%.2f).", i, len(poses), frac)
                return None, solved, False
            joints_seq.append(tgt); solved += 1

        return joints_seq, solved, False

    # ---------- scelta P0  ----------
    def pick_P0_ahead_of_tip(self):
        for _ in range(200):
            if self.points is not None and len(self.points)>0: break
            rospy.sleep(0.05)
        if self.points is None or len(self.points)==0:
            rospy.logerr("Cloud non disponibile su %s", self.cloud_topic); return None, None

        cur = self.group.get_current_pose(self.ee_link).pose
        q_now = np.array([cur.orientation.x, cur.orientation.y, cur.orientation.z, cur.orientation.w], float)
        R_now = tft.quaternion_matrix(q_now)[:3,:3]
        r_tip = self.tip_vec_ee if self.tip_vec_ee is not None else np.array([0.0, 0.0, float(self.tip_to_contact)], float)
        p_tip = np.array([cur.position.x, cur.position.y, cur.position.z], float) + R_now.dot(r_tip)
        z_now = R_now[:,2]

        dirn = -norm(z_now)  # “davanti” alla punta lungo -Z_tool
        r = self.points - p_tip[None,:]
        t = np.einsum('ij,j->i', r, dirn)
        idxs = np.where(t > 0.0)[0]
        if idxs.size == 0:
            idxs = np.arange(len(self.points)); t = np.maximum(t, 0.0)
        d2 = np.einsum('ij,ij->i', r, r) - t*t
        best = idxs[np.argmin(d2[idxs])] if idxs.size>0 else int(np.argmin(d2))

        P0 = self.points[best]
        n0 = self.normals[best] if (self.normals is not None and best < len(self.normals)) else None
        rospy.loginfo("P0 scelto davanti alla punta: [%.3f, %.3f, %.3f] (idx %d).", P0[0], P0[1], P0[2], best)
        return (P0, n0)

    # ---------- touch ----------
    def touch_cloud(self):
        P0_n = self.pick_P0_ahead_of_tip()
        if P0_n is None:
            rospy.logerr("Impossibile selezionare P0 dalla cloud."); return
        P0, n0 = P0_n

        cur = self.group.get_current_pose(self.ee_link).pose
        p_now = np.array([cur.position.x, cur.position.y, cur.position.z], float)
        q_now = np.array([cur.orientation.x, cur.orientation.y, cur.orientation.z, cur.orientation.w], float)

        poses, _ = self.make_waypoints(p_now, q_now, P0, n0)
        joints_seq, solved, partial = self.ik_for_all(poses, self.ref_frame)
        if joints_seq is None:
            rospy.logerr("IK insufficiente per la fase touch."); return

        rospy.loginfo("Eseguo traiettoria touch (joint) con %d punti risolti su %d.", solved, len(poses))
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
