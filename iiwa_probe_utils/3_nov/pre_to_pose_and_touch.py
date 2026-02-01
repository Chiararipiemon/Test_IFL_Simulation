#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import numpy as np

import rospy
import tf2_ros
import tf2_geometry_msgs
import tf.transformations as tft

from geometry_msgs.msg import Pose, PoseStamped, PointStamped
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

from moveit_commander import (
    roscpp_initialize, roscpp_shutdown, MoveGroupCommander, RobotCommander
)
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# ---------- util ----------
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

# ---------- main ----------
class PreToPoseAndTouch(object):
    def __init__(self):
        roscpp_initialize(sys.argv)
        rospy.init_node("pre_to_pose_and_touch")

        # Parametri MoveIt/joint
        self.group_name   = rospy.get_param("~group_name", "manipulator")
        self.ee_link      = rospy.get_param("~ee_link", "iiwa_link_ee")
        self.speed_scale  = float(rospy.get_param("~speed_scale", 0.20))
        self.ref_frame    = rospy.get_param("~ref_frame", "world")

        # Step 1 e 2 (joint)
        self.pre_joints_default    = [-2.529,  0.271, -0.268,  1.141,  2.932,  1.581,  0.174]
        self.target_joints_default = [-0.176,  0.675,  0.008, -0.789, -0.004,  1.669, -0.169]
        self.pre_joints    = rospy.get_param("~pre_joints",    self.pre_joints_default)
        self.target_joints = rospy.get_param("~target_joints", self.target_joints_default)

        self.fallback_steps_home = int(rospy.get_param("~fallback_steps_home", 30))
        self.fallback_steps_pre  = int(rospy.get_param("~fallback_steps_pre",  30))
        self.fallback_steps_tgt  = int(rospy.get_param("~fallback_steps_tgt",  40))
        self.fallback_dt         = float(rospy.get_param("~fallback_dt", 0.20))

        # Step 3 (touch cloud)
        self.cloud_topic     = rospy.get_param("~cloud_topic", "/cloud_with_normals")
        self.tip_to_contact  = float(rospy.get_param("~tip_to_contact", 0.00))  # m: distanza EE->punta lungo +Z_tool
        self.contact_margin  = float(rospy.get_param("~contact_margin", 0.005)) # m: fermati prima
        self.approach_dist   = float(rospy.get_param("~approach_dist", 0.08))   # m: pre-offset dalla superficie
        self.retreat_dist    = float(rospy.get_param("~retreat_dist", 0.12))    # m: far-pre extra
        self.far_steps       = int(rospy.get_param("~far_steps", 25))
        self.pre_steps       = int(rospy.get_param("~pre_steps", 18))
        self.approach_steps  = int(rospy.get_param("~approach_steps", 22))
        self.step_time       = float(rospy.get_param("~step_time", 0.25))
        self.ik_timeout      = float(rospy.get_param("~ik_timeout", 1.0))
        self.ik_service_param= rospy.get_param("~ik_service", "")

        # Accetta parziali
        self.allow_partial        = bool(rospy.get_param("~allow_partial", True))
        self.min_partial_fraction = float(rospy.get_param("~min_partial_fraction", 0.20))
        self.pos_tol_final        = float(rospy.get_param("~pos_tol_final", 0.02))

        # MoveIt
        ns = rospy.get_namespace()
        robot_description = (ns if ns != "/" else "") + "robot_description"
        self.robot = RobotCommander(robot_description=robot_description, ns=ns)
        self.group = MoveGroupCommander(self.group_name, robot_description=robot_description, ns=ns)
        self.group.set_end_effector_link(self.ee_link)
        self.group.set_pose_reference_frame(self.ref_frame)
        self.group.set_max_velocity_scaling_factor(self.speed_scale)
        self.group.set_max_acceleration_scaling_factor(self.speed_scale)
        self.joint_names = self.group.get_active_joints()

        # TF
        self.tf_buf = tf2_ros.Buffer(rospy.Duration(60.0))
        self.tf_lis = tf2_ros.TransformListener(self.tf_buf)

        # Cloud
        self.points = None
        self.normals = None
        rospy.Subscriber(self.cloud_topic, PointCloud2, self.cloud_cb, queue_size=1)

        # IK
        self.ik_srv = None
        self.ik_name = None

        # Sequenza
        self.step1_to_pre()
        self.step2_to_target()
        self.step3_touch_cloud()

        rospy.signal_shutdown("done")

    # ---------- Cloud ----------
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
            for p in pc2.read_points(msg, field_names=("x","y","z",nx,ny,nz), skip_nans=True):
                pts.append([p[0], p[1], p[2]])
                nors.append([p[3], p[4], p[5]])
        else:
            for p in pc2.read_points(msg, field_names=("x","y","z"), skip_nans=True):
                pts.append([p[0], p[1], p[2]])

        self.points = np.asarray(pts, float) if len(pts) else None
        if has_norm:
            self.normals = np.asarray(nors, float)
            nrm = np.linalg.norm(self.normals, axis=1); nrm[nrm==0]=1.0
            self.normals = self.normals / nrm[:,None]

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

    def go_or_fallback(self, joint_vals, steps, tag):
        jd = {}
        for i in range(7):
            name = f"iiwa_joint_{i+1}"
            if name in self.joint_names:
                jd[name] = float(joint_vals[i])

        # planner
        self.group.set_start_state_to_current_state()
        self.group.set_joint_value_target(jd)
        ok = self.group.go(wait=True)
        self.group.stop(); self.group.clear_pose_targets()
        if ok:
            rospy.loginfo("Raggiunto %s via planner.", tag)
            return True

        # fallback diretto
        cur = np.array(self.group.get_current_joint_values(), float)
        tgt = np.array([jd[n] for n in self.joint_names], float)
        seq = []
        for s in np.linspace(0.0, 1.0, max(2, int(steps))):
            pos = (1.0 - s) * cur + s * tgt
            seq.append({n: float(v) for n, v in zip(self.joint_names, pos)})
        rospy.logwarn("Planner fallito per %s. Eseguo fallback diretto (%d step).", tag, len(seq))
        return self.exec_joint_traj(seq, self.fallback_dt)

    # ---------- step 1 e 2 ----------
    def step1_to_pre(self):
        self.go_or_fallback(self.pre_joints, self.fallback_steps_pre, "pre_approach")

    def step2_to_target(self):
        self.go_or_fallback(self.target_joints, self.fallback_steps_tgt, "target_pose")

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

    # ---------- waypoints touch ----------
    def make_waypoints(self, p_now, q_now, P0, n0):
        # Z_tool desiderato: -normale
        z_des = -norm(n0) if n0 is not None else norm(P0 - p_now)
        q_des = quat_from_z_min_yaw(q_now, z_des)
        z_vec = tft.quaternion_matrix(q_des)[:3,2]

        far_pre = P0 - (self.tip_to_contact + self.approach_dist + self.retreat_dist) * z_vec
        pre_pos = P0 - (self.tip_to_contact + self.approach_dist) * z_vec
        final_p = P0 - (self.tip_to_contact + self.contact_margin) * z_vec

        way_far = [pose_from((1.0 - s)*p_now + s*far_pre, slerp(q_now, q_des, s))
                   for s in np.linspace(0.0, 1.0, max(2, self.far_steps))]
        way_pre = [pose_from((1.0 - s)*far_pre + s*pre_pos, q_des)
                   for s in np.linspace(0.0, 1.0, max(2, self.pre_steps))]
        way_app = [pose_from((1.0 - s)*pre_pos + s*final_p, q_des)
                   for s in np.linspace(0.0, 1.0, max(2, self.approach_steps))]
        return way_far + way_pre + way_app, final_p

    def ik_for_all(self, poses, frame_id, final_p):
        ps = PoseStamped(); ps.header.frame_id = frame_id
        joints_seq = []
        solved = 0
        # seed = stato corrente
        cur = self.group.get_current_joint_values()
        joints_seq.append(to_dict(self.joint_names, cur))
        last_good = None

        for i, pose in enumerate(poses, 1):
            ps.header.stamp = rospy.Time.now()
            ps.pose = pose

            tgt = self.ik_solve(ps, True) or self.ik_solve(ps, False)
            if tgt is None:
                frac = solved / float(len(poses))
                dist_final = None
                if last_good is not None:
                    lg = last_good.position
                    dist_final = np.linalg.norm(np.array([lg.x, lg.y, lg.z]) - np.array(final_p))
                if self.allow_partial and (frac >= self.min_partial_fraction or
                                           (dist_final is not None and dist_final <= self.pos_tol_final)):
                    rospy.logwarn("IK fallita al waypoint %d/%d: accetto parziale (frac=%.2f, dist_final=%s).",
                                  i, len(poses), frac, "N/A" if dist_final is None else f"{dist_final:.3f} m")
                    return joints_seq, solved, True
                rospy.logerr("IK fallita al waypoint %d/%d e parziale non accettabile (frac=%.2f).", i, len(poses), frac)
                return None, solved, False
            joints_seq.append(tgt)
            solved += 1
            last_good = pose

        return joints_seq, solved, False

    # ---------- scelta P0 (davanti alla punta del probe) ----------
    def pick_P0_ahead_of_tip(self):
        # attendi cloud
        for _ in range(200):
            if self.points is not None and len(self.points)>0:
                break
            rospy.sleep(0.05)
        if self.points is None or len(self.points)==0:
            rospy.logerr("Cloud non disponibile su %s", self.cloud_topic)
            return None, None

        cur = self.group.get_current_pose(self.ee_link).pose
        q_now = np.array([cur.orientation.x, cur.orientation.y, cur.orientation.z, cur.orientation.w], float)
        R = tft.quaternion_matrix(q_now)[:3,:3]
        z_now = R[:,2]  # +Z tool
        p_now = np.array([cur.position.x, cur.position.y, cur.position.z], float)
        p_tip = p_now + self.tip_to_contact * z_now

        # scegli punto più vicino alla retta p_tip + tau*(-z_now), tau>0 (cioè lungo -Z_tool)
        dirn = -norm(z_now)
        r = self.points - p_tip[None,:]
        t = np.einsum('ij,j->i', r, dirn)
        mask = t > 0.0
        idxs = np.where(mask)[0]
        if idxs.size == 0:
            idxs = np.arange(len(self.points)); t = np.maximum(t, 0.0)
        d2 = np.einsum('ij,ij->i', r, r) - t*t
        best = idxs[np.argmin(d2[idxs])] if idxs.size>0 else int(np.argmin(d2))

        P0 = self.points[best]
        n0 = self.normals[best] if (self.normals is not None and best < len(self.normals)) else None
        rospy.loginfo("P0 scelto davanti alla punta: [%.3f, %.3f, %.3f] (idx %d).", P0[0], P0[1], P0[2], best)
        return (P0, n0)

    # ---------- step 3 ----------
    def step3_touch_cloud(self):
        P0_n = self.pick_P0_ahead_of_tip()
        if P0_n is None:
            rospy.logerr("Impossibile selezionare P0 dalla cloud.")
            return
        P0, n0 = P0_n

        cur = self.group.get_current_pose(self.ee_link).pose
        p_now = np.array([cur.position.x, cur.position.y, cur.position.z], float)
        q_now = np.array([cur.orientation.x, cur.orientation.y, cur.orientation.z, cur.orientation.w], float)

        poses, final_p = self.make_waypoints(p_now, q_now, P0, n0)
        joints_seq, solved, partial = self.ik_for_all(poses, self.ref_frame, final_p)
        if joints_seq is None:
            rospy.logerr("IK insufficiente per la fase touch.")
            return

        rospy.loginfo("Eseguo traiettoria touch (joint) con %d punti risolti su %d.", solved, len(poses))
        ok = self.exec_joint_traj(joints_seq, self.step_time)
        if ok:
            if partial:
                rospy.loginfo("Touch eseguito in modo parziale (risolti %d/%d).", solved, len(poses))
            else:
                rospy.loginfo("Touch eseguito fino a contact_margin=%.3f m.", self.contact_margin)
        else:
            rospy.logerr("Esecuzione traiettoria touch fallita.")

if __name__ == "__main__":
    try:
        PreToPoseAndTouch()
    except rospy.ROSInterruptException:
        pass
    finally:
        roscpp_shutdown()
