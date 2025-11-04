#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#4_nov
# Code for touching a point P0 on a point‑cloud surface using MoveIt and IK

author_doc = """
This ROS node moves a manipulator through three phases to gently touch a point
on a point cloud:

1) Move to a pre-approach joint pose.
2) Move to a target joint pose (e.g., better vantage point for the sensor).
3) Select a contact point P0 in front of the tool tip from a PointCloud2 topic,
   compute a collision-aware IK trajectory that orients the tool Z axis toward
   the surface normal (or toward P0 if normals are absent), and execute a
   far→pre→approach sequence to make contact with a configurable margin.

Highlights
- Uses TF to recover the vector from the end effector (EE) frame to the tool tip.
- If TF is unavailable, it falls back to a scalar tip offset along local Z.
- Chooses a P0 that lies in front of the tip along −Z_tool and near the ray.
- Builds waypoints that smoothly transition orientation (via slerp) and
  positions, then runs an IK solve per waypoint. Partial paths may be accepted
  if enough waypoints succeed.

Parameters (private namespace, with defaults shown)
- ~group_name: MoveIt planning group ("manipulator")
- ~ee_link: end effector link ("iiwa_link_ee")
- ~ref_frame: reference frame for poses ("world")
- ~speed_scale: MoveIt velocity/acceleration scale (0.20)
- ~pre_joints, ~target_joints: joint arrays for phases 1–2
- ~fallback_steps_pre, ~fallback_steps_tgt: linear joint interpolation steps if
  MoveIt planning fails
- ~fallback_dt: seconds between interpolated joint points
- ~cloud_topic: PointCloud2 topic with optional normals ("/cloud_with_normals")
- ~contact_margin: safety offset before contact along tool Z (0.006 m)
- ~approach_dist, ~retreat_dist: distances used to build waypoints
- ~far_steps, ~pre_steps, ~approach_steps: waypoint counts for each segment
- ~step_time: time between executed joint points during touch trajectory
- ~ik_timeout: timeout per IK query
- ~ik_service: explicit IK service name (optional)
- ~allow_partial, ~min_partial_fraction: accept partial IK solutions if this
  fraction of waypoints are solvable
- ~pos_tol_final: not used directly here but kept for potential final checks
- ~tip_frame: frame of the physical tip (preferred over scalar offset)
- ~tip_to_contact: fallback scalar offset from EE to tip along +Z_EE
- ~flip_check: flip desired Z_tool when it would point away from P0
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
    roscpp_initialize, roscpp_shutdown, MoveGroupCommander, RobotCommander
)
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# ---------------- utils ----------------
# Normalizzazione di un vettore, eps è una tolleranza
def norm(v, eps=1e-12):
    """Return the unit vector of v; if ||v|| <= eps"""
    n = np.linalg.norm(v)
    return v / n if n > eps else np.zeros_like(v)

# Costruire un quaternione (x, y, z, w) a partire da tre assi cartesiani di un frame (x, y, z) che descrivono l’orientazione desiderata dello strumento/EE

def quat_from_axes(x, y, z):
    """Build a quaternion from three (approximately) orthonormal axes.

    The axes are column-stacked into a rotation matrix (x, y, z). If inputs are
    not unit/orthogonal, they are normalized and re-orthogonalized implicitly by
    the caller; here we only normalize each axis.
    """
    R = np.column_stack((norm(x), norm(y), norm(z)))
    T = np.eye(4)
    T[:3, :3] = R
    qx, qy, qz, qw = tft.quaternion_from_matrix(T)
    return np.array([qx, qy, qz, qw], float)

# Questa funzione serve per trovare un quaternione che:
# allinei l’asse Z dello strumento (Z_tool) a un vettore desiderato z_des,
# cambi il meno possibile lo yaw (cioè la “testa/heading”) rispetto all’orientazione precedente q_prev.
def quat_from_z_min_yaw(q_prev, z_des):
    """Compute a quaternion whose Z axis follows z_des while minimally changing yaw.

    We project the previous X axis onto the plane orthogonal to z_des to anchor
    the yaw. This keeps the tool's heading as close as possible to the current
    yaw while aligning Z.
    """
    q_prev = np.asarray(q_prev, float)
    Rprev = tft.quaternion_matrix(q_prev)[:3, :3]
    x_prev = Rprev[:, 0]
    z = norm(z_des)

    # Project x_prev onto the plane orthogonal to z; if degenerate, choose a fallback up vector
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

# Serve per interpolare dolcemente un’orientazione tra due quaternioni q0 e q1 usando SLERP
def slerp(q0, q1, s):
    """Spherical linear interpolation between quaternions q0 and q1 at fraction s∈[0,1]."""
    return np.array(tft.quaternion_slerp(q0, q1, s), float)

#p: vettore di posizione di 3 elementi [x, y,z]
#q: quaternione di 4 elementi [qx,qy,qz,qw]
def pose_from(p, q):
    """Construct a geometry_msgs/Pose from position (3,) and quaternion (4,)."""
    ps = Pose()
    ps.position.x, ps.position.y, ps.position.z = float(p[0]), float(p[1]), float(p[2])
    ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w = q.tolist()
    return ps


# names: lista dei nomi dei giunti (sono stringhe)
# vals: lista dei valori corrispondenti (sono in radianti)
# è tipo un dizionario da giunto a valore corrispondente
def to_dict(names, vals):
    """Zip joint names and values into a {name: float(value)} dict."""
    return {n: float(v) for n, v in zip(names, vals)}


# ---------------- main node ----------------
class PreToPoseAndTouch(object):
    """Drive a manipulator to touch a point on a sensed surface

    Sequence at construction time:
      1) Move to a pre-approach joint configuration (ho scelto io questa pose)
      2) Move to a target joint configuration
      3) From a point cloud, pick a contact point P0 in front of the tool tip,
         generate waypoints (far → pre → approach), solve IK per waypoint, and
         execute the resulting joint trajectory to touch the surface
    """

    def __init__(self):
        roscpp_initialize(sys.argv)
        rospy.init_node("pre_to_pose_and_touch")

        # MoveIt / group configuration
        self.group_name = rospy.get_param("~group_name", "manipulator")
        self.ee_link = rospy.get_param("~ee_link", "iiwa_link_ee")
        self.ref_frame = rospy.get_param("~ref_frame", "world")
        self.speed_scale = float(rospy.get_param("~speed_scale", 0.20))

        # Step 1–2: nominal joint positions and fallback interpolation settings
        self.pre_joints = rospy.get_param("~pre_joints", [-2.529, 0.271, -0.268, 1.141, 2.932, 1.581, 0.174])
        self.target_joints = rospy.get_param("~target_joints", [-0.176, 0.675, 0.008, -0.789, -0.004, 1.669, -0.169])
        self.fallback_steps_pre = int(rospy.get_param("~fallback_steps_pre", 30))
        self.fallback_steps_tgt = int(rospy.get_param("~fallback_steps_tgt", 40))
        self.fallback_dt = float(rospy.get_param("~fallback_dt", 0.20))

        # Step 3: touch logic parameters
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

        # Tip: EE→tip vector recovered via TF (preferred) or scalar fallback along +Z_EE
        self.tip_frame = rospy.get_param("~tip_frame", "probe_tip")  # your physical tip frame
        self.tip_to_contact = float(rospy.get_param("~tip_to_contact", 0.0))  # scalar fallback
        self.tip_vec_ee = None

        # Automatically flip the desired tool Z if the tip would point "backwards" relative to P0
        self.flip_check = bool(rospy.get_param("~flip_check", True))

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

        # TF / Point cloud subscription
        self.tf_buf = tf2_ros.Buffer(rospy.Duration(60.0))
        self.tf_lis = tf2_ros.TransformListener(self.tf_buf)
        self.points = None
        self.normals = None
        rospy.Subscriber(self.cloud_topic, PointCloud2, self.cloud_cb, queue_size=1)

        # IK service handle: dato un target di posa dell’end-effector (posizione + orientazione, es. un PoseStamped), l’IK calcola i valori dei giunti (angoli/traslazioni) che portano il robot in quella posa.
        self.ik_srv = None
        self.ik_name = None

        # Compute the tip vector once (tries TF first)
        self.compute_tip_vector()

        # --- Main sequence ---
        self.step_to_joints(self.pre_joints, self.fallback_steps_pre, "pre_approach")
        self.step_to_joints(self.target_joints, self.fallback_steps_tgt, "target_pose")
        self.touch_cloud()

        rospy.signal_shutdown("done")

    # ---------- cloud ----------
    def cloud_cb(self, msg):
        """Parse incoming PointCloud2; optionally extract per-point normals if present.

        Recognizes fields named either (normal_x, normal_y, normal_z) or (nx, ny, nz).
        Stores arrays:
          - self.points: shape (N,3)
          - self.normals: shape (N,3) or None if not present
        Normals are normalized to unit length where available.
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
            # Read XYZ + normals
            for p in pc2.read_points(msg, field_names=("x", "y", "z", nx, ny, nz), skip_nans=True):
                pts.append([p[0], p[1], p[2]])
                nors.append([p[3], p[4], p[5]])
        else:
            # Read XYZ only
            for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                pts.append([p[0], p[1], p[2]])

        self.points = np.asarray(pts, float) if len(pts) else None
        if has_norm:
            self.normals = np.asarray(nors, float)
            nrm = np.linalg.norm(self.normals, axis=1)
            nrm[nrm == 0] = 1.0
            self.normals = self.normals / nrm[:, None]

    # ---------- tip vector ----------
    def compute_tip_vector(self):
        """Try to resolve the EE→tip vector via TF; otherwise use scalar fallback.

        The TF lookup obtains the translation from EE frame to tip frame. This
        allows for arbitrary tip placement (not only along +Z_EE).
        """
        try:
            tfm = self.tf_buf.lookup_transform(self.ee_link, self.tip_frame, rospy.Time(0), rospy.Duration(2.0))
            tx = tfm.transform.translation.x
            ty = tfm.transform.translation.y
            tz = tfm.transform.translation.z
            self.tip_vec_ee = np.array([tx, ty, tz], float)
            rospy.loginfo("tip_frame=%s -> vettore EE->tip = [%.3f, %.3f, %.3f] (frame %s).",
                          self.tip_frame, tx, ty, tz, self.ee_link)
        except Exception as e:
            # Fallback: keep None so later code uses tip_to_contact along +Z_EE
            self.tip_vec_ee = None
            rospy.logwarn("lookup_transform(%s->%s) fallita: %s. Fallback tip_to_contact=%.3f.",
                          self.ee_link, self.tip_frame, str(e), self.tip_to_contact)

    # ---------- joint helpers ----------
    def exec_joint_traj(self, joints_seq, dt=None):
        """Execute a precomputed joint-trajectory (list of {joint: pos}) with fixed timing

        For simplicity, velocities/accelerations are set to zero and timing is
        uniformly spaced by `dt`.
        """
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
        """Try to move to `joint_vals` using MoveIt's planner; if it fails, linearly interpolate.

        The fallback builds `steps` intermediate joint states from the current
        joint position to the target and executes them as a trajectory.
        """
        jd = {}
        for i in range(7):
            name = f"iiwa_joint_{i+1}"
            if name in self.joint_names:
                jd[name] = float(joint_vals[i])

        # Planner attempt
        self.group.set_start_state_to_current_state()
        self.group.set_joint_value_target(jd)
        ok = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
        if ok:
            rospy.loginfo("Raggiunto %s via planner.", tag)
            return True

        # Fallback: straight-line in joint space
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
        """Resolve an IK service to call, checking common names within our namespace."""
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
        """Call MoveIt's IK service for a PoseStamped; return {joint: pos} or None.

        Two attempts are made: first with collision avoidance, then without, if
        the first fails. The current robot state is used as a seed.
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

    # ---------- waypoints (with correction toward Z_tool) ----------
    def make_waypoints(self, p_now, q_now, P0, n0):
        """Build Cartesian waypoints that align the tool Z toward the surface and move to P0.

        Returns
        -------
        poses : list[geometry_msgs/Pose]
            Concatenated [far → pre → approach] EE poses in self.ref_frame.
        tip_final : ndarray shape (3,)
            Final tip position (i.e., contact point minus margin) in self.ref_frame.
        """
        R_now = tft.quaternion_matrix(q_now)[:3, :3]
        r_tip = self.tip_vec_ee if self.tip_vec_ee is not None else np.array([0.0, 0.0, float(self.tip_to_contact)], float)
        p_tip_now = p_now + R_now.dot(r_tip)

        # 1) Desired direction for Z_tool
        if n0 is not None:
            z_guess = -norm(n0)  # point tool Z toward the surface
        else:
            z_guess = norm(P0 - p_tip_now)  # if normals are absent, point directly to P0

        # 2) Ensure Z_tool points toward P0 from the current tip position
        to_P0 = norm(P0 - p_tip_now)
        if self.flip_check and np.dot(z_guess, to_P0) < 0.0:
            z_guess = -z_guess
            rospy.loginfo("Flip Z_tool per coerenza con direzione verso P0.")

        # 3) Final orientation
        q_des = quat_from_z_min_yaw(q_now, z_guess)
        R_des = tft.quaternion_matrix(q_des)[:3, :3]
        z_vec = R_des[:, 2]

        # 4) Tip and EE targets
        tip_final = P0 - self.contact_margin * z_vec
        ee_final = tip_final - R_des.dot(r_tip)

        tip_pre = P0 - (self.contact_margin + self.approach_dist) * z_vec
        tip_far = P0 - (self.contact_margin + self.approach_dist + self.retreat_dist) * z_vec
        ee_pre = tip_pre - R_des.dot(r_tip)
        ee_far = tip_far - R_des.dot(r_tip)

        # 5) Waypoint lists (interpolate position; hold or slerp orientation as needed)
        way_far = [pose_from((1.0 - s) * p_now + s * ee_far, slerp(q_now, q_des, s))
                   for s in np.linspace(0.0, 1.0, max(2, self.far_steps))]
        way_pre = [pose_from((1.0 - s) * ee_far + s * ee_pre, q_des)
                   for s in np.linspace(0.0, 1.0, max(2, self.pre_steps))]
        way_app = [pose_from((1.0 - s) * ee_pre + s * ee_final, q_des)
                   for s in np.linspace(0.0, 1.0, max(2, self.approach_steps))]

        return way_far + way_pre + way_app, tip_final

    def ik_for_all(self, poses, frame_id):
        """Solve IK for a sequence of EE poses; optionally accept partial results.

        Returns
        -------
        joints_seq : list[dict] or None
            A list of joint dictionaries beginning with the current joint state.
            None if unsolvable and partial solutions are not acceptable.
        solved : int
            Number of waypoints for which IK was found.
        partial : bool
            True if we stopped early and accepted a partial plan.
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
                    rospy.logwarn("IK fallita al waypoint %d/%d: accetto parziale (frac=%.2f).", i, len(poses), frac)
                    return joints_seq, solved, True
                rospy.logerr("IK fallita al waypoint %d/%d e parziale non accettabile (frac=%.2f).", i, len(poses), frac)
                return None, solved, False
            joints_seq.append(tgt)
            solved += 1

        return joints_seq, solved, False

    # ---------- choose P0 in front of the tip ---------- P0 non è un parametro fissato: viene calcolato a runtime nella funzione seguente
    def pick_P0_ahead_of_tip(self):
        """Pick a point-cloud index that lies in front of the tip along −Z_tool and near the ray.

        Heuristic:
        - Compute direction `dirn = -Z_tool` at the current tip position.
        - For each point r = (point - tip), compute t = r·dirn.
          Prefer points with t > 0 (in front); among them, minimize lateral
          distance squared d2 = ||r||^2 − t^2.
        - If no points have t > 0, fall back to minimizing d2 over all points.
        Returns (P0, n0) where n0 may be None if there are no normals
        """
        # Wait briefly for the cloud to arrive
        for _ in range(200):
            if self.points is not None and len(self.points) > 0:
                break
            rospy.sleep(0.05)
        if self.points is None or len(self.points) == 0:
            rospy.logerr("Cloud non disponibile su %s", self.cloud_topic)
            return None, None

        # Current EE pose and tip pose
        cur = self.group.get_current_pose(self.ee_link).pose
        q_now = np.array([cur.orientation.x, cur.orientation.y, cur.orientation.z, cur.orientation.w], float)
        R_now = tft.quaternion_matrix(q_now)[:3, :3]
        r_tip = self.tip_vec_ee if self.tip_vec_ee is not None else np.array([0.0, 0.0, float(self.tip_to_contact)], float)
        p_tip = np.array([cur.position.x, cur.position.y, cur.position.z], float) + R_now.dot(r_tip)
        z_now = R_now[:, 2]

        dirn = -norm(z_now)  # "forward" from the tip along −Z_tool
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
        rospy.loginfo("P0 scelto davanti alla punta: [%.3f, %.3f, %.3f] (idx %d).", P0[0], P0[1], P0[2], best)
        return (P0, n0)

    # ---------- touch ----------
    def touch_cloud(self):
        """Plan and execute the far→pre→approach touch sequence for the selected P0."""
        P0_n = self.pick_P0_ahead_of_tip()
        if P0_n is None:
            rospy.logerr("Impossibile selezionare P0 dalla cloud.")
            return
        P0, n0 = P0_n

        cur = self.group.get_current_pose(self.ee_link).pose
        p_now = np.array([cur.position.x, cur.position.y, cur.position.z], float)
        q_now = np.array([cur.orientation.x, cur.orientation.y, cur.orientation.z, cur.orientation.w], float)

        poses, _ = self.make_waypoints(p_now, q_now, P0, n0)
        joints_seq, solved, partial = self.ik_for_all(poses, self.ref_frame)
        if joints_seq is None:
            rospy.logerr("IK insufficiente per la fase touch.")
            return

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


