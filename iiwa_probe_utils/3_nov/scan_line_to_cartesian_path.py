#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import math
import numpy as np

import rospy
import tf2_ros
import tf2_geometry_msgs
import tf.transformations as tft

from geometry_msgs.msg import PointStamped, Pose, PoseStamped
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

from moveit_commander import (
    roscpp_initialize, roscpp_shutdown,
    RobotCommander, PlanningSceneInterface, MoveGroupCommander
)

def norm(v, eps=1e-12):
    n = np.linalg.norm(v)
    return v / n if n > eps else v * 0.0

def quat_from_axes(x, y, z):
    """Costruisce un quaternione a partire dagli assi X,Y,Z (colonne di R)."""
    R = np.column_stack((x, y, z))
    # Ortonormalizza e garantisci base destra
    u, _, vt = np.linalg.svd(R)
    R = u @ vt
    T = np.eye(4)
    T[:3, :3] = R
    qx, qy, qz, qw = tft.quaternion_from_matrix(T)  # (x,y,z,w)
    return (qx, qy, qz, qw)

class SurfaceScanPlanner(object):
    def __init__(self):
        roscpp_initialize(sys.argv)
        rospy.init_node("scan_line_to_cartesian_path")

        # Params
        self.cloud_topic     = rospy.get_param("~cloud_topic", "/cloud_with_normals")
        self.clicked_topic   = rospy.get_param("~clicked_topic", "/clicked_point")
        self.group_name      = rospy.get_param("~group_name", "manipulator")
        self.ee_link         = rospy.get_param("~ee_link", "iiwa_link_ee")
        self.N               = int(rospy.get_param("~N", 50))
        self.backoff         = float(rospy.get_param("~backoff", 0.010))  # m
        self.approach_offset = float(rospy.get_param("~approach_offset", 0.060))  # m
        self.eef_step        = float(rospy.get_param("~eef_step", 0.003))
        self.jump_threshold  = float(rospy.get_param("~jump_threshold", 0.0))
        self.speed_scale     = float(rospy.get_param("~speed_scale", 0.20))
        self.p0_param        = rospy.get_param("~P0", None)
        self.pdes_param      = rospy.get_param("~Pdes", None)
        self.planning_time     = float(rospy.get_param("~planning_time", 10.0))
        self.planning_attempts = int(rospy.get_param("~planning_attempts", 10))

        self.cloud = None
        self.cloud_frame = None
        self.points = None     # Nx3
        self.normals = None    # Nx3

        # TF
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(60.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # MoveIt
        ns = rospy.get_namespace()  # es. "/iiwa/"
        robot_description = (ns if ns != "/" else "") + "robot_description"
        self.group = MoveGroupCommander(self.group_name, robot_description=robot_description, ns=ns)
        self.group.set_end_effector_link(self.ee_link)
        self.group.set_max_velocity_scaling_factor(self.speed_scale)
        self.group.set_max_acceleration_scaling_factor(self.speed_scale)
        self.group.set_planning_time(self.planning_time)
        self.group.set_num_planning_attempts(self.planning_attempts)
        self.group.set_pose_reference_frame("world")  # si aggiorna a cloud_frame quando arriva la cloud

        # Subs
        rospy.Subscriber(self.cloud_topic, PointCloud2, self.cloud_cb, queue_size=1)
        self.clicked_pts = []
        if self.p0_param is None or self.pdes_param is None:
            rospy.Subscriber(self.clicked_topic, PointStamped, self.click_cb, queue_size=10)
            rospy.loginfo("Usa l'RViz 'Publish Point' e clicca due punti per definire P0 e Pdes.")

    # --- Callbacks ---
    def cloud_cb(self, msg):
        if self.cloud is None:
            rospy.loginfo("Cloud ricevuta: %d punti, frame: %s", msg.width * msg.height, msg.header.frame_id)
        self.cloud = msg
        self.cloud_frame = msg.header.frame_id
        self.group.set_pose_reference_frame(self.cloud_frame)
        # Parse points + normals in numpy
        fields = [f.name for f in msg.fields]
        nx_name = "normal_x" if "normal_x" in fields else ("nx" if "nx" in fields else None)
        ny_name = "normal_y" if "normal_y" in fields else ("ny" if "ny" in fields else None)
        nz_name = "normal_z" if "normal_z" in fields else ("nz" if "nz" in fields else None)
        if not (nx_name and ny_name and nz_name):
            rospy.logerr("La cloud non contiene campi delle normali (normal_x/y/z o nx/ny/nz).")
            return
        pts, nors = [], []
        for p in pc2.read_points(msg, field_names=("x","y","z",nx_name,ny_name,nz_name), skip_nans=True):
            pts.append([p[0], p[1], p[2]])
            nors.append([p[3], p[4], p[5]])
        self.points  = np.asarray(pts, dtype=float)
        self.normals = np.asarray(nors, dtype=float)
        nrm = np.linalg.norm(self.normals, axis=1)
        nrm[nrm == 0] = 1.0
        self.normals = self.normals / nrm[:,None]

    def click_cb(self, msg):
        if self.cloud_frame is None:
            rospy.logwarn("Cloud non ancora disponibile: ignoro il click.")
            return
        try:
            pt = self.tf_buffer.transform(msg, self.cloud_frame, rospy.Duration(1.0))
        except Exception as e:
            rospy.logwarn("TF fallita per clicked point -> cloud_frame: %s", str(e))
            return
        p = np.array([pt.point.x, pt.point.y, pt.point.z], dtype=float)
        self.clicked_pts.append(p)
        rospy.loginfo("Punto cliccato #%d: %s", len(self.clicked_pts), p)
        if len(self.clicked_pts) == 2:
            rospy.loginfo("Ricevuti P0 e Pdes. Avvio pianificazione.")
            self.plan_and_execute(self.clicked_pts[0], self.clicked_pts[1])
            self.clicked_pts = []  # consente nuovi tentativi

    # --- Core ---
    def nearest_on_cloud(self, q):
        d = self.points - q[None,:]
        idx = np.argmin(np.einsum('ij,ij->i', d, d))
        return self.points[idx], self.normals[idx]

    def build_waypoints(self, P0, Pdes):
        P0 = np.asarray(P0, float)
        Pdes = np.asarray(Pdes, float)
        scan_dir = norm(Pdes - P0)
        if np.linalg.norm(scan_dir) < 1e-9:
            raise RuntimeError("P0 e Pdes coincidono.")

        alphas = np.linspace(0.0, 1.0, self.N)
        wps = []
        for a in alphas:
            q = (1-a)*P0 + a*Pdes
            psurf, n = self.nearest_on_cloud(q)
            z_tool = -norm(n)  # Z del tool verso la superficie
            t = scan_dir - np.dot(scan_dir, n) * n  # proiezione sul piano tangente
            if np.linalg.norm(t) < 1e-6:
                world_hint = np.array([0.0, 1.0, 0.0])
                t = norm(np.cross(world_hint, z_tool))
                if np.linalg.norm(t) < 1e-6:
                    world_hint = np.array([1.0, 0.0, 0.0])
                    t = norm(np.cross(world_hint, z_tool))
            x_tool = norm(t)
            y_tool = norm(np.cross(z_tool, x_tool))  # terna destra
            x_tool = norm(np.cross(y_tool, z_tool))  # re-ortho

            p_tcp = psurf - self.backoff * n  # arretra lungo la normale
            qx, qy, qz, qw = quat_from_axes(x_tool, y_tool, z_tool)

            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = p_tcp.tolist()
            pose.orientation.x = qx; pose.orientation.y = qy
            pose.orientation.z = qz; pose.orientation.w = qw
            wps.append(pose)
        return wps

    def plan_and_execute(self, P0, Pdes):
        if self.points is None or self.normals is None:
            raise RuntimeError("Cloud con normali non disponibile.")

        # Waypoints sulla traiettoria offsettata
        waypoints = self.build_waypoints(P0, Pdes)

        # --- PRE-APPROACH (usiamo go() per evitare il problema del tuple) ---
        first_pose = waypoints[0]
        q = [first_pose.orientation.x,
             first_pose.orientation.y,
             first_pose.orientation.z,
             first_pose.orientation.w]
        R = tft.quaternion_matrix(q)[:3, :3]
        z_tool = R[:, 2]  # asse Z del tool

        pre_pose = Pose()
        pre_pose.position.x = first_pose.position.x + self.approach_offset * z_tool[0]
        pre_pose.position.y = first_pose.position.y + self.approach_offset * z_tool[1]
        pre_pose.position.z = first_pose.position.z + self.approach_offset * z_tool[2]
        pre_pose.orientation = first_pose.orientation

        pre_ps = PoseStamped()
        pre_ps.header.frame_id = self.cloud_frame
        pre_ps.pose = pre_pose

        self.group.set_start_state_to_current_state()
        self.group.set_pose_target(pre_ps)
        ok = self.group.go(wait=True)  # pianifica+esegue
        self.group.stop()
        self.group.clear_pose_targets()
        if not ok:
            rospy.logerr("Pianificazione pre-approach fallita (go()=False).")
            return

        # --- CARTESIAN PATH lungo i waypoints ---
        rospy.loginfo("Calcolo percorso cartesiano (%d waypoints)...", len(waypoints))
        (traj, fraction) = self.group.compute_cartesian_path(
            waypoints,
            eef_step=self.eef_step,
            jump_threshold=self.jump_threshold,
            avoid_collisions=True
        )
        rospy.loginfo("Frazione cartesian path: %.1f%%", 100.0*fraction)
        if hasattr(traj, "joint_trajectory") and len(traj.joint_trajectory.points) > 0:
            rospy.loginfo("Eseguo traiettoria cartesiana...")
            self.group.execute(traj, wait=True)
            self.group.stop()
        else:
            rospy.logerr("Traiettoria cartesiana vuota.")

    def wait_and_maybe_run_from_params(self):
        rate = rospy.Rate(10)
        ran = False
        while not rospy.is_shutdown():
            if self.cloud is not None and not ran:
                if self.p0_param is not None and self.pdes_param is not None:
                    P0 = np.asarray(self.p0_param, float)
                    Pdes = np.asarray(self.pdes_param, float)
                    rospy.loginfo("Uso P0/Pdes da parametri: %s -> %s", P0, Pdes)
                    try:
                        self.plan_and_execute(P0, Pdes)
                        ran = True
                    except Exception as e:
                        rospy.logerr("Errore nella pianificazione: %s", str(e))
                        ran = True
                # altrimenti attendo i click
            rate.sleep()

if __name__ == "__main__":
    try:
        planner = SurfaceScanPlanner()
        planner.wait_and_maybe_run_from_params()
    except rospy.ROSInterruptException:
        pass
    finally:
        roscpp_shutdown()

