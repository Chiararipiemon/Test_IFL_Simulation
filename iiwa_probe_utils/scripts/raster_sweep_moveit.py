#!/usr/bin/env python3
import sys, math, rospy
from geometry_msgs.msg import Pose
from moveit_commander import (
    MoveGroupCommander, RobotCommander, PlanningSceneInterface, roscpp_initialize
)

def make_pose(x,y,z,orient):
    p = Pose()
    p.position.x, p.position.y, p.position.z = float(x), float(y), float(z)
    p.orientation = orient
    return p

def frange(a,b,step):
    x=a
    if step<=0: raise ValueError("step must be >0")
    # include b (epsilon)
    while x < b - 1e-12:
        yield x
        x += step
    yield b

def main():
    roscpp_initialize(sys.argv)
    rospy.init_node("raster_sweep_moveit")

    # ROI e parametri
    cx = float(rospy.get_param("~cx", 0.00))
    cy = float(rospy.get_param("~cy", 0.42))
    cz = float(rospy.get_param("~cz", 0.66))
    dx = float(rospy.get_param("~dx", 0.18))   # larghezza lungo X (numero righe)
    dy = float(rospy.get_param("~dy", 0.24))   # lunghezza riga lungo Y
    row_step   = float(rospy.get_param("~row_step",   0.02))  # distanza fra righe
    line_step  = float(rospy.get_param("~line_step",  0.006)) # passo lungo la riga
    eef_step   = float(rospy.get_param("~eef_step",   0.01))  # risoluzione cartesiana
    preclear_z = float(rospy.get_param("~preclear_z", 0.12))
    vel_scale  = float(rospy.get_param("~vel_scale",  0.20))
    avoid_collisions = bool(rospy.get_param("~avoid_collisions", True))
    # pass2 perpendicolare opzionale
    do_pass2 = bool(rospy.get_param("~pass2", False))

    # MoveIt (ns /iiwa, come negli script che ti funzionano)
    robot = RobotCommander(ns="/iiwa")
    scene = PlanningSceneInterface(ns="/iiwa", synchronous=True)
    rospy.sleep(0.5)
    rospy.loginfo("Known objects in scene: %s", scene.get_known_object_names())
    group = MoveGroupCommander("manipulator", ns="/iiwa")
    group.set_end_effector_link("iiwa_link_ee")
    group.set_pose_reference_frame("world")
    group.set_planning_time(10.0)
    group.set_num_planning_attempts(5)
    group.allow_replanning(True)
    group.set_max_velocity_scaling_factor(vel_scale)
    group.set_max_acceleration_scaling_factor(vel_scale)

    # Orientazione: usa quella ATTUALE (così non rompiamo l’IK)
    orient = group.get_current_pose("iiwa_link_ee").pose.orientation

    x0, x1 = cx - dx/2.0, cx + dx/2.0
    y0, y1 = cy - dy/2.0, cy + dy/2.0

    def plan_and_exec_row(pts, label="row"):
        # move-to-start con pre-lift
        start = pts[0]
        pre = make_pose(start.position.x, start.position.y, start.position.z + preclear_z, start.orientation)
        group.set_start_state_to_current_state()
        group.set_pose_target(pre); ok1 = group.go(wait=True)
        group.stop(); group.clear_pose_targets()
        group.set_pose_target(start); ok2 = group.go(wait=True)
        group.stop(); group.clear_pose_targets()
        if not (ok1 and ok2):
            rospy.logerr("[%s] move-to-start failed (prova cz + 0.02 o riduci dy/dx).", label)
            return False

        # cartesiano (riga singola = robusto)
        try:
            plan, fraction = group.compute_cartesian_path(pts, eef_step, avoid_collisions)
            plan, fraction = group.compute_cartesian_path(pts, eef_step, jump_threshold=0.0, avoid_collisions=avoid_collisions)
        except TypeError:
            plan, fraction = group.compute_cartesian_path(pts, eef_step, 0.0)
          # fallback for older moveit_commander versions without avoid_collisions kw
            plan, fraction = group.compute_cartesian_path(pts, eef_step, 0.0)
        rospy.loginfo("[%s] Cartesian fraction: %.1f%%", label, 100.0*float(fraction))

        if getattr(plan, "joint_trajectory", None) and len(plan.joint_trajectory.points) > 1 and fraction > 0.8:
            group.execute(plan, wait=True)
            return True
        else:
            rospy.logerr("[%s] cartesian path insufficiente. (Alza cz, riduci line_step o set avoid_collisions:=false per test)", label)
            return False

    # -------- Passata #1: righe a X costante, movimento lungo Y (serpentina) --------
    rows = []
    row_idx = 0
    for xi in frange(x0, x1, row_step):
        if row_idx % 2 == 0:
            ys = list(frange(y0, y1, line_step))  # Y crescente
        else:
            ys = list(frange(y1, y0, line_step))  # Y decrescente
        pts = [make_pose(xi, y, cz, orient) for y in ys]
        rows.append(pts)
        row_idx += 1

    for i, pts in enumerate(rows):
        if rospy.is_shutdown(): break
        ok = plan_and_exec_row(pts, label=f"pass1_row{i}")
        if not ok: break

    # -------- (Opzionale) Passata #2: righe a Y costante, movimento lungo X --------
    if do_pass2 and not rospy.is_shutdown():
        rows2 = []
        row_idx = 0
        for yi in frange(y0, y1, row_step):
            if row_idx % 2 == 0:
                xs = list(frange(x0, x1, line_step))
            else:
                xs = list(frange(x1, x0, line_step))
            pts = [make_pose(x, yi, cz, orient) for x in xs]
            rows2.append(pts)
            row_idx += 1
        for i, pts in enumerate(rows2):
            if rospy.is_shutdown(): break
            ok = plan_and_exec_row(pts, label=f"pass2_row{i}")
            if not ok: break

    rospy.loginfo("Raster sweep COMPLETATO.")

if __name__ == "__main__":
    main()
