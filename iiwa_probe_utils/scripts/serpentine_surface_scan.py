#!/usr/bin/env python3
import os, sys, math, time, rospy, struct
from geometry_msgs.msg import Pose
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, roscpp_initialize, roscpp_shutdown

# ------------- Math helpers -------------

def vec_add(a,b): return (a[0]+b[0], a[1]+b[1], a[2]+b[2])
def vec_sub(a,b): return (a[0]-b[0], a[1]-b[1], a[2]-b[2])
def vec_dot(a,b): return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]
def vec_cross(a,b): return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
def vec_len(a): return math.sqrt(vec_dot(a,a))
def vec_norm(a):
    l = vec_len(a)
    if l < 1e-12: return (0.0,0.0,0.0)
    return (a[0]/l, a[1]/l, a[2]/l)

def quat_from_matrix(R):
    # R as columns x,y,z
    m00, m01, m02 = R[0][0], R[0][1], R[0][2]
    m10, m11, m12 = R[1][0], R[1][1], R[1][2]
    m20, m21, m22 = R[2][0], R[2][1], R[2][2]
    tr = m00+m11+m22
    if tr > 0:
        S = math.sqrt(tr+1.0)*2
        qw = 0.25*S
        qx = (m21 - m12)/S
        qy = (m02 - m20)/S
        qz = (m10 - m01)/S
    elif (m00 > m11) and (m00 > m22):
        S = math.sqrt(1.0 + m00 - m11 - m22) * 2
        qw = (m21 - m12) / S
        qx = 0.25 * S
        qy = (m01 + m10) / S
        qz = (m02 + m20) / S
    elif m11 > m22:
        S = math.sqrt(1.0 + m11 - m00 - m22) * 2
        qw = (m02 - m20) / S
        qx = (m01 + m10) / S
        qy = 0.25 * S
        qz = (m12 + m21) / S
    else:
        S = math.sqrt(1.0 + m22 - m00 - m11) * 2
        qw = (m10 - m01) / S
        qx = (m02 + m20) / S
        qy = (m12 + m21) / S
        qz = 0.25 * S
    return (qx, qy, qz, qw)

def rot_from_quat(q):
    x,y,z,w = q
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z
    # columns
    X = (1-2*(yy+zz), 2*(xy+wz), 2*(xz-wy))
    Y = (2*(xy-wz), 1-2*(xx+zz), 2*(yz+wx))
    Z = (2*(xz+wy), 2*(yz-wx), 1-2*(xx+yy))
    return (X,Y,Z)

def quat_multiply(q1,q2):
    x1,y1,z1,w1=q1; x2,y2,z2,w2=q2
    return (
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    )

def read_stl(path):
    # Returns list of triangles; ASCII or binary STL
    tris = []
    def is_binary():
        try:
            with open(path,'rb') as f:
                hdr=f.read(80)
                if len(hdr)<80: return False
                n = int.from_bytes(f.read(4), 'little', signed=False)
                size = os.path.getsize(path)
                return size == 84 + n*50
        except Exception:
            return False
    if is_binary():
        with open(path,'rb') as f:
            f.seek(80)
            n = int.from_bytes(f.read(4),'little')
            for _ in range(n):
                rec = f.read(50)
                if len(rec)<50: break
                # 12 floats + ushort
                vals = struct.unpack('<12fH', rec)
                v0=(vals[3],vals[4],vals[5]); v1=(vals[6],vals[7],vals[8]); v2=(vals[9],vals[10],vals[11])
                tris.append((v0,v1,v2))
    else:
        with open(path,'r',errors='ignore') as f:
            cur=[]
            for line in f:
                t=line.strip().split()
                if len(t)>=4 and t[0].lower()=='vertex':
                    cur.append((float(t[1]), float(t[2]), float(t[3])))
                    if len(cur)==3:
                        tris.append(tuple(cur)); cur=[]
    return tris

def apply_transform_tris(tris, scale, q, t):
    # scale (sx,sy,sz), rotation quaternion q, translation t
    R = rot_from_quat(q)
    out=[]
    for (v0,v1,v2) in tris:
        vs=[]
        for v in (v0,v1,v2):
            x = v[0]*scale[0]; y=v[1]*scale[1]; z=v[2]*scale[2]
            # rotate
            rx = R[0][0]*x + R[0][1]*y + R[0][2]*z
            ry = R[1][0]*x + R[1][1]*y + R[1][2]*z
            rz = R[2][0]*x + R[2][1]*y + R[2][2]*z
            # translate
            vs.append((rx+t[0], ry+t[1], rz+t[2]))
        out.append(tuple(vs))
    return out

def ray_tri_intersect_z(ray_origin, tris):
    # Ray direction fixed: d=(0,0,-1)
    ox, oy, oz = ray_origin
    d = (0.0, 0.0, -1.0)
    closest_t = None
    hit_point = None
    hit_normal = None
    for (v0,v1,v2) in tris:
        e1 = vec_sub(v1, v0)
        e2 = vec_sub(v2, v0)
        pvec = vec_cross(d, e2)
        det = vec_dot(e1, pvec)
        if abs(det) < 1e-12:
            continue
        inv_det = 1.0/det
        tvec = (ox - v0[0], oy - v0[1], oz - v0[2])
        u = vec_dot(tvec, pvec)*inv_det
        if u < 0.0 or u > 1.0: continue
        qvec = vec_cross(tvec, e1)
        v = vec_dot(d, qvec)*inv_det
        if v < 0.0 or u + v > 1.0: continue
        t = vec_dot(e2, qvec)*inv_det
        if t <= 0.0:
            continue
        if (closest_t is None) or (t < closest_t):
            closest_t = t
            hit_point = (ox, oy, oz + d[2]*t)
            n = vec_norm(vec_cross(e1, e2))
            if n[2] < 0: n = (-n[0], -n[1], -n[2])  # orient upward
            hit_normal = n
    return hit_point, hit_normal

def compute_cartesian_path_robust(group, wps, eef_step, jump_thresh):
    # Try different MoveIt signatures across distros
    try:
        return group.compute_cartesian_path(wps, eef_step, jump_thresh)
    except TypeError:
        try:
            return group.compute_cartesian_path(wps, eef_step, True)
        except TypeError:
            try:
                return group.compute_cartesian_path(wps, eef_step, True, b'')
            except Exception as e:
                rospy.logerr("compute_cartesian_path failed on all variants: %s", str(e))
                return (None, 0.0)

# ------------- Main -------------

def main():
    roscpp_initialize(sys.argv)
    rospy.init_node("serpentine_surface_scan", anonymous=True)

    object_name   = rospy.get_param("~object_name", "patient_surface")
    primary_axis  = rospy.get_param("~primary_axis", "y")  # 'x' or 'y'
    point_spacing = float(rospy.get_param("~point_spacing", 0.02))
    row_spacing   = float(rospy.get_param("~row_spacing", 0.03))
    margin        = float(rospy.get_param("~margin", 0.03))
    clearance     = float(rospy.get_param("~clearance", 0.0))  # set 0 for true contact
    speed_scale   = float(rospy.get_param("~speed_scale", 0.2))
    allow_collisions = bool(rospy.get_param("~allow_collisions", False))
    max_path_time = float(rospy.get_param("~max_path_time", 20.0))
    eef_step      = float(rospy.get_param("~eef_step", 0.004))
    planning_time = float(rospy.get_param("~planning_time", 20.0))
    lock_roll_to_path = bool(rospy.get_param("~lock_roll_to_path", True))
    approach_lift = float(rospy.get_param("~approach_lift", 0.04))
    probe_tip_offset = float(rospy.get_param("~probe_tip_offset", 0.12))  # meters
    tip_gap = float(rospy.get_param("~tip_gap", 0.0))  # small positive if you want a visual gap

    ns = rospy.get_namespace().rstrip('/')
    robot_ns = ns if ns else "iiwa"
    scene = PlanningSceneInterface(ns="/"+robot_ns)
    group = MoveGroupCommander("manipulator", ns="/"+robot_ns)
    group.set_pose_reference_frame("world")
    group.set_end_effector_link("iiwa_link_ee")
    group.set_max_velocity_scaling_factor(speed_scale)
    group.set_planning_time(planning_time)
    try:
        group.set_num_planning_attempts(10)
    except Exception:
        pass

    # Retrieve object pose
    rospy.sleep(0.5)
    poses = scene.get_object_poses([object_name])
    if object_name not in poses:
        rospy.logerr("Object '%s' not found in PlanningScene.", object_name); sys.exit(1)
    obj_pose = poses[object_name]
    q_obj = (obj_pose.orientation.x, obj_pose.orientation.y, obj_pose.orientation.z, obj_pose.orientation.w)
    t_obj = (obj_pose.position.x, obj_pose.position.y, obj_pose.position.z)

    # Mesh path & scale
    mesh_path = rospy.get_param("~mesh_path", None)
    sx=sy=sz=1.0
    if not mesh_path:
        rospy.logerr("~mesh_path is required (pass _mesh_path:=/path/to/file.stl)"); sys.exit(1)
    else:
        sx = float(rospy.get_param("~scale_x", 1.0))
        sy = float(rospy.get_param("~scale_y", 1.0))
        sz = float(rospy.get_param("~scale_z", 1.0))

    # Read mesh triangles and transform to world
    tris_local = read_stl(mesh_path)
    if not tris_local:
        rospy.logerr("Failed to read STL: %s", mesh_path); sys.exit(1)
    tris_world = apply_transform_tris(tris_local, (sx,sy,sz), q_obj, t_obj)

    # Bounding box XY (shrink by margin)
    minx=miny=  1e9
    maxx=maxy= -1e9
    maxz = -1e9
    for (a,b,c) in tris_world:
        for (x,y,z) in (a,b,c):
            minx=min(minx,x); maxx=max(maxx,x)
            miny=min(miny,y); maxy=max(maxy,y)
            maxz=max(maxz,z)
    minx+=margin; maxx-=margin
    miny+=margin; maxy-=margin
    if maxx<=minx or maxy<=miny:
        rospy.logerr("Bounding box collapsed after margin; reduce margin."); sys.exit(1)

    # Build serpentine grid
    lines=[]
    if primary_axis.lower()=='x':
        y = miny; idx=0
        while y<=maxy+1e-9:
            xs = []
            if idx%2==0:
                xi=minx
                while xi<=maxx+1e-9: xs.append(xi); xi += point_spacing
            else:
                xi=maxx
                while xi>=minx-1e-9: xs.append(xi); xi -= point_spacing
            line=[(x, y) for x in xs]; lines.append(line)
            idx+=1; y+=row_spacing
    else:
        x = minx; idx=0
        while x<=maxx+1e-9:
            ys = []
            if idx%2==0:
                yi=miny
                while yi<=maxy+1e-9: ys.append(yi); yi += point_spacing
            else:
                yi=maxy
                while yi>=miny-1e-9: ys.append(yi); yi -= point_spacing
            line=[(x, y) for y in ys]; lines.append(line)
            idx+=1; x+=row_spacing

    # Ray cast from above for each point
    z_high = maxz + 0.5
    waypoints_all = []
    for li, line in enumerate(lines):
        waypoints=[]
        # scan direction (used for roll locking)
        if primary_axis.lower()=='y':
            line_dir = (0.0, 1.0, 0.0) if (li % 2 == 0) else (0.0, -1.0, 0.0)
        else:
            line_dir = (1.0, 0.0, 0.0) if (li % 2 == 0) else (-1.0, 0.0, 0.0)
        for (x,y) in line:
            pt, normal = ray_tri_intersect_z((x,y,z_high), tris_world)
            if not pt:
                rospy.logwarn("No intersection at (%.3f, %.3f); skipping.", x, y)
                continue

            # EE origin so that PROBE TIP (+Z of EE) lies on the surface:
            # pos = contact + n * (probe_tip_offset + tip_gap + clearance)
            pos = (pt[0] + normal[0]*(probe_tip_offset + tip_gap + clearance),
                   pt[1] + normal[1]*(probe_tip_offset + tip_gap + clearance),
                   pt[2] + normal[2]*(probe_tip_offset + tip_gap + clearance))

            # Orientation: set EE +Z axis to -normal (probe points toward the skin)
            z_axis = (-normal[0], -normal[1], -normal[2])

            # X axis: project scan direction onto plane orthogonal to z_axis
            x_hint = line_dir if lock_roll_to_path else (1.0, 0.0, 0.0)
            dot_h = vec_dot(x_hint, z_axis)
            proj = (x_hint[0]-dot_h*z_axis[0], x_hint[1]-dot_h*z_axis[1], x_hint[2]-dot_h*z_axis[2])
            if vec_len(proj) < 1e-6:
                proj = (0.0, 1.0, 0.0)
                dot_h = vec_dot(proj, z_axis)
                proj = (proj[0]-dot_h*z_axis[0], proj[1]-dot_h*z_axis[1], proj[2]-dot_h*z_axis[2])
            x_axis = vec_norm(proj)
            y_axis = vec_cross(z_axis, x_axis)
            R = (x_axis, y_axis, z_axis)
            q_orient = quat_from_matrix(R)

            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = pos
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q_orient
            waypoints.append(pose)
        if waypoints:
            # Approach: move away from skin along +normal, then descend to first point
            first = waypoints[0]
            # Recompute 'normal' at first point by casting again (simple and safe)
            pt_first, n_first = ray_tri_intersect_z((first.position.x, first.position.y, z_high), tris_world)
            if n_first is None:
                n_first = (0.0, 0.0, 1.0)
            approach = Pose()
            approach.position.x = first.position.x + n_first[0]*approach_lift
            approach.position.y = first.position.y + n_first[1]*approach_lift
            approach.position.z = first.position.z + n_first[2]*approach_lift
            approach.orientation = first.orientation

            rospy.loginfo("Moving to approach of line %d ...", li)
            group.set_pose_target(approach)
            ok = group.go(wait=True)
            group.stop(); group.clear_pose_targets()
            if ok:
                (traj_pre, frac_pre) = compute_cartesian_path_robust(group, [approach, first], eef_step, 0.0)
                if traj_pre and len(traj_pre.joint_trajectory.points)>0:
                    group.execute(traj_pre, wait=True)
            waypoints_all.append(waypoints)

    if not waypoints_all:
        rospy.logerr("No waypoints generated; check params."); sys.exit(1)

    # Execute line by line
    jump_thresh = 0.0
    for i, wps in enumerate(waypoints_all):
        if rospy.is_shutdown(): break
        rospy.loginfo("Cartesian path for line %d with %d waypoints ...", i, len(wps))
        (traj, fraction) = compute_cartesian_path_robust(group, wps, eef_step, jump_thresh)
        rospy.loginfo("  fraction=%.3f", fraction)
        if fraction < 0.7:
            rospy.logwarn("  Low fraction (%.2f). Trying to move to first point via plan+execute.", fraction)
            group.set_pose_target(wps[0])
            ok = group.go(wait=True)
            group.stop(); group.clear_pose_targets()
            if not ok:
                rospy.logwarn("  Failed to reach first point; skipping this line.")
                continue
            (traj, fraction) = compute_cartesian_path_robust(group, wps, eef_step, jump_thresh)
            rospy.loginfo("  retry fraction=%.3f", fraction)

        if traj and len(traj.joint_trajectory.points)>0:
            # Optional cap on total time
            if max_path_time > 0:
                last = traj.joint_trajectory.points[-1].time_from_start.to_sec()
                if last > max_path_time:
                    rospy.logwarn("  Truncating trajectory from %.2fs to %.2fs", last, max_path_time)
                    pts = []
                    for pt in traj.joint_trajectory.points:
                        if pt.time_from_start.to_sec() <= max_path_time:
                            pts.append(pt)
                        else:
                            break
                    traj.joint_trajectory.points = pts
            group.execute(traj, wait=True)
        else:
            rospy.logwarn("  Empty trajectory for line %d; skipping.", i)

    rospy.loginfo("Serpentine surface scan COMPLETED.")
    roscpp_shutdown()

if __name__ == "__main__":
    main()
