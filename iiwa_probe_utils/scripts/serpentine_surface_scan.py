#!/usr/bin/env python3
import os, sys, math, time, rospy, struct
from geometry_msgs.msg import Pose, PoseStamped
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander, roscpp_initialize, roscpp_shutdown

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
    # R as rows (row-major): R = ((r00,r01,r02),(r10,r11,r12),(r20,r21,r22))
    m00, m01, m02 = R[0][0], R[0][1], R[0][2]
    m10, m11, m12 = R[1][0], R[1][1], R[1][2]
    m20, m21, m22 = R[2][0], R[2][1], R[2][2]
    tr = m00+m11+m22
    if tr > 0:
        S = math.sqrt(tr+1.0)*2.0
        qw = 0.25*S
        qx = (m21 - m12)/S
        qy = (m02 - m20)/S
        qz = (m10 - m01)/S
    elif (m00 > m11) and (m00 > m22):
        S = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw = (m21 - m12) / S
        qx = 0.25 * S
        qy = (m01 + m10) / S
        qz = (m02 + m20) / S
    elif m11 > m22:
        S = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw = (m02 - m20) / S
        qx = (m01 + m10) / S
        qy = 0.25 * S
        qz = (m12 + m21) / S
    else:
        S = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw = (m10 - m01) / S
        qx = (m02 + m20) / S
        qy = (m12 + m21) / S
        qz = 0.25 * S
    return (qx, qy, qz, qw)

def rot_from_quat(q):
    # Return rotation matrix as rows (row-major)
    x,y,z,w = q
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z
    # rows
    R0 = (1-2*(yy+zz), 2*(xy - wz), 2*(xz + wy))
    R1 = (2*(xy + wz), 1-2*(xx+zz), 2*(yz - wx))
    R2 = (2*(xz - wy), 2*(yz + wx), 1-2*(xx+yy))
    return (R0, R1, R2)

def quat_multiply(q1,q2):
    x1,y1,z1,w1=q1; x2,y2,z2,w2=q2
    return (
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    )

def quat_from_rpy(roll,pitch,yaw):
    cy = math.cos(yaw*0.5); sy=math.sin(yaw*0.5)
    cp = math.cos(pitch*0.5); sp=math.sin(pitch*0.5)
    cr = math.cos(roll*0.5); sr=math.sin(roll*0.5)
    qw = cr*cp*cy + sr*sp*sy
    qx = sr*cp*cy - cr*sp*sy
    qy = cr*sp*cy + sr*cp*sy
    qz = cr*cp*sy - sr*sp*cy
    return (qx,qy,qz,qw)

# --- NEW helpers: quaternion ops, rotate vector, align two vectors ---
def quat_normalize(q):
    x,y,z,w = q
    n = math.sqrt(x*x + y*y + z*z + w*w) + 1e-12
    return (x/n, y/n, z/n, w/n)

def quat_conjugate(q):
    x,y,z,w = q
    return (-x, -y, -z, w)

def quat_rotate_vector(q, v):
    # v' = q * (v_quat) * q_conj
    qx,qy,qz,qw = q
    vx,vy,vz = v
    # quaternion multiplication q * v_quat
    ix =  qw*vx + qy*vz - qz*vy
    iy =  qw*vy + qz*vx - qx*vz
    iz =  qw*vz + qx*vy - qy*vx
    iw = -qx*vx - qy*vy - qz*vz
    # multiply result * q_conj
    rx = ix*qw - iw*qx + iy*qz - iz*qy
    ry = iy*qw - iw*qy + iz*qx - ix*qz
    rz = iz*qw - iw*qz + ix*qy - iy*qx
    return (rx, ry, rz)

def quat_from_axis_angle(axis, angle):
    ax = vec_norm(axis)
    s = math.sin(angle*0.5)
    return quat_normalize((ax[0]*s, ax[1]*s, ax[2]*s, math.cos(angle*0.5)))

def q_from_two_vectors(a, b):
    # returns quaternion that rotates vector a -> b
    ax = vec_norm(a); bx = vec_norm(b)
    dot = vec_dot(ax, bx)
    if dot >= 1.0 - 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    if dot <= -1.0 + 1e-12:
        # 180 deg: find an orthogonal axis
        ort = (1.0, 0.0, 0.0)
        if abs(ax[0]) > 0.9:
            ort = (0.0, 1.0, 0.0)
        axis = vec_norm(vec_cross(ax, ort))
        return quat_from_axis_angle(axis, math.pi)
    axis = vec_norm(vec_cross(ax, bx))
    angle = math.acos(max(-1.0, min(1.0, dot)))
    return quat_from_axis_angle(axis, angle)
def read_stl(path):
    # Returns list of triangles; for ASCII, parse lines; for binary, fixed-size records
    tris = []
    def is_binary():
        try:
            with open(path,'rb') as f:
                hdr=f.read(80)
                if len(hdr)<80: return False
                n_bytes = f.read(4)
                if len(n_bytes) < 4: return False
                n = int.from_bytes(n_bytes, 'little', signed=False)
                size = os.path.getsize(path)
                return size == 84 + n*50
        except Exception:
            return False
    try:
        if is_binary():
            with open(path,'rb') as f:
                f.seek(80)
                n = int.from_bytes(f.read(4),'little')
                for _ in range(n):
                    rec = f.read(50)
                    if len(rec)<50: break
                    v = list(struct.unpack('<12fH', rec))
                    v0=(v[3],v[4],v[5]); v1=(v[6],v[7],v[8]); v2=(v[9],v[10],v[11])
                    tris.append((v0,v1,v2))
        else:
            with open(path,'r',errors='ignore') as f:
                cur=[]
                for line in f:
                    t=line.strip().split()
                    if len(t)>=4 and t[0].lower()=='vertex':
                        try:
                            cur.append((float(t[1]), float(t[2]), float(t[3])))
                        except ValueError:
                            continue
                        if len(cur)==3:
                            tris.append(tuple(cur)); cur=[]
    except Exception as e:
        rospy.logerr("Error reading STL '%s': %s", path, str(e))
    return tris

def apply_transform_tris(tris, scale, q, t):
    # scale (sx,sy,sz), rotation quaternion q, translation t
    R = rot_from_quat(q)  # rows
    out=[]
    for (v0,v1,v2) in tris:
        vs=[]
        for v in (v0,v1,v2):
            x = v[0]*scale[0]; y=v[1]*scale[1]; z=v[2]*scale[2]
            # rotate using row-major matrix
            rx = R[0][0]*x + R[0][1]*y + R[0][2]*z
            ry = R[1][0]*x + R[1][1]*y + R[1][2]*z
            rz = R[2][0]*x + R[2][1]*y + R[2][2]*z
            # translate
            vs.append((rx+t[0], ry+t[1], rz+t[2]))
        out.append(tuple(vs))
    return out

def ray_tri_intersect_z(ray_origin, tris):
    # Ray direction is fixed: d=(0,0,-1)
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
        if abs(det) < 1e-12:  # parallel
            continue
        inv_det = 1.0/det
        tvec = (ox - v0[0], oy - v0[1], oz - v0[2])
        u = vec_dot(tvec, pvec)*inv_det
        if u < 0.0 or u > 1.0: continue
        qvec = vec_cross(tvec, e1)
        v = vec_dot(d, qvec)*inv_det
        if v < 0.0 or u + v > 1.0: continue
        t = vec_dot(e2, qvec)*inv_det
        if t <= 0.0:  # behind origin
            continue
        if (closest_t is None) or (t < closest_t):
            closest_t = t
            hit_point = (ox + d[0]*t, oy + d[1]*t, oz + d[2]*t)
            n = vec_norm(vec_cross(e1, e2))
            # Ensure normal points "up" (positive Z)
            if n[2] < 0: n = (-n[0], -n[1], -n[2])
            hit_normal = n
    return hit_point, hit_normal

def basis_from_normal(n, x_hint=(1.0,0.0,0.0)):
    z = vec_norm(n)
    # project hint on plane
    dot_h = vec_dot(x_hint, z)
    proj = (x_hint[0]-dot_h*z[0], x_hint[1]-dot_h*z[1], x_hint[2]-dot_h*z[2])
    if vec_len(proj) < 1e-6:
        x = (0.0, 1.0, 0.0)  # fallback
        dot_h = vec_dot(x, z)
        proj = (x[0]-dot_h*z[0], x[1]-dot_h*z[1], x[2]-dot_h*z[2])
    x = vec_norm(proj)
    y = vec_cross(z, x)
    # Return row-major matrix: rows = ( [x0,y0,z0], [x1,y1,z1], [x2,y2,z2] )
    R = ((x[0], y[0], z[0]), (x[1], y[1], z[1]), (x[2], y[2], z[2]))
    return R


def _compute_cartesian_path_robust(group, wps, eef_step, jump_thresh):
    """
    Try different MoveIt signatures across distros:
    1) (waypoints, eef_step, jump_threshold: float)
    2) (waypoints, eef_step, avoid_collisions: bool)
    3) (waypoints, eef_step, avoid_collisions: bool, path_constraints: bytes)
    Returns (trajectory, fraction) or (None, 0.0) on failure.
    """
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

    object_name = rospy.get_param("~object_name", "patient_surface")
    primary_axis = rospy.get_param("~primary_axis", "y")  # 'x' or 'y' (direction of lines)
    point_spacing = float(rospy.get_param("~point_spacing", 0.02))  # along line
    row_spacing    = float(rospy.get_param("~row_spacing", 0.03))   # distance between lines
    margin         = float(rospy.get_param("~margin", 0.03))        # shrink bbox to avoid edges
    clearance      = float(rospy.get_param("~clearance", 0.005))    # lift along normal (m)
    speed_scale    = float(rospy.get_param("~speed_scale", 0.2))
    allow_collisions = bool(rospy.get_param("~allow_collisions", False))
    max_path_time  = float(rospy.get_param("~max_path_time", 20.0))

    # Normalize namespace (avoid leading/trailing slashes issues)
    ns = rospy.get_namespace().strip('/')
    robot_ns = ns if ns else "iiwa"  # default
    ns_arg = '/' + robot_ns

    scene = PlanningSceneInterface(ns=ns_arg, synchronous=True)
    group = MoveGroupCommander("manipulator", ns=ns_arg)
    group.set_pose_reference_frame("world")
    try:
        group.set_end_effector_link("iiwa_link_ee")
    except Exception:
        rospy.logwarn("Failed to set end effector link; continuing.")
    group.set_max_velocity_scaling_factor(speed_scale)
    group.set_planning_time(10.0)

    # Retrieve object pose (handle Pose or PoseStamped)
    rospy.sleep(0.5)
    poses = scene.get_object_poses([object_name])
    if object_name not in poses:
        rospy.logerr("Object '%s' not found in PlanningScene.", object_name); sys.exit(1)
    obj_pose = poses[object_name]
    if isinstance(obj_pose, PoseStamped):
        obj_pose = obj_pose.pose
    q = (obj_pose.orientation.x, obj_pose.orientation.y, obj_pose.orientation.z, obj_pose.orientation.w)
    t = (obj_pose.position.x, obj_pose.position.y, obj_pose.position.z)

    # Get mesh file and scale from params
    mesh_path = rospy.get_param("~mesh_path", None)
    sx=sy=sz=1.0
    if not mesh_path:
        rospy.logwarn("~mesh_path not provided; trying to read from add_patient_mesh_on_table params...")
        possible_param_roots = ["/{}/add_patient_mesh_on_table".format(robot_ns), "~", "/add_patient_mesh_on_table"]
        for root in possible_param_roots:
            try:
                mp = rospy.get_param(root+"/mesh_path")
                sx = float(rospy.get_param(root+"/scale_x", 1.0))
                sy = float(rospy.get_param(root+"/scale_y", 1.0))
                sz = float(rospy.get_param(root+"/scale_z", 1.0))
                mesh_path = mp
                break
            except KeyError:
                continue
    if not mesh_path:
        rospy.logerr("Mesh path not found. Pass _mesh_path:=/path/to/file.stl"); sys.exit(1)

    # Read mesh triangles
    tris_local = read_stl(mesh_path)
    if not tris_local:
        rospy.logerr("Failed to read STL: %s", mesh_path); sys.exit(1)

    # Transform triangles to world
    tris_world = apply_transform_tris(tris_local, (sx,sy,sz), q, t)

    # Build bounding box in XY
    minx=miny= 1e9
    maxx=maxy=-1e9
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
        y = miny
        idx=0
        while y<=maxy+1e-9:
            xs = []
            if idx%2==0:
                xi=minx
                while xi<=maxx+1e-9:
                    xs.append(xi); xi += point_spacing
            else:
                xi=maxx
                while xi>=minx-1e-9:
                    xs.append(xi); xi -= point_spacing
            line=[(x, y) for x in xs]
            lines.append(line)
            idx+=1; y+=row_spacing
    else:
        x = minx
        idx=0
        while x<=maxx+1e-9:
            ys = []
            if idx%2==0:
                yi=miny
                while yi<=maxy+1e-9:
                    ys.append(yi); yi += point_spacing
            else:
                yi=maxy
                while yi>=miny-1e-9:
                    ys.append(yi); yi -= point_spacing
            line=[(x, y) for y in ys]
            lines.append(line)
            idx+=1; x+=row_spacing

    # Ray cast from above for each point
    z_high = maxz + 0.5
    waypoints_all = []
    for li, line in enumerate(lines):
        waypoints=[]
        for (x,y) in line:
            pt, normal = ray_tri_intersect_z((x,y,z_high), tris_world)
            if not pt:
                rospy.logwarn("No intersection at (%.3f, %.3f); skipping.", x, y)
                continue
            # Lift along normal by clearance
            pos = (pt[0] + normal[0]*clearance, pt[1] + normal[1]*clearance, pt[2] + normal[2]*clearance)

            # --- NEW: build quaternion so tool +Z aligns with surface normal ---
            # assume tool local +Z is the probe direction that must point to surface normal
            q_align = q_from_two_vectors((0.0, 0.0, 1.0), normal)

            # choose a yaw so that tool X aligns with world X projected on tangent plane
            world_x = (1.0, 0.0, 0.0)
            # vector that becomes tool X after q_align
            tool_x_after = quat_rotate_vector(q_align, world_x)
            # desired x direction = project world_x onto plane orthogonal to normal
            proj = (world_x[0] - vec_dot(world_x, normal)*normal[0],
                    world_x[1] - vec_dot(world_x, normal)*normal[1],
                    world_x[2] - vec_dot(world_x, normal)*normal[2])
            if vec_len(proj) < 1e-6:
                proj = (0.0, 1.0, 0.0)
            proj = vec_norm(proj)

            # compute signed angle between tool_x_after and proj around axis = normal
            cross_tp = vec_cross(tool_x_after, proj)
            sin_a = vec_dot(cross_tp, normal)
            cos_a = vec_dot(tool_x_after, proj)
            angle = math.atan2(sin_a, cos_a)

            q_yaw = quat_from_axis_angle(normal, angle)
            # apply yaw after alignment: q_total = q_yaw * q_align
            q_total = quat_multiply(q_yaw, q_align)
            q_total = quat_normalize(q_total)

            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = pos
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q_total
            waypoints.append(pose)
        if waypoints:
            waypoints_all.append(waypoints)

    if not waypoints_all:
        rospy.logerr("No waypoints generated; check params."); sys.exit(1)

    # Move and scan line by line using Cartesian paths
    eef_step = max(0.005, point_spacing/2.0)
    jump_thresh = 0.0
    for i, wps in enumerate(waypoints_all):
        if rospy.is_shutdown(): break
        rospy.loginfo("Cartesian path for line %d with %d waypoints ...", i, len(wps))
        (traj, fraction) = _compute_cartesian_path_robust(group, wps, eef_step, jump_thresh)
        rospy.loginfo("  fraction=%.3f", fraction)
        if fraction < 0.7:
            rospy.logwarn("Low fraction (%.2f). Trying to move to first point via plan+execute.", fraction)
            try:
                group.set_pose_target(wps[0])
                ok = group.go(wait=True)
            except Exception as e:
                rospy.logwarn("  Exception moving to first point: %s", str(e)); ok=False
            finally:
                group.stop(); group.clear_pose_targets()
            if not ok:
                rospy.logwarn("  Failed to reach first point; skipping this line.")
                continue
            (traj, fraction) = _compute_cartesian_path_robust(group, wps, eef_step, jump_thresh)
            rospy.loginfo("  retry fraction=%.3f", fraction)

        if traj and getattr(traj, 'joint_trajectory', None) and len(traj.joint_trajectory.points) > 0:
            # Optional: cap time if too long
            if max_path_time > 0:
                last = traj.joint_trajectory.points[-1].time_from_start.to_sec()
                if last > max_path_time:
                    rospy.logwarn("  Truncating trajectory from %.2fs to %.2fs", last, max_path_time)
                    pts = []
                    for pt in traj.joint_trajectory.points:
                        if pt.time_from_start.to_sec() <= max_path_time:
                            pts.append(pt)
                        else: break
                    traj.joint_trajectory.points = pts
            try:
                group.execute(traj, wait=True)
            except Exception as e:
                rospy.logerr("  Execute failed: %s", str(e))
        else:
            rospy.logwarn("  Empty trajectory for line %d", i)

    rospy.loginfo("Serpentine surface scan COMPLETED.")
    roscpp_shutdown()

if __name__ == "__main__":
    main()
