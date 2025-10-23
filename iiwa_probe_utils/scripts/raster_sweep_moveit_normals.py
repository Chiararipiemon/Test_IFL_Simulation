#!/usr/bin/env python3
import rospy, math
import numpy as np
from geometry_msgs.msg import Pose, Quaternion
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest
from moveit_msgs.msg import PlanningSceneComponents

# ---------- util ----------
def q_from_two_vectors(a, b):
    a = a / (np.linalg.norm(a) + 1e-12)
    b = b / (np.linalg.norm(b) + 1e-12)
    v = np.cross(a, b)
    c = np.dot(a, b)
    if c < -0.999999:
        axis = np.array([1.0, 0.0, 0.0])
        if abs(a[0]) > 0.9: axis = np.array([0.0, 1.0, 0.0])
        v = np.cross(a, axis); v /= (np.linalg.norm(v) + 1e-12)
        return np.array([v[0], v[1], v[2], 0.0])
    s = math.sqrt((1.0 + c) * 2.0)
    return np.array([v[0]/s, v[1]/s, v[2]/s, s*0.5])

def pose_xyz_q(x, y, z, q):
    p = Pose()
    p.position.x, p.position.y, p.position.z = float(x), float(y), float(z)
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    return p

def ray_triangle_intersect(O, D, v0, v1, v2):
    eps = 1e-9
    e1 = v1 - v0
    e2 = v2 - v0
    h = np.cross(D, e2)
    a = np.dot(e1, h)
    if -eps < a < eps: return None
    f = 1.0 / a
    s = O - v0
    u = f * np.dot(s, h)
    if u < 0.0 or u > 1.0: return None
    q = np.cross(s, e1)
    v = f * np.dot(s, q)
    if v < 0.0 or u + v > 1.0: return None
    t = f * np.dot(e2, q)
    if t > eps: return t
    return None

def quaternion_to_matrix(q):
    x, y, z, w = q
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z
    M = np.eye(4)
    M[0,0] = 1 - 2*(yy + zz)
    M[0,1] = 2*(xy - wz)
    M[0,2] = 2*(xz + wy)
    M[1,0] = 2*(xy + wz)
    M[1,1] = 1 - 2*(xx + zz)
    M[1,2] = 2*(yz - wx)
    M[2,0] = 2*(xz - wy)
    M[2,1] = 2*(yz + wx)
    M[2,2] = 1 - 2*(xx + yy)
    return M

# ---------- planning scene helper (no hang) ----------
def get_scene_proxy(ns="/iiwa", timeout=3.0):
    candidates = []
    if ns and ns != "/":
        candidates.append(ns + "/get_planning_scene")
    candidates.append("/get_planning_scene")
    last_err = None
    for s in candidates:
        try:
            rospy.loginfo("Attendo servizio: %s (timeout %.1fs)...", s, timeout)
            rospy.wait_for_service(s, timeout=timeout)
            rospy.loginfo("OK: userò il servizio %s", s)
            return rospy.ServiceProxy(s, GetPlanningScene)
        except rospy.ROSException as e:
            last_err = e
            rospy.logwarn("Non trovato %s: %s", s, str(e))
    raise rospy.ROSException("Nessun servizio get_planning_scene disponibile (provati: %s)" % ", ".join(candidates))

def load_mesh_triangles(ns, object_id):
    get_scene = get_scene_proxy(ns, timeout=3.0)
    req = GetPlanningSceneRequest()
    req.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
    scene = get_scene(req).scene
    tris = []
    count_meshes = 0
    for co in scene.world.collision_objects:
        if co.id != object_id:
            continue
        for mesh, mp in zip(co.meshes, co.mesh_poses):
            count_meshes += 1
            q = mp.orientation
            t = mp.position
            R = quaternion_to_matrix([q.x, q.y, q.z, q.w])
            M = R.copy()
            M[0:3, 3] = [t.x, t.y, t.z]
            V = np.array([[v.x, v.y, v.z] for v in mesh.vertices], dtype=float)
            Vw = (M @ np.hstack([V, np.ones((V.shape[0], 1))]).T).T[:, :3]
            for tri in mesh.triangles:
                v0 = Vw[tri.vertex_indices[0]]
                v1 = Vw[tri.vertex_indices[1]]
                v2 = Vw[tri.vertex_indices[2]]
                tris.append((v0, v1, v2))
    rospy.loginfo("Mesh '%s': %d submesh, %d triangoli.", object_id, count_meshes, len(tris))
    return tris

# ---------- raycast normale ----------
def surface_hit_and_normal(triangles, x, y, z_start):
    O = np.array([x, y, z_start], dtype=float)
    D = np.array([0.0, 0.0, -1.0], dtype=float)  # verso -Z
    best_t, best_tri = None, None
    for (v0, v1, v2) in triangles:
        t = ray_triangle_intersect(O, D, v0, v1, v2)
        if t is None or t <= 0: 
            continue
        if best_t is None or t < best_t:
            best_t, best_tri = t, (v0, v1, v2)
    if best_t is None:
        return None, None
    P = O + best_t * D
    v0, v1, v2 = best_tri
    n = np.cross(v1 - v0, v2 - v0)
    n_norm = np.linalg.norm(n) + 1e-12
    n = n / n_norm
    if n[2] < 0:  # scegli normale "verso l'alto"
        n = -n
    return P, n

def main():
    rospy.init_node("raster_sweep_moveit_normals", anonymous=True)

    # Parametri raster
    cx = rospy.get_param("~cx", 0.00)
    cy = rospy.get_param("~cy", 0.42)
    cz = rospy.get_param("~cz", 0.68)
    dx = rospy.get_param("~dx", 0.12)
    dy = rospy.get_param("~dy", 0.16)
    row_step = rospy.get_param("~row_step", 0.02)
    line_step = rospy.get_param("~line_step", 0.006)
    eef_step = rospy.get_param("~eef_step", 0.01)
    preclear_z = rospy.get_param("~preclear_z", 0.20)
    vel_scale = rospy.get_param("~vel_scale", 0.20)
    avoid_collisions = bool(rospy.get_param("~avoid_collisions", True))
    pass2 = bool(rospy.get_param("~pass2", False))
    contact_offset = rospy.get_param("~contact_offset", 0.0)
    patient_object = rospy.get_param("~patient_object", "patient_back")
    tool_axis = rospy.get_param("~tool_axis", "-z")
    tilt_deg = rospy.get_param("~tilt_deg", 0.0)
    eef_link_param = rospy.get_param("~eef_link", "tool0")  # <— forziamo tool0 di default

    # MoveIt
    desc_ns = "/iiwa" if rospy.has_param("/iiwa/robot_description") else ""
    ns = "/iiwa"
    RobotCommander(ns=desc_ns)
    scene = PlanningSceneInterface(ns=ns, synchronous=True)
    group = MoveGroupCommander("manipulator", ns=ns)
    group.set_pose_reference_frame("world")
    group.set_end_effector_link(eef_link_param)
    group.set_max_velocity_scaling_factor(vel_scale)
    group.set_max_acceleration_scaling_factor(vel_scale)
    group.set_planning_time(20.0)
    group.allow_replanning(True)

    rospy.sleep(0.8)
    rospy.loginfo("EEF link usato: %s", eef_link_param)

    # Carica triangoli dal planning scene (no hang)
    triangles = load_mesh_triangles(ns, patient_object)
    if not triangles:
        rospy.logerr("Mesh '%s' non trovata o vuota. Lancia add_patient_mesh e verifica id/pose.", patient_object)
        return

    def axis_vector(local_axis):
        mapping = {"x":[1,0,0], "+x":[1,0,0], "-x":[-1,0,0],
                   "y":[0,1,0], "+y":[0,1,0], "-y":[0,-1,0],
                   "z":[0,0,1], "+z":[0,0,1], "-z":[0,0,-1]}
        return np.array(mapping.get(local_axis, [0,0,-1]), dtype=float)

    tool_axis_vec = axis_vector(tool_axis)

    def orientation_from_normal(n, row_dir):
        q_align = q_from_two_vectors(tool_axis_vec, n)
        if abs(tilt_deg) > 1e-6:
            rd = row_dir / (np.linalg.norm(row_dir) + 1e-12)
            theta = math.radians(tilt_deg)
            s = math.sin(theta/2.0); c = math.cos(theta/2.0)
            q_tilt = np.array([rd[0]*s, rd[1]*s, rd[2]*s, c])
            ax, ay, az, aw = q_align
            bx, by, bz, bw = q_tilt
            q = np.array([
                aw*bx + ax*bw + ay*bz - az*by,
                aw*by - ax*bz + ay*bw + az*bx,
                aw*bz + ax*by - ay*bx + az*bw,
                aw*bw - ax*bx - ay*by - az*bz
            ])
            return q
        return q_align

    # griglia raster
    def gen_rows(orth=False):
        halfx, halfy = dx*0.5, dy*0.5
        ys = np.arange(cy - halfy, cy + halfy + 1e-9, row_step)
        x0, x1 = (cx - halfx, cx + halfx)
        for i, y in enumerate(ys):
            if rospy.is_shutdown(): break
            forward = (i % 2 == 0)
            xs = np.arange(x0, x1 + 1e-12, line_step)
            if not forward: xs = xs[::-1]
            row = []
            row_dir = np.array([1,0,0.0]) if not orth else np.array([0,1,0.0])
            for x in xs:
                X, Y = (x, y) if not orth else (y, x)
                hit, n = surface_hit_and_normal(triangles, X, Y, cz + preclear_z + 0.02)
                if hit is None: 
                    continue
                P = hit + n * contact_offset
                q = orientation_from_normal(n, row_dir if forward else -row_dir)
                row.append(pose_xyz_q(P[0], P[1], P[2], q))
            if row:
                yield i, row

    # move-to-start
    first_found = False
    for _, row in gen_rows(False):
        if not row: continue
        first = row[0]
        pre = Pose()
        pre.position.x, pre.position.y = first.position.x, first.position.y
        pre.position.z = cz + preclear_z
        pre.orientation = first.orientation
        group.set_start_state_to_current_state()
        group.set_pose_target(pre, eef_link_param)
        if group.go(wait=True):
            first_found = True
        group.stop(); group.clear_pose_targets()
        break

    if not first_found:
        rospy.logerr("Nessun punto valido trovato sulla mesh. Alza cz/preclear_z o verifica la mesh.")
        return

    def exec_row(poses, label):
        (plan, frac) = group.compute_cartesian_path(poses, eef_step, 0.0, avoid_collisions=avoid_collisions)
        if frac < 0.9:
            rospy.logwarn("[%s] cartesian fraction=%.2f -> fallback punti singoli", label, frac)
            for p in poses:
                group.set_pose_target(p, eef_link_param)
                if not group.go(wait=True):
                    group.stop(); group.clear_pose_targets()
                    rospy.logerr("[%s] fallito a punto singolo.", label)
                    return False
                group.stop(); group.clear_pose_targets()
            return True
        ok = group.execute(plan, wait=True)
        group.stop()
        return ok

    # passata principale (X)
    for i, row in gen_rows(False):
        if rospy.is_shutdown(): break
        if not exec_row(row, "pass1_row%d" % i):
            rospy.logerr("[pass1_row%d] fallita. Suggerimenti: alza cz/preclear_z, riduci dx/dy, aumenta vel_scale.", i)
            break

    # passata ortogonale (Y) opzionale
    if pass2 and not rospy.is_shutdown():
        for i, row in gen_rows(True):
            if rospy.is_shutdown(): break
            if not exec_row(row, "pass2_row%d" % i):
                rospy.logerr("[pass2_row%d] fallita.", i)
                break

    rospy.loginfo("Raster sweep (normali) COMPLETATA.")

if __name__ == "__main__":
    main()

