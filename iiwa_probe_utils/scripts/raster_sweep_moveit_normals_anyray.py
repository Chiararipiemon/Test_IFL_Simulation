#!/usr/bin/env python3
import rospy, math, numpy as np
from geometry_msgs.msg import Pose, Quaternion
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest
from moveit_msgs.msg import PlanningSceneComponents

# -------- utils base --------
def q_from_two_vectors(a, b):
    a = a / (np.linalg.norm(a) + 1e-12)
    b = b / (np.linalg.norm(b) + 1e-12)
    v = np.cross(a, b); c = np.dot(a, b)
    if c < -0.999999:
        axis = np.array([1.0,0.0,0.0]) if abs(a[0])<0.9 else np.array([0.0,1.0,0.0])
        v = np.cross(a, axis); v /= (np.linalg.norm(v)+1e-12)
        return np.array([v[0], v[1], v[2], 0.0])
    s = math.sqrt((1.0 + c) * 2.0)
    return np.array([v[0]/s, v[1]/s, v[2]/s, s*0.5])

def pose_xyz_q(x, y, z, q):
    p = Pose(); p.position.x, p.position.y, p.position.z = float(x), float(y), float(z)
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]); return p

def quaternion_to_matrix(q):
    x,y,z,w = q; xx,yy,zz=x*x,y*y,z*z; xy,xz,yz=x*y,x*z,y*z; wx,wy,wz=w*x,w*y,w*z
    M = np.eye(4)
    M[0,0]=1-2*(yy+zz); M[0,1]=2*(xy-wz); M[0,2]=2*(xz+wy)
    M[1,0]=2*(xy+wz);   M[1,1]=1-2*(xx+zz); M[1,2]=2*(yz-wx)
    M[2,0]=2*(xz-wy);   M[2,1]=2*(yz+wx);   M[2,2]=1-2*(xx+yy)
    return M

def axis_vector(a):
    m={"x":[1,0,0],"+x":[1,0,0],"-x":[-1,0,0],
       "y":[0,1,0],"+y":[0,1,0],"-y":[0,-1,0],
       "z":[0,0,1],"+z":[0,0,1],"-z":[0,0,-1]}
    return np.array(m.get(a,[0,0,-1]), dtype=float)

# -------- planning scene helpers --------
def get_scene_proxy(ns="/iiwa", timeout=3.0):
    for s in ([ns + "/get_planning_scene"] if ns and ns!="/" else []) + ["/get_planning_scene"]:
        try: rospy.wait_for_service(s, timeout=timeout); return rospy.ServiceProxy(s, GetPlanningScene)
        except rospy.ROSException: pass
    raise rospy.ROSException("get_planning_scene non trovato")

def load_mesh_arrays(ns, object_id):
    get_scene = get_scene_proxy(ns, timeout=3.0)
    req = GetPlanningSceneRequest(); req.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
    scene = get_scene(req).scene
    V0,V1,V2 = [],[],[]
    for co in scene.world.collision_objects:
        if co.id != object_id: continue
        for mesh, mp in zip(co.meshes, co.mesh_poses):
            q,t = mp.orientation, mp.position
            R = quaternion_to_matrix([q.x,q.y,q.z,q.w]); M = R.copy(); M[0:3,3]=[t.x,t.y,t.z]
            V = np.array([[v.x, v.y, v.z] for v in mesh.vertices], dtype=float)
            Vw = (M @ np.hstack([V, np.ones((V.shape[0],1))]).T).T[:, :3]
            for tri in mesh.triangles:
                V0.append(Vw[tri.vertex_indices[0]]); V1.append(Vw[tri.vertex_indices[1]]); V2.append(Vw[tri.vertex_indices[2]])
    if not V0: return None
    V0 = np.array(V0); V1 = np.array(V1); V2 = np.array(V2)
    xmin = np.minimum.reduce([V0[:,0], V1[:,0], V2[:,0]])
    xmax = np.maximum.reduce([V0[:,0], V1[:,0], V2[:,0]])
    ymin = np.minimum.reduce([V0[:,1], V1[:,1], V2[:,1]])
    ymax = np.maximum.reduce([V0[:,1], V1[:,1], V2[:,1]])
    zmin = np.minimum.reduce([V0[:,2], V1[:,2], V2[:,2]])
    zmax = np.maximum.reduce([V0[:,2], V1[:,2], V2[:,2]])
    rospy.loginfo("Triangoli mesh: %d", V0.shape[0])
    return dict(V0=V0,V1=V1,V2=V2,xmin=xmin,xmax=xmax,ymin=ymin,ymax=ymax,zmin=zmin,zmax=zmax)

# -------- ray helpers (bbox prefilter + Möller–Trumbore) --------
def dims_for_ray(ray_axis):
    # restituisce (i,j,k) dove k è la componente lungo il raggio, i e j sono le due trasversali usate per la bbox
    if ray_axis in ("-z","+z"): return (0,1,2)   # x,y | z
    if ray_axis in ("-y","+y"): return (0,2,1)   # x,z | y
    return (1,2,0)                               # default: y,z | x  (per ±x)

def ray_hit_normal_filtered(M, ray_axis, P0, D, preclear):
    # P0 è [x,y,z] sul piano della griglia; origine O = P0 + D*preclear
    i,j,k = dims_for_ray(ray_axis)
    O = np.array(P0, dtype=float) + D * float(preclear + 0.02)

    # filtro bbox su coordinate trasversali (i,j). Richiede anche che il triangolo stia "davanti" lungo k
    pi, pj, pk = O[i], O[j], O[k]
    mask = (M[['xmin','ymin','zmin'][i]] <= pi) & (pi <= M[['xmax','ymax','zmax'][i]]) & \
           (M[['xmin','ymin','zmin'][j]] <= pj) & (pj <= M[['xmax','ymax','zmax'][j]]) & \
           ( (D[k] < 0) * (M[['xmin','ymin','zmin'][k]] <= pk)  |  (D[k] > 0) * (M[['xmax','ymax','zmax'][k]] >= pk) )
    idx = np.nonzero(mask)[0]
    if idx.size == 0: return None, None

    # Möller–Trumbore sui candidati
    best_t, best_i = None, None
    for ii in idx:
        v0,v1,v2 = M['V0'][ii], M['V1'][ii], M['V2'][ii]
        e1 = v1 - v0; e2 = v2 - v0; h = np.cross(D, e2); a = np.dot(e1, h)
        if abs(a) < 1e-9: continue
        f = 1.0/a; s = O - v0; u = f*np.dot(s, h)
        if u < 0.0 or u > 1.0: continue
        q = np.cross(s, e1); v = f*np.dot(D, q)
        if v < 0.0 or u + v > 1.0: continue
        t = f*np.dot(e2, q)
        if t <= 1e-9: continue
        if best_t is None or t < best_t: best_t, best_i = t, ii

    if best_t is None: return None, None
    P = O + best_t*D
    v0,v1,v2 = M['V0'][best_i], M['V1'][best_i], M['V2'][best_i]
    n = np.cross(v1-v0, v2-v0); n = n/(np.linalg.norm(n)+1e-12)
    # verso della normale: opposto al raggio (punta verso la tool)
    if np.dot(n, -D) < 0: n = -n
    return P, n

# -------------------- main --------------------
def main():
    rospy.init_node("raster_sweep_moveit_normals_anyray", anonymous=True)

    # Parametri
    cx = rospy.get_param("~cx", 0.00); cy = rospy.get_param("~cy", 0.42); cz = rospy.get_param("~cz", 0.68)
    dx = rospy.get_param("~dx", 0.18); dy = rospy.get_param("~dy", 0.24)
    row_step = rospy.get_param("~row_step", 0.02); line_step = rospy.get_param("~line_step", 0.006)
    eef_step = rospy.get_param("~eef_step", 0.01)
    preclear = rospy.get_param("~preclear_z", 0.20)  # distanza di partenza lungo il raggio
    vel_scale = rospy.get_param("~vel_scale", 0.20); avoid_collisions = bool(rospy.get_param("~avoid_collisions", True))
    pass2 = bool(rospy.get_param("~pass2", False))
    contact_offset = rospy.get_param("~contact_offset", 0.0)
    patient_object = rospy.get_param("~patient_object", "patient_back")
    tool_axis = rospy.get_param("~tool_axis", "-z")
    tilt_deg = rospy.get_param("~tilt_deg", 0.0)
    eef_link = rospy.get_param("~eef_link", "tool0")
    ray_axis = rospy.get_param("~ray_axis", "-z")   # "-z" (default), "-y", "+y", "-x", "+x"

    # MoveIt
    desc_ns = "/iiwa" if rospy.has_param("/iiwa/robot_description") else ""; ns="/iiwa"
    RobotCommander(ns=desc_ns); PlanningSceneInterface(ns=ns, synchronous=True)
    group = MoveGroupCommander("manipulator", ns=ns)
    group.set_pose_reference_frame("world"); group.set_end_effector_link(eef_link)
    group.set_max_velocity_scaling_factor(vel_scale); group.set_max_acceleration_scaling_factor(vel_scale)
    group.set_planning_time(20.0); group.allow_replanning(True)
    rospy.sleep(0.5); rospy.loginfo("EEF link usato: %s", eef_link)

    # Mesh in arrays + bbox
    M = load_mesh_arrays(ns, patient_object)
    if M is None:
        rospy.logerr("Mesh '%s' non trovata o vuota.", patient_object); return

    tool_axis_vec = axis_vector(tool_axis)
    D = axis_vector(ray_axis); D = D/(np.linalg.norm(D)+1e-12)
    i,j,k = dims_for_ray(ray_axis)
    rospy.loginfo("Ray axis: %s  (bbox sulle componenti %s,%s; avanzamento lungo %s)", ray_axis, "xyz"[i], "xyz"[j], "xyz"[k])

    def orientation_from_normal(n, row_dir):
        q_align = q_from_two_vectors(tool_axis_vec, n)
        if abs(tilt_deg) > 1e-6:
            rd = row_dir/(np.linalg.norm(row_dir)+1e-12); th=math.radians(tilt_deg)
            s=math.sin(th/2.0); c=math.cos(th/2.0)
            q_tilt=np.array([rd[0]*s, rd[1]*s, rd[2]*s, c])
            ax,ay,az,aw=q_align; bx,by,bz,bw=q_tilt
            return np.array([aw*bx+ax*bw+ay*bz-az*by,
                             aw*by-ax*bz+ay*bw+az*bx,
                             aw*bz+ax*by-ay*bx+az*bw,
                             aw*bw-ax*bx-ay*by-az*bz])
        return q_align

    # griglia (su XY come prima; il raggio può essere ±x/±y/±z)
    def gen_rows(orth=False):
        halfx, halfy = dx*0.5, dy*0.5
        ys = np.arange(cy - halfy, cy + halfy + 1e-9, row_step)
        x0, x1 = (cx - halfx, cx + halfx)
        for ri, y in enumerate(ys):
            if rospy.is_shutdown(): break
            forward = (ri % 2 == 0)
            xs = np.arange(x0, x1 + 1e-12, line_step)
            if not forward: xs = xs[::-1]
            row=[]; row_dir = np.array([1,0,0.0]) if not orth else np.array([0,1,0.0])
            for x in xs:
                P0 = np.array([x, y, cz], dtype=float)  # punto “centrale”; la quota è usata solo se il raggio ha componente z
                hit, n = ray_hit_normal_filtered(M, ray_axis, P0, D, preclear)
                if hit is None: continue
                q = orientation_from_normal(n, row_dir if forward else -row_dir)
                # offset LUNGO la normale, non lungo Z
                Pt = hit + n * float(contact_offset)
                row.append(pose_xyz_q(Pt[0], Pt[1], Pt[2], q))
            if row:
                yield ri, row

    # move-to-start: al primo punto valido risalgo indietro di preclear lungo -D
    first_found=False
    for _, row in gen_rows(False):
        if not row: continue
        first = row[0]
        pre = Pose(); pre.position.x, pre.position.y, pre.position.z = first.position.x, first.position.y, first.position.z
        pre.orientation = first.orientation
        pre.position.x += (-D[0]) * preclear
        pre.position.y += (-D[1]) * preclear
        pre.position.z += (-D[2]) * preclear
        group.set_start_state_to_current_state(); group.set_pose_target(pre, eef_link)
        if group.go(wait=True): first_found=True
        group.stop(); group.clear_pose_targets(); break
    if not first_found:
        rospy.logerr("Nessun punto valido trovato con ray=%s. Prova _ray_axis diverso o aumenta _preclear_z.", ray_axis)
        return

    def exec_row(poses, label):
        plan, frac = group.compute_cartesian_path(poses, eef_step, 0.0, avoid_collisions=avoid_collisions)
        if frac < 0.9:
            rospy.logwarn("[%s] fraction=%.2f -> fallback a punti singoli", label, frac)
            for p in poses:
                group.set_pose_target(p, eef_link)
                if not group.go(wait=True): group.stop(); group.clear_pose_targets(); return False
                group.stop(); group.clear_pose_targets()
            return True
        ok = group.execute(plan, wait=True); group.stop(); return ok

    for i,row in gen_rows(False):
        if rospy.is_shutdown(): break
        if not exec_row(row, "pass1_row%d"%i): rospy.logerr("[pass1_row%d] fallita.", i); break
    if pass2 and not rospy.is_shutdown():
        for i,row in gen_rows(True):
            if not exec_row(row, "pass2_row%d"%i): rospy.logerr("[pass2_row%d] fallita.", i); break
    rospy.loginfo("Raster sweep (any-ray + bbox) COMPLETATA.")

if __name__ == "__main__":
    main()

