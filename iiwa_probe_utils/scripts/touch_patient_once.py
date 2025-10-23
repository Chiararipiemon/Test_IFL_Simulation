#!/usr/bin/env python3
import os, sys, math, struct, rospy
from geometry_msgs.msg import Pose
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, roscpp_initialize, roscpp_shutdown

# --- util ---
def vec_sub(a,b): return (a[0]-b[0], a[1]-b[1], a[2]-b[2])
def vec_dot(a,b): return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]
def vec_cross(a,b): return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
def vec_len(a): return math.sqrt(vec_dot(a,a))
def vec_norm(a):
    l = vec_len(a)
    return (0,0,0) if l<1e-12 else (a[0]/l,a[1]/l,a[2]/l)

def quat_from_matrix(R):
    m00,m01,m02 = R[0]; m10,m11,m12 = R[1]; m20,m21,m22 = R[2]
    tr = m00+m11+m22
    if tr>0:
        S = math.sqrt(tr+1.0)*2
        qw=0.25*S; qx=(m21-m12)/S; qy=(m02-m20)/S; qz=(m10-m01)/S
    elif (m00>m11) and (m00>m22):
        S=math.sqrt(1.0+m00-m11-m22)*2
        qw=(m21-m12)/S; qx=0.25*S; qy=(m01+m10)/S; qz=(m02+m20)/S
    elif m11>m22:
        S=math.sqrt(1.0+m11-m00-m22)*2
        qw=(m02-m20)/S; qx=(m01+m10)/S; qy=0.25*S; qz=(m12+m21)/S
    else:
        S=math.sqrt(1.0+m22-m00-m11)*2
        qw=(m10-m01)/S; qx=(m02+m20)/S; qy=(m12+m21)/S; qz=0.25*S
    return (qx,qy,qz,qw)

def rot_from_quat(q):
    x,y,z,w=q; xx,yy,zz=x*x,y*y,z*z; xy,xz,yz=x*y,x*z,y*z; wx,wy,wz=w*x,w*y,w*z
    X=(1-2*(yy+zz), 2*(xy+wz),    2*(xz-wy))
    Y=(2*(xy-wz),   1-2*(xx+zz),  2*(yz+wx))
    Z=(2*(xz+wy),   2*(yz-wx),    1-2*(xx+yy))
    return (X,Y,Z)

def read_stl(path):
    tris=[]
    def is_bin():
        try:
            with open(path,'rb') as f:
                hdr=f.read(80); 
                if len(hdr)<80: return False
                n=int.from_bytes(f.read(4),'little'); 
                return os.path.getsize(path)==84+n*50
        except: return False
    if is_bin():
        with open(path,'rb') as f:
            f.seek(80); n=int.from_bytes(f.read(4),'little')
            for _ in range(n):
                rec=f.read(50); if len(rec)<50: break
                vals=struct.unpack('<12fH', rec)
                v0=(vals[3],vals[4],vals[5]); v1=(vals[6],vals[7],vals[8]); v2=(vals[9],vals[10],vals[11])
                tris.append((v0,v1,v2))
    else:
        with open(path,'r',errors='ignore') as f:
            cur=[]
            for line in f:
                t=line.strip().split()
                if len(t)>=4 and t[0].lower()=='vertex':
                    cur.append((float(t[1]),float(t[2]),float(t[3])))
                    if len(cur)==3: tris.append(tuple(cur)); cur=[]
    return tris

def apply_transform_tris(tris, scale, q, t):
    R=rot_from_quat(q); out=[]
    for (a,b,c) in tris:
        vs=[]
        for v in (a,b,c):
            x=v[0]*scale[0]; y=v[1]*scale[1]; z=v[2]*scale[2]
            rx=R[0][0]*x+R[0][1]*y+R[0][2]*z
            ry=R[1][0]*x+R[1][1]*y+R[1][2]*z
            rz=R[2][0]*x+R[2][1]*y+R[2][2]*z
            vs.append((rx+t[0],ry+t[1],rz+t[2]))
        out.append(tuple(vs))
    return out

def ray_intersect_down_from_xy(x,y,z_high,tris):
    # ray dir d=(0,0,-1)
    ox,oy,oz=x,y,z_high; d=(0,0,-1)
    best=None; hit=None; nrm=None
    for (v0,v1,v2) in tris:
        e1=vec_sub(v1,v0); e2=vec_sub(v2,v0)
        p=vec_cross(d,e2); det=vec_dot(e1,p)
        if abs(det)<1e-12: continue
        inv=1.0/det
        tvec=(ox-v0[0], oy-v0[1], oz-v0[2])
        u=vec_dot(tvec,p)*inv
        if u<0 or u>1: continue
        q=vec_cross(tvec,e1)
        v=vec_dot(d,q)*inv
        if v<0 or u+v>1: continue
        t=vec_dot(e2,q)*inv
        if t<=0: continue
        if best is None or t<best:
            best=t
            hit=(ox,oy,oz+d[2]*t)
            n=vec_norm(vec_cross(e1,e2))
            if n[2]<0: n=(-n[0],-n[1],-n[2])
            nrm=n
    return hit,nrm

def main():
    roscpp_initialize(sys.argv)
    rospy.init_node("touch_patient_once", anonymous=True)

    # --- parametri ---
    object_name = rospy.get_param("~object_name","patient_surface")
    mesh_path   = rospy.get_param("~mesh_path","")  # se vuoto, prova a leggerlo dai param di add_patient_mesh_on_table
    sx = float(rospy.get_param("~scale_x",1.0))
    sy = float(rospy.get_param("~scale_y",1.0))
    sz = float(rospy.get_param("~scale_z",1.0))

    tip_offset  = float(rospy.get_param("~tip_offset", 0.12))  # distanza EE->punta lungo asse del probe
    gap         = float(rospy.get_param("~gap", 0.001))        # piccolo distacco per evitare collisione dura (0 per contatto)
    approach    = float(rospy.get_param("~approach", 0.05))    # avvicinamento iniziale
    roll_hint   = rospy.get_param("~roll_hint", "+X")          # per definire l'asse X locale (solo estetica)
    # il tuo holder punta lungo -Z dell'EE (z:=-0.12), quindi la direzione punta = -Z
    probe_axis  = rospy.get_param("~probe_axis", "-Z")

    ns = rospy.get_namespace().rstrip('/') or "iiwa"
    scene = PlanningSceneInterface(ns="/"+ns)
    group = MoveGroupCommander("manipulator", ns="/"+ns)
    group.set_pose_reference_frame("world")
    group.set_end_effector_link("iiwa_link_ee")
    group.set_planning_time(20.0)
    group.set_max_velocity_scaling_factor(0.2)

    rospy.sleep(0.5)
    poses = scene.get_object_poses([object_name])
    if object_name not in poses:
        rospy.logerr("Oggetto '%s' non trovato nella PlanningScene.", object_name); sys.exit(1)
    obj = poses[object_name]
    q_obj=(obj.orientation.x,obj.orientation.y,obj.orientation.z,obj.orientation.w)
    t_obj=(obj.position.x,obj.position.y,obj.position.z)

    # Mesh path fallback dai param dell'add_patient_mesh_on_table
    if not mesh_path:
        for root in ["/{}/add_patient_mesh_on_table".format(ns), "/add_patient_mesh_on_table", "~"]:
            try:
                mesh_path = rospy.get_param(root+"/mesh_path")
                sx=float(rospy.get_param(root+"/scale_x",1.0))
                sy=float(rospy.get_param(root+"/scale_y",1.0))
                sz=float(rospy.get_param(root+"/scale_z",1.0))
                break
            except KeyError:
                pass
    if not mesh_path:
        rospy.logerr("~mesh_path mancante: passa _mesh_path:=/percorso/file.stl"); sys.exit(1)

    tris_local=read_stl(mesh_path)
    if not tris_local:
        rospy.logerr("STL vuoto o non leggibile: %s", mesh_path); sys.exit(1)
    tris_world=apply_transform_tris(tris_local,(sx,sy,sz),q_obj,t_obj)

    # bounding box per scegliere il centro
    minx=miny=  1e9; maxx=maxy=-1e9; maxz=-1e9
    for (a,b,c) in tris_world:
        for (x,y,z) in (a,b,c):
            minx=min(minx,x); maxx=max(maxx,x)
            miny=min(miny,y); maxy=max(maxy,y); maxz=max(maxz,z)
    cx=(minx+maxx)/2.0; cy=(miny+maxy)/2.0
    z_high=maxz+0.5

    # intersezione verticale in (cx,cy)
    pt, n = ray_intersect_down_from_xy(cx, cy, z_high, tris_world)
    if not pt:
        rospy.logerr("Nessuna intersezione trovata al centro bbox."); sys.exit(1)

    # direzione della PUNTA del probe in world (EE local -Z => punta = -n)
    tip_dir_world = (-n[0], -n[1], -n[2]) if probe_axis=="-Z" else (n[0],n[1],n[2])
    # posa EE: metti l'origine a distanza tip_offset lungo tip_dir_world "indietro" dal punto di contatto
    pos = (pt[0] - tip_dir_world[0]*tip_offset - n[0]*gap,
           pt[1] - tip_dir_world[1]*tip_offset - n[1]*gap,
           pt[2] - tip_dir_world[2]*tip_offset - n[2]*gap)

    # orientamento: vogliamo +Z_ee = +n se probe_axis = -Z (così -Z_ee punta verso la pelle)
    if probe_axis=="-Z":
        z_axis = ( n[0],  n[1],  n[2])
    else:  # +Z
        z_axis = (-n[0], -n[1], -n[2])

    # roll hint (per definire X): proiettiamo il mondo +X o +Y sul piano tangente
    hint = (1,0,0) if roll_hint.upper()=="+X" else (0,1,0)
    dot_h = vec_dot(hint, z_axis)
    proj  = (hint[0]-dot_h*z_axis[0], hint[1]-dot_h*z_axis[1], hint[2]-dot_h*z_axis[2])
    x_axis = vec_norm(proj) if vec_len(proj)>1e-6 else (0,1,0)
    y_axis = vec_cross(z_axis, x_axis)

    R=(x_axis, y_axis, z_axis)
    q=quat_from_matrix(R)

    # approach sopra lungo +n
    approach_pos = (pos[0]+n[0]*approach, pos[1]+n[1]*approach, pos[2]+n[2]*approach)

    # vai all'approach
    pose = Pose(); pose.position.x,pose.position.y,pose.position.z = approach_pos
    pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w = q
    group.set_pose_target(pose)
    ok = group.go(wait=True); group.stop(); group.clear_pose_targets()
    if not ok:
        rospy.logwarn("Pianificazione approach fallita.")

    # breve discesa cartesiana fino al contatto
    wps=[]
    p2 = Pose(); p2.position.x,p2.position.y,p2.position.z = pos
    p2.orientation.x,p2.orientation.y,p2.orientation.z,p2.orientation.w = q
    wps.append(pose); wps.append(p2)
    try:
        (traj,frac)=group.compute_cartesian_path(wps, 0.003, True)
    except TypeError:
        (traj,frac)=group.compute_cartesian_path(wps, 0.003, 0.0)
    rospy.loginfo("Cartesian descend fraction=%.3f", frac)
    if traj and len(traj.joint_trajectory.points)>0:
        group.execute(traj, wait=True)
    else:
        rospy.logwarn("Nessuna traiettoria cartesiana generata; provo a fare un go() diretto al punto.")
        group.set_pose_target(p2); group.go(wait=True); group.stop(); group.clear_pose_targets()

    rospy.loginfo("Touch pose eseguita.")
    roscpp_shutdown()

if __name__ == "__main__":
    main()
