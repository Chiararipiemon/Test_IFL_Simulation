#!/usr/bin/env python3
import os
import sys
import struct
import math
import rospy
from geometry_msgs.msg import PoseStamped
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown

def is_binary_stl(path):
    with open(path, 'rb') as f:
        header = f.read(80)
        if len(header) < 80:
            return False
        n_tri_bytes = f.read(4)
        if len(n_tri_bytes) < 4:
            return False
        n_tris = struct.unpack('<I', n_tri_bytes)[0]
        expected = 84 + n_tris * 50
        try:
            size = os.path.getsize(path)
            return size == expected
        except OSError:
            return False

def read_stl_vertices(path):
    verts = []
    if is_binary_stl(path):
        with open(path, 'rb') as f:
            f.seek(80)
            n_tris = struct.unpack('<I', f.read(4))[0]
            for _ in range(n_tris):
                # normal(3f) + v1(3f) + v2(3f) + v3(3f) + attr(2B)
                data = f.read(50)
                if len(data) < 50:
                    break
                # skip normal (3 floats)
                v = struct.unpack('<12fH', data)  # 12 floats + ushort
                # vertices start at indices 3..11
                v1 = (v[3], v[4], v[5])
                v2 = (v[6], v[7], v[8])
                v3 = (v[9], v[10], v[11])
                verts.extend([v1, v2, v3])
    else:
        with open(path, 'r', errors='ignore') as f:
            for line in f:
                line = line.strip()
                if line.lower().startswith('vertex'):
                    parts = line.split()
                    if len(parts) >= 4:
                        try:
                            x = float(parts[1]); y = float(parts[2]); z = float(parts[3])
                            verts.append((x,y,z))
                        except ValueError:
                            pass
    return verts

def rot_matrix_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    # R = Rz * Ry * Rx
    Rz = ((cy, -sy, 0.0),
          (sy,  cy, 0.0),
          (0.0, 0.0, 1.0))
    Ry = ((cp, 0.0, sp),
          (0.0, 1.0, 0.0),
          (-sp,0.0, cp))
    Rx = ((1.0, 0.0, 0.0),
          (0.0, cr, -sr),
          (0.0, sr,  cr))
    # Multiply Rz * Ry * Rx
    def matmul(A, B):
        return tuple(tuple(sum(A[i][k]*B[k][j] for k in range(3)) for j in range(3)) for i in range(3))
    return matmul(matmul(Rz, Ry), Rx)

def quat_from_rpy(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5); sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5);  sr = math.sin(roll * 0.5)
    qw = cr*cp*cy + sr*sp*sy
    qx = sr*cp*cy - cr*sp*sy
    qy = cr*sp*cy + sr*cp*sy
    qz = cr*cp*sy - sr*sp*cy
    return (qx, qy, qz, qw)

def apply_transform(verts, R, scale):
    sx, sy, sz = scale
    out = []
    for (x,y,z) in verts:
        x *= sx; y *= sy; z *= sz
        tx = R[0][0]*x + R[0][1]*y + R[0][2]*z
        ty = R[1][0]*x + R[1][1]*y + R[1][2]*z
        tz = R[2][0]*x + R[2][1]*y + R[2][2]*z
        out.append((tx,ty,tz))
    return out

def main():
    roscpp_initialize(sys.argv)
    rospy.init_node("add_patient_mesh_on_table", anonymous=True)

    mesh_path = rospy.get_param("~mesh_path")
    frame_id  = rospy.get_param("~frame_id", "world")
    moveit_ns = rospy.get_param("~moveit_ns", "")

    # position in XY (center of mesh origin)
    x = float(rospy.get_param("~x", 0.90))
    y = float(rospy.get_param("~y", 0.00))

    # orientation in degrees
    roll_deg  = float(rospy.get_param("~roll_deg", 90.0))
    pitch_deg = float(rospy.get_param("~pitch_deg", 0.0))
    yaw_deg   = float(rospy.get_param("~yaw_deg", 90.0))
    z_lift    = float(rospy.get_param("~z_lift", 0.01))  # meters, small lift to avoid visual interpenetration
    align_mesh_yaw_with_table = bool(rospy.get_param("~align_mesh_yaw_with_table", False))

    # uniform or anisotropic scale
    sx = float(rospy.get_param("~scale_x", 1.0))
    sy = float(rospy.get_param("~scale_y", 1.0))
    sz = float(rospy.get_param("~scale_z", 1.0))

    # Read environment params to align with table
    pedestal_size    = float(rospy.get_param("~pedestal_size", 0.60))
    pedestal_z_top   = float(rospy.get_param("~pedestal_z_top", 0.0))
    table_height     = float(rospy.get_param("~table_height", 0.75))
    table_align_top  = bool(rospy.get_param("~table_align_with_pedestal_top", True))

    # Compute table top Z
    if table_align_top:
        table_top_z = pedestal_z_top
    else:
        floor_z = pedestal_z_top - pedestal_size
        table_top_z = floor_z + table_height

    # Optionally align mesh yaw with table_yaw (convert rad->deg)
    if align_mesh_yaw_with_table:
        table_yaw = float(rospy.get_param("~table_yaw", 0.0))
        yaw_deg = math.degrees(table_yaw)

    # Compute lowest Z of mesh after scale+rotation (around origin)
    verts = read_stl_vertices(mesh_path)
    if not verts:
        rospy.logerr("No vertices read from STL: %s", mesh_path)
        sys.exit(1)

    roll  = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw   = math.radians(yaw_deg)
    R = rot_matrix_rpy(roll, pitch, yaw)
    tv = apply_transform(verts, R, (sx, sy, sz))
    min_z = min(v[2] for v in tv)

    # Place so the lowest point sits on table top
    z = table_top_z - min_z + z_lift

    # Pose
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = frame_id
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    qx,qy,qz,qw = quat_from_rpy(roll, pitch, yaw)
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw

    scene = PlanningSceneInterface(ns=moveit_ns) if moveit_ns else PlanningSceneInterface()
    rospy.sleep(1.0)

    name = rospy.get_param("~name", "patient_surface")
    size = (sx, sy, sz)  # MoveIt uses this as scale for add_mesh
    rospy.loginfo("Adding mesh '%s' at (%.3f, %.3f, %.3f) with RPY_deg=(%.1f, %.1f, %.1f) scale=%s",
                  name, x, y, z, roll_deg, pitch_deg, yaw_deg, size)
    scene.add_mesh(name, pose, mesh_path, size=size)

    # confirm
    for _ in range(50):
        if name in scene.get_known_object_names():
            rospy.loginfo("Confirmed in scene: %s", name)
            break
        rospy.sleep(0.2)

    rospy.spin()
    roscpp_shutdown()

if __name__ == "__main__":
    main()
