#!/usr/bin/env python3

import csv
import os
import numpy as np
# codice per registrare tracking da ambiente Moveit! a ambiente Imfusion
# ========= CONFIG =========

# CSV originale 
# Deve contenere per ogni riga: ... x, y, z, qx, qy, qz, qw
CSV_IN  = "/home/chiararipiemo/iiwa_csv/us_poses_1762856006_no_vertical_tail.csv"

# CSV di output compatibile ImFusion 
CSV_OUT = "/home/chiararipiemo/iiwa_stack_ws/src/iiwa_probe_utils/Hybrid_simulation/us_poses_1762856006_imfusion_frame.csv"

# Unità del CSV:
INPUT_UNITS = "mm"

# Trasformazione dal frame robot al frame ImFusion (in mm)
T_IMFUSION_FROM_ROBOT = np.array([
    [1.0, 0.0, 0.0, -642.0],
    [0.0, 0.0, 1.0, -364.0],  
    [0.0, -1.0, 0.0, -200.0],
    [0.0, 0.0, 0.0,    1.0]
], dtype=float)

# ==========================


def quat_to_rot(qx, qy, qz, qw):
    """Quaternion (x,y,z,w) -> matrice di rotazione 3x3."""
    q = np.array([qx, qy, qz, qw], dtype=float)
    n = np.linalg.norm(q)
    if n == 0:
        return np.eye(3)
    q /= n
    x, y, z, w = q
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)]
    ], dtype=float)


def rot_to_quat(R):
    """Matrice di rotazione 3x3 -> quaternione (x, y, z, w)."""
    m00, m01, m02 = R[0]
    m10, m11, m12 = R[1]
    m20, m21, m22 = R[2]
    tr = m00 + m11 + m22

    if tr > 0.0:
        S = np.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S
        qx = (m21 - m12) / S
        qy = (m02 - m20) / S
        qz = (m10 - m01) / S
    elif (m00 > m11) and (m00 > m22):
        S = np.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw = (m21 - m12) / S
        qx = 0.25 * S
        qy = (m01 + m10) / S
        qz = (m02 + m20) / S
    elif m11 > m22:
        S = np.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw = (m02 - m20) / S
        qx = (m01 + m10) / S
        qy = 0.25 * S
        qz = (m12 + m21) / S
    else:
        S = np.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw = (m10 - m01) / S
        qx = (m02 + m20) / S
        qy = (m12 + m21) / S
        qz = 0.25 * S

    q = np.array([qx, qy, qz, qw], dtype=float)
    n = np.linalg.norm(q)
    if n == 0:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    return q / n


def load_poses(csv_path):
    """
    Legge il CSV originale.
    Supporta formati:
      - [..., x, y, z, qx, qy, qz, qw]
      - [x, y, z, qx, qy, qz, qw]
    Restituisce lista di matrici 4x4 nelle unità originali.
    """
    if not os.path.exists(csv_path):
        raise FileNotFoundError(f"File di input non trovato: {csv_path}")

    poses = []
    with open(csv_path, "r") as f:
        r = csv.reader(f)
        header = next(r, None)

        def row_to_pose(row):
            vals = [float(v) for v in row]
            if len(vals) < 7:
                return None
            x, y, z, qx, qy, qz, qw = vals[-7:]
            R = quat_to_rot(qx, qy, qz, qw)
            T = np.eye(4, dtype=float)
            T[:3, :3] = R
            T[:3, 3] = [x, y, z]
            return T

        # Se la prima riga è numerica, è già una pose
        if header:
            try:
                _ = [float(v) for v in header]
                pose = row_to_pose(header)
                if pose is not None:
                    poses.append(pose)
            except ValueError:
                # header testuale, lo ignoriamo
                pass

        for row in r:
            if not row:
                continue
            try:
                pose = row_to_pose(row)
            except ValueError:
                continue
            if pose is not None:
                poses.append(pose)

    if not poses:
        raise RuntimeError("Nessuna pose valida trovata nel CSV di input")

    return poses


def convert_to_imfusion_frame(poses):
    """
    Applica T_IMFUSION_FROM_ROBOT.
    Restituisce lista di 4x4 nel frame ImFusion.
    Se INPUT_UNITS == "m", converte le traslazioni in mm.
    """
    T_conv = T_IMFUSION_FROM_ROBOT.copy()
    out = []
    for T in poses:
        # dal frame robot al frame ImFusion
        T_if = T_conv @ T

        # se input in metri → porta la traslazione in mm
        if INPUT_UNITS.lower() == "m":
            T_if = T_if.copy()
            T_if[:3, 3] *= 1000.0

        out.append(T_if)
    return out


def save_imfusion_csv(poses_if, csv_path):
    """
    Salva il CSV per ImFusion nel formato:
      x_mm, y_mm, z_mm, qx, qy, qz, qw
    dove le pose sono nel frame ImFusion.
    """
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["x_mm", "y_mm", "z_mm", "qx", "qy", "qz", "qw"])
        for T in poses_if:
            x, y, z = T[:3, 3]
            R = T[:3, :3]
            qx, qy, qz, qw = rot_to_quat(R)
            w.writerow([x, y, z, qx, qy, qz, qw])


def main():
    poses_robot = load_poses(CSV_IN)
    poses_if = convert_to_imfusion_frame(poses_robot)
    save_imfusion_csv(poses_if, CSV_OUT)
    print(f"Convertite {len(poses_if)} pose in frame ImFusion (x,y,z,qx,qy,qz,qw) → {CSV_OUT}")


if __name__ == "__main__":
    main()

