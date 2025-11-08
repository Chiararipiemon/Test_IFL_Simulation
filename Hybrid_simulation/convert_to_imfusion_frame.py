import csv
import numpy as np

# ========= CONFIG =========

# CSV originale (pose nel frame robot / mondo sorgente)
CSV_IN  = r"/home/chiararipiemo/iiwa_stack_ws/src/iiwa_probe_utils/Hybrid_simulation/us_poses_1762452771.csv"

# CSV di output compatibile ImFusion (quello che hai chiamato *_imfusion_frame.csv)
CSV_OUT = r"/home/chiararipiemo/iiwa_stack_ws/src/iiwa_probe_utils/Hybrid_simulation/us_poses_1762452771_imfusion_frame.csv"

# Unità del CSV originale
# - "m" se le posizioni sono in metri
# - "mm" se sono già in millimetri
INPUT_UNITS = "m"

# Matrice 4x4: trasformazione dal frame del CSV al frame ImFusion.
# Questa è la T_imfusion_from_robot che avevi usato (QUI DEVI METTERE I TUOI VALORI).
# Di default identità (nessuna trasformazione).
T_IMFUSION_FROM_ROBOT = np.array([
    [1.0, 0.0, 0.0, 0.0],   # R e t in unità del CSV (prima di conversione mm)
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
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


def load_poses(csv_path):
    """
    Legge il CSV originale.
    Supporta formati:
      - [..., x, y, z, qx, qy, qz, qw]
      - [x, y, z, qx, qy, qz, qw]
    Restituisce lista di 4x4 in unità originali.
    """
    poses = []
    with open(csv_path, "r") as f:
        r = csv.reader(f)
        header = next(r, None)

        def row_to_pose(row):
            vals = [float(v) for v in row]
            # prendi gli ultimi 7 valori come [x,y,z,qx,qy,qz,qw]
            if len(vals) < 7:
                return None
            x, y, z, qx, qy, qz, qw = vals[-7:]
            R = quat_to_rot(qx, qy, qz, qw)
            T = np.eye(4, dtype=float)
            T[:3, :3] = R
            T[:3, 3] = [x, y, z]
            return T

        # se la prima riga non è numerica, è header → ok, partiamo dalla seconda
        if header:
            try:
                _ = [float(v) for v in header]
                # prima riga è già dati
                pose = row_to_pose(header)
                if pose is not None:
                    poses.append(pose)
            except ValueError:
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
    Applica T_IMFUSION_FROM_ROBOT e converte in millimetri.
    Restituisce lista di 4x4 in mm nel frame ImFusion.
    """
    T_conv = T_IMFUSION_FROM_ROBOT.copy()
    out = []
    for T in poses:
        # dal frame robot al frame ImFusion
        T_if = T_conv @ T

        # conversione unità: se input in metri → porta la traslazione in mm
        if INPUT_UNITS.lower() == "m":
            T_if = T_if.copy()
            T_if[:3, 3] *= 1000.0

        out.append(T_if)
    return out


def save_imfusion_csv(poses_if, csv_path):
    """
    Salva il CSV per ImFusion.
    Qui usiamo formato semplice:
      x_mm, y_mm, z_mm, r00, r01, ..., r22
    (16 valori: la 4x4 row-major; l'ultima riga è 0 0 0 1)
    """
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        # header (facoltativo)
        w.writerow([
            "x_mm", "y_mm", "z_mm",
            "r00", "r01", "r02",
            "r10", "r11", "r12",
            "r20", "r21", "r22",
            "t30", "t31", "t32", "t33"
        ])
        for T in poses_if:
            x, y, z = T[:3, 3]
            R = T[:3, :3].reshape(-1)
            row = [x, y, z] + list(R) + list(T[3, :])
            w.writerow(row)


def main():
    poses_robot = load_poses(CSV_IN)
    poses_if = convert_to_imfusion_frame(poses_robot)
    save_imfusion_csv(poses_if, CSV_OUT)
    print(f"Convertite {len(poses_if)} pose in frame ImFusion → {CSV_OUT}")


if __name__ == "__main__":
    main()
