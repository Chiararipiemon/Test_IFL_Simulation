#!/usr/bin/env python3

import imfusion as imf
import numpy as np
import csv

# Path del CSV con le posizioni già nel frame ImFusion (x_mm, y_mm, z_mm, qx, qy, qz, qw)
CSV_PATH = r"/home/chiararipiemo/iiwa_stack_ws/src/iiwa_probe_utils/Hybrid_simulation/us_poses_1762452771_imfusion_frame_quat.csv"

# Finestra per lo smoothing (numero di punti vicini considerati per la media)
SMOOTH_WINDOW = 15


def load_positions_from_csv():
    """
    Legge le posizioni (x, y, z) dal CSV.
    Si aspetta almeno le prime 3 colonne come coordinate in mm.
    """
    positions = []

    with open(CSV_PATH, "r") as f:
        r = csv.reader(f)
        next(r, None)  # salta header
        for row in r:
            if not row:
                continue
            try:
                vals = [float(v) for v in row]
            except ValueError:
                continue
            if len(vals) < 3:
                continue
            x, y, z = vals[0], vals[1], vals[2]
            positions.append([x, y, z])

    if not positions:
        print("Nessuna posizione valida trovata nel CSV")
        return None

    # Tracking già in mm
    return np.asarray(positions, dtype=float)


def smooth_positions(pts, window):
    """
    Applica uno smoothing semplice con media mobile sulle posizioni.
    """
    N = len(pts)
    if N < 3 or window <= 0:
        return pts.copy()

    out = np.zeros_like(pts)
    for i in range(N):
        i0 = max(0, i - window)
        i1 = min(N, i + window + 1)
        out[i] = pts[i0:i1].mean(axis=0)
    return out


def create_smooth_tracking():
    """
    Crea una TrackingSequence ImFusion usando le posizioni smoothate.
    """
    pts = load_positions_from_csv()
    if pts is None:
        return

    pts_s = smooth_positions(pts, SMOOTH_WINDOW)

    ts = imf.TrackingSequence()
    t = 0.0
    dt = 0.01  # tempo fittizio

    for p in pts_s:
        M = np.eye(4, dtype=float)
        M[:3, 3] = p
        ts.add(M, t, 1.0)
        t += dt

    if ts.size == 0:
        print("Tracking vuota, qualcosa non va")
        return

    ts.name = "Tracking Smooth from CSV"
    imf.app.data_model.add(ts)
    print("TrackingSequence creata:", ts.name, "size =", ts.size)


create_smooth_tracking()
