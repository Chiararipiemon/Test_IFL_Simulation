import imfusion as imf
import numpy as np
import csv
# questo è il path per il csv gà precedentemente registrato per allinearlo ai frame della segmentazione e della cloudpoint. Successivamente però ha bisogno di essere "abbassato" di altri 94 mm per allinearsi sulla schiena del paziente

CSV_PATH = r"/home/chiararipiemo/iiwa_stack_ws/src/iiwa_probe_utils/Hybrid_simulation/us_poses_1762452771_imfusion_frame.csv"
Y_OFFSET_MM = -94.0      # offset verso il basso
SMOOTH_WINDOW = 15       # per migliorare la traiettoria e allisciarla come avvine nel metodo 1

def quat_to_rot(qx, qy, qz, qw):
    q = np.array([qx, qy, qz, qw], dtype=float)
    n = np.linalg.norm(q)
    if n == 0:
        return np.eye(3, dtype=float)
    q /= n
    x, y, z, w = q
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),       2*(x*z + y*w)],
        [2*(x*y + z*w),         1 - 2*(x*x + z*z),   2*(y*z - x*w)],
        [2*(x*z - y*w),         2*(y*z + x*w),       1 - 2*(x*x + y*y)]
    ], dtype=float)

def load_positions_from_csv():
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
    pts = np.asarray(positions, dtype=float)
    # tracking già in mm -> se fosse in metri qui moltiplicheremmo per 1000
    pts[:,1] += Y_OFFSET_MM
    return pts

def smooth_positions(pts, window):
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
    pts = load_positions_from_csv()
    if pts is None:
        return
    pts_s = smooth_positions(pts, SMOOTH_WINDOW)

    ts = imf.TrackingSequence()
    t = 0.0
    dt = 0.01   # tempo fittizio, non critico

    for p in pts_s:
        M = np.eye(4, dtype=float)
        M[:3,3] = p
        ts.add(M, t, 1.0)
        t += dt

    if ts.size == 0:
        print("Tracking vuota, qualcosa non va")
        return

    ts.name = "Tracking Smooth from CSV"
    imf.app.data_model.add(ts)
    print("TrackingSequence creata:", ts.name, "size =", ts.size)

create_smooth_tracking()
