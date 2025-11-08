import imfusion
import numpy as np
import csv
import os
# questo codice importa la csv ricavata da moveit, la trasla verso il basso di 94 mm come ultimo affinamento della registrazione, la alliscia per evitare i zig e zg e genera due spline: transducer e direction spline. Queste spline sono delle polyline che devono essere successivamente modificate in GlSpline nell'XML del workspace.
# ================= CONFIG =================

LABELMAP_PATH = r"/home/chiararipiemo/iiwa_stack_ws/src/iiwa_probe_utils/Hybrid_simulation/segm_relabel.nii.gz"
CSV_PATH      = r"/home/chiararipiemo/iiwa_stack_ws/src/iiwa_probe_utils/Hybrid_simulation/us_poses_1762452771_imfusion_frame.csv"

N_FRAMES            = 100       # numero di punti per le spline
CSV_UNITS           = "mm"      # "mm" se il CSV è già in millimetri
Y_OFFSET_MM         = -94.0     # shift globale lungo Y
DIRECTION_OFFSET_MM = 60.0      # quanto traslare la Direction Spline rispetto alla Transducer

SMOOTH_WINDOW       = 15        # per allisciare la traiettoria ed evitare che sia a zig e zag

# Direzione "verso il basso" nel SISTEMA MONDO
# Cambia qui se vedi che va nel verso opposto.
DIRECTION_WORLD_AXIS = "-Y"     # scegli tra: "+Z", "-Z", "+Y", "-Y", "+X", "-X"

# ==========================================


def ensure_app():
    if not getattr(imfusion, "app", None):
        imfusion.app = imfusion.ConsoleController()


def dm():
    return imfusion.app.data_model


def am():
    return imfusion.app.annotation_model


def get_all_annotations(model):
    if not hasattr(model, "annotations"):
        return []
    anns = model.annotations
    if callable(anns):
        try:
            return list(anns())
        except TypeError:
            return []
    try:
        return list(anns)
    except TypeError:
        return []


def delete_existing_splines():
    model = am()
    to_remove = [
        a for a in get_all_annotations(model)
        if getattr(a, "name", "") in ("Transducer Spline", "Direction Spline")
    ]
    for a in to_remove:
        if hasattr(model, "remove_annotation"):
            try:
                model.remove_annotation(a)
                continue
            except Exception:
                pass
        if hasattr(model, "remove"):
            try:
                model.remove(a)
            except Exception:
                pass
    if to_remove:
        print(f"Rimosse {len(to_remove)} spline vecchie.")


def get_or_load_segm():
    for d in dm():
        if "segm_relabel" in str(getattr(d, "name", "")):
            return d

    if not os.path.exists(LABELMAP_PATH):
        raise RuntimeError(f"Labelmap non trovata: {LABELMAP_PATH}")

    data_list = imfusion.load(LABELMAP_PATH)
    if not data_list:
        raise RuntimeError(f"Impossibile caricare labelmap da {LABELMAP_PATH}")

    d = dm().add(data_list[0])
    d.name = "segm_relabel"
    return d


def load_centers(csv_path, units="mm"):
    """Legge solo (x,y,z) dal CSV (con header), applica Y_OFFSET."""
    centers = []
    with open(csv_path, "r") as f:
        r = csv.reader(f)
        next(r, None)  # header
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
            centers.append([x, y, z])

    if not centers:
        raise RuntimeError("Nessuna posizione valida trovata nel CSV")

    centers = np.asarray(centers, float)

    if units.lower() == "m":
        centers *= 1000.0

    centers[:, 1] += Y_OFFSET_MM
    return centers


def smooth_moving_average(points, window):
    """Smoothing semplice lungo la traiettoria."""
    pts = np.asarray(points, float)
    N = len(pts)
    if N < 3 or window <= 0:
        return pts.copy()
    out = np.zeros_like(pts)
    for i in range(N):
        i0 = max(0, i - window)
        i1 = min(N, i + window + 1)
        out[i] = pts[i0:i1].mean(axis=0)
    return out


def resample_along_curve(points, k):
    """Ricampionamento uniforme lungo la lunghezza della curva."""
    pts = np.asarray(points, float)
    if k <= 1 or len(pts) < 2:
        return np.repeat(pts[:1], k, axis=0)

    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    dist = np.concatenate(([0.0], np.cumsum(seg)))
    total = dist[-1]

    if total == 0:
        return np.repeat(pts[:1], k, axis=0)

    targets = np.linspace(0.0, total, k)
    out = []
    j = 0
    for td in targets:
        while j + 1 < len(dist) and dist[j + 1] < td:
            j += 1
        if j + 1 == len(dist):
            out.append(pts[-1])
        else:
            t0, t1 = dist[j], dist[j + 1]
            alpha = (td - t0) / (t1 - t0) if t1 > t0 else 0.0
            p = (1 - alpha) * pts[j] + alpha * pts[j + 1]
            out.append(p)
    return np.vstack(out)


def get_direction_vector():
    """Restituisce il vettore mondo lungo cui traslare la direction spline."""
    ax = DIRECTION_WORLD_AXIS.upper()
    if ax == "+Z":
        v = np.array([0.0, 0.0, 1.0])
    elif ax == "-Z":
        v = np.array([0.0, 0.0, -1.0])
    elif ax == "+Y":
        v = np.array([0.0, 1.0, 0.0])
    elif ax == "-Y":
        v = np.array([0.0, -1.0, 0.0])
    elif ax == "+X":
        v = np.array([1.0, 0.0, 0.0])
    elif ax == "-X":
        v = np.array([-1.0, 0.0, 0.0])
    else:
        raise ValueError("DIRECTION_WORLD_AXIS deve essere uno tra ±X, ±Y, ±Z")
    return v / np.linalg.norm(v)


def compute_splines(centers_raw):
    """
    Transducer Spline:
      - centers_raw lisciati con moving-average
      - ricampionati in N_FRAMES

    Direction Spline:
      - stessi punti, traslati di DIRECTION_OFFSET_MM lungo DIRECTION_WORLD_AXIS
      - stessa forma, solo spostata.
    """
    smooth = smooth_moving_average(centers_raw, SMOOTH_WINDOW)
    centers = resample_along_curve(smooth, N_FRAMES)

    dir_vec = get_direction_vector()
    dirs = centers + dir_vec * DIRECTION_OFFSET_MM

    return centers, dirs


def get_spline_type():
    at = imfusion.Annotation.AnnotationType
    for name in ("SPLINE_3D", "SPLINE", "SMART_SPLINE", "POLY_LINE"):
        if hasattr(at, name):
            return getattr(at, name)
    raise RuntimeError("Nessun tipo spline disponibile")


def create_spline(name, points, dataset):
    model = am()
    spline_type = get_spline_type()

    # rimuovi eventuale spline esistente con lo stesso nome
    for a in get_all_annotations(model):
        if getattr(a, "name", "") == name:
            if hasattr(model, "remove_annotation"):
                try:
                    model.remove_annotation(a)
                    continue
                except Exception:
                    pass
            if hasattr(model, "remove"):
                try:
                    model.remove(a)
                except Exception:
                    pass

    ann = model.create_annotation(spline_type)
    ann.name = name
    ann.points = [tuple(map(float, p)) for p in points]

    # prova ad associarla al dataset (se non basta, fai "Move to" a mano)
    for attr in ("data", "dataset", "parent", "parent_dataset", "parentData"):
        if hasattr(ann, attr):
            try:
                setattr(ann, attr, dataset)
                break
            except Exception:
                pass

    print(f"{name}: {len(ann.points)} punti (tipo={spline_type})")
    return ann


def main():
    ensure_app()
    delete_existing_splines()

    segm = get_or_load_segm()
    print("Caricata segm_relabel")

    centers_raw = load_centers(CSV_PATH, units=CSV_UNITS)
    centers, dirs = compute_splines(centers_raw)

    create_spline("Transducer Spline", centers, segm)
    create_spline("Direction Spline",  dirs,    segm)

    print("OK: Transducer Spline liscia + Direction Spline uguale ma traslata verso il basso.")


if __name__ == "__main__":
    main()

