import imfusion
import numpy as np
import csv
import os

# Questo script:
# - importa la tracking dal CSV (già registrata correttamente in ImFusion frame),
# - la liscia per togliere zig-zag,
# - la ricampiona in N_FRAMES punti = Transducer Spline,
# - genera la Direction Spline come copia traslata della Transducer Spline.
#
# Le spline create sono annotazioni in ImFusion, da usare/convertire poi in GlSpline nell'XML.

# ================= CONFIG =================

LABELMAP_PATH = r"/home/chiararipiemo/iiwa_stack_ws/src/iiwa_probe_utils/Hybrid_simulation/segm_relabel.nii.gz"
CSV_PATH      = r"/home/chiararipiemo/iiwa_stack_ws/src/iiwa_probe_utils/Hybrid_simulation/us_poses_1762856006_imfusion_frame.csv"

N_FRAMES            = 100       # numero di punti per le spline (ricampionamento lungo la curva)
CSV_UNITS           = "mm"      # "mm" se il CSV è già in millimetri, "m" se in metri

# Offset globale lungo Y (da usare solo se vuoi micro-aggiustare la tracking).
# Per default nessuno shift: la tracking è considerata già corretta.
Y_OFFSET_MM         = 0.0

# Offset per la Direction Spline rispetto alla Transducer Spline
DIRECTION_OFFSET_MM = 70.0      # distanza lungo DIRECTION_WORLD_AXIS

# Finestra per il moving-average (in numero di campioni) per lisciare la tracking
SMOOTH_WINDOW       = 15

# Direzione lungo cui traslare la Direction Spline nel sistema mondo di ImFusion
# Scegli tra: "+Z", "-Z", "+Y", "-Y", "+X", "-X"
DIRECTION_WORLD_AXIS = "-Y"

# ==========================================


def ensure_app():
    """Garantisce che imfusion.app esista (se eseguito fuori da ImFusion Suite)."""
    if not getattr(imfusion, "app", None):
        imfusion.app = imfusion.ConsoleController()


def dm():
    return imfusion.app.data_model


def am():
    """Ritorna l'annotation_model corrente."""
    model = getattr(imfusion.app, "annotation_model", None)
    if model is None:
        raise RuntimeError(
            "annotation_model è None. "
            "Esegui questo script nella Python console di ImFusion con un workspace aperto."
        )
    return model


def get_all_annotations(model):
    """Restituisce tutte le annotazioni dell'annotation model."""
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
    """Rimuove eventuali spline precedenti con i nomi usati da questo script."""
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
    """Recupera o carica la labelmap 'segm_relabel'."""
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
    """
    Legge solo (x,y,z) dal CSV (con header),
    converte in mm se necessario,
    applica eventuale offset globale lungo Y (Y_OFFSET_MM).
    """
    centers = []
    with open(csv_path, "r") as f:
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
            centers.append([x, y, z])

    if not centers:
        raise RuntimeError("Nessuna posizione valida trovata nel CSV")

    centers = np.asarray(centers, float)

    # Se il CSV è in metri, converti in mm
    if units.lower() == "m":
        centers *= 1000.0

    # Offset globale opzionale lungo Y
    centers[:, 1] += Y_OFFSET_MM

    return centers


def smooth_moving_average(points, window):
    """Applica un semplice moving-average 3D lungo la traiettoria."""
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
    """
    Ricampionamento uniforme lungo la lunghezza della curva.
    Ritorna k punti equispaziati lungo la polilinea definita da 'points'.
    """
    pts = np.asarray(points, float)
    if k <= 1 or len(pts) < 2:
        return np.repeat(pts[:1], k, axis=0)

    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    dist = np.concatenate(([0.0], np.cumsum(seg)))
    total = dist[-1]

    if total == 0.0:
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
            p = (1.0 - alpha) * pts[j] + alpha * pts[j + 1]
            out.append(p)
    return np.vstack(out)


def get_direction_vector():
    """Restituisce il versore lungo DIRECTION_WORLD_AXIS."""
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
        raise ValueError("DIRECTION_WORLD_AXIS deve essere uno tra '+Z', '-Z', '+Y', '-Y', '+X', '-X'")
    return v / np.linalg.norm(v)


def compute_splines(centers_raw):
    """
    Transducer Spline:
      - parte direttamente dalla tracking del CSV (già registrata correttamente),
      - smoothing con moving-average,
      - ricampionamento uniforme in N_FRAMES.
      Nessuno shift ulteriore.

    Direction Spline:
      - stessi punti della Transducer Spline,
      - traslati di DIRECTION_OFFSET_MM lungo DIRECTION_WORLD_AXIS.
    """
    # smoothing della tracking originale
    smooth = smooth_moving_average(centers_raw, SMOOTH_WINDOW)

    # ricampionamento in N_FRAMES punti lungo la curva smussata
    centers = resample_along_curve(smooth, N_FRAMES)

    # direction spline: offset parallelo
    dir_vec = get_direction_vector()
    dirs = centers + dir_vec * DIRECTION_OFFSET_MM

    return centers, dirs


def get_spline_type():
    """Sceglie un tipo spline supportato da ImFusion."""
    at = imfusion.Annotation.AnnotationType
    for name in ("SPLINE_3D", "SPLINE", "SMART_SPLINE", "POLY_LINE"):
        if hasattr(at, name):
            return getattr(at, name)
    raise RuntimeError("Nessun tipo spline disponibile nelle AnnotationType di ImFusion")


def create_spline(name, points, dataset):
    """Crea (o sostituisce) una spline con il nome dato e i punti specificati."""
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

    # prova ad associarla al dataset
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

    print("OK: Transducer Spline liscia dalla tracking + Direction Spline parallela traslata.")


if __name__ == "__main__":
    main()

