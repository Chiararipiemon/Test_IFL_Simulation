import imfusion
import numpy as np
import csv
import os

# This script:
# - loads robot poses from a CSV (already expressed in ImFusion world frame),
# - builds a "Transducer Spline" from positions (x,y,z),
# - builds a "Direction Spline" by offsetting each point along a TILTED direction
#   computed from the quaternion (qx,qy,qz,qw) in the CSV (tool-local axis rotated to world).
#
# Works for:
# - Trajectory CSVs (many rows): position resampling + quaternion SLERP along arc-length
# - Single-pose CSV (one row): creates a tiny segment so a spline is valid

# ================= CONFIG =================

LABELMAP_PATH = r"/home/chiara_piemontese/Hybrid_simulation_last_try/segm_relabel_therightone.nii.gz"
CSV_PATH      = r"/home/chiara_piemontese/Hybrid_simulation_last_try/pose_from_robot_for_imfusion.csv"

N_FRAMES  = 100          # number of points in the final splines
CSV_UNITS = "mm"         # usually from Moveit! i have meters but this .csv is yet converted in mm for imfusion

Y_OFFSET_MM = 0.0        # optional global Y shift (mm)

DIRECTION_OFFSET_MM = 70.0   # distance (mm) from transducer spline to direction spline

SMOOTH_WINDOW = 15       # moving-average half-window for position smoothing (set 0 to disable)

# IMPORTANT:
# This is the tool-local axis that represents the "forward/beam" direction.
# Common choices: "+Z", "-Z", "+X", "-X", "+Y", "-Y" depending on your tool frame definition.
TOOL_FORWARD_AXIS_LOCAL = "+Z"

# If CSV has only one pose, we create a micro-segment of this length (mm) along the tool forward axis
STATIC_EPS_MM = 0.01

# ==========================================


def ensure_app():
    """Ensure imfusion.app exists (ConsoleController if run outside ImFusion)."""
    if not getattr(imfusion, "app", None):
        imfusion.app = imfusion.ConsoleController()


def dm():
    return imfusion.app.data_model


def am():
    """Return the current annotation model (must exist in an open ImFusion workspace)."""
    model = getattr(imfusion.app, "annotation_model", None)
    if model is None:
        raise RuntimeError(
            "annotation_model is None. Run this script inside ImFusion Python console with a workspace open."
        )
    return model


def get_all_annotations(model):
    """Return all annotations robustly across API versions."""
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
    """Remove old splines created by this script."""
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
        print(f"Removed {len(to_remove)} old spline(s).")


def get_or_load_segm():
    """Find or load the labelmap dataset 'segm_relabel'."""
    for d in dm():
        if "segm_relabel" in str(getattr(d, "name", "")):
            return d

    if not os.path.exists(LABELMAP_PATH):
        raise RuntimeError(f"Labelmap not found: {LABELMAP_PATH}")

    data_list = imfusion.load(LABELMAP_PATH)
    if not data_list:
        raise RuntimeError(f"Failed to load labelmap from: {LABELMAP_PATH}")

    d = dm().add(data_list[0])
    d.name = "segm_relabel"
    return d


def _axis_to_vec(axis_str):
    """Map '+Z','-Z','+Y','-Y','+X','-X' to a 3D unit vector."""
    ax = axis_str.upper()
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
        raise ValueError("Axis must be one of '+Z','-Z','+Y','-Y','+X','-X'")
    return v / np.linalg.norm(v)


# -------- Quaternion utilities (qx,qy,qz,qw) --------

def quat_normalize(q):
    """Normalize quaternion q = [x,y,z,w]."""
    q = np.asarray(q, float)
    n = np.linalg.norm(q)
    if n == 0:
        return np.array([0.0, 0.0, 0.0, 1.0], float)
    return q / n


def quat_slerp(q0, q1, t):
    """
    Spherical linear interpolation between q0 and q1 (both [x,y,z,w]).
    t in [0,1].
    """
    q0 = quat_normalize(q0)
    q1 = quat_normalize(q1)

    dot = float(np.dot(q0, q1))

    # If dot < 0, slerp the long way around. Flip one quaternion to take the short path.
    if dot < 0.0:
        q1 = -q1
        dot = -dot

    # If very close, use lerp to avoid numerical issues
    if dot > 0.9995:
        q = q0 + t * (q1 - q0)
        return quat_normalize(q)

    theta_0 = np.arccos(np.clip(dot, -1.0, 1.0))   # angle between q0 and q1
    sin_theta_0 = np.sin(theta_0)

    theta = theta_0 * t
    sin_theta = np.sin(theta)

    s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0

    return s0 * q0 + s1 * q1


def quat_rotate_vec(q, v):
    """
    Rotate vector v (3,) by quaternion q = [x,y,z,w].
    Assumes q represents rotation from tool-local frame to world frame.
    """
    q = quat_normalize(q)
    x, y, z, w = q
    q_xyz = np.array([x, y, z], float)
    v = np.asarray(v, float)

    # v' = v + 2*cross(q_xyz, cross(q_xyz, v) + w*v)
    t = 2.0 * np.cross(q_xyz, v)
    v_rot = v + w * t + np.cross(q_xyz, t)
    return v_rot


# -------- CSV loading --------

def load_poses(csv_path, units="mm"):
    """
    Load positions and quaternions from a CSV with header.
    Expected columns at least: x, y, z, qx, qy, qz, qw (in that order).
    Extra columns are ignored.

    Returns:
      centers: Nx3
      quats:   Nx4  (qx,qy,qz,qw)
    """
    centers = []
    quats = []

    with open(csv_path, "r") as f:
        r = csv.reader(f)
        _header = next(r, None)

        for row in r:
            if not row:
                continue
            try:
                vals = [float(v) for v in row]
            except ValueError:
                continue
            if len(vals) < 7:
                continue

            x, y, z = vals[0], vals[1], vals[2]
            qx, qy, qz, qw = vals[3], vals[4], vals[5], vals[6]

            centers.append([x, y, z])
            quats.append([qx, qy, qz, qw])

    if not centers:
        raise RuntimeError("No valid pose found in the CSV (need at least one row).")

    centers = np.asarray(centers, float)
    quats = np.asarray(quats, float)

    # Convert meters -> millimeters if needed
    if units.lower() == "m":
        centers *= 1000.0

    # Optional global Y shift
    centers[:, 1] += Y_OFFSET_MM

    # Normalize all quaternions
    quats = np.array([quat_normalize(q) for q in quats], float)

    return centers, quats


# -------- Smoothing & resampling --------

def smooth_moving_average(points, window):
    """Simple 3D moving-average along the sequence."""
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


def resample_positions_and_quats_along_curve(centers_raw, quats_raw, k):
    """
    Resample positions AND quaternions uniformly along arc-length.
    - Positions: linear interpolation on segments
    - Quaternions: SLERP with the same segment alpha

    Returns:
      centers_k: kx3
      quats_k:   kx4
    """
    centers_raw = np.asarray(centers_raw, float)
    quats_raw = np.asarray(quats_raw, float)

    if k <= 1:
        return np.repeat(centers_raw[:1], k, axis=0), np.repeat(quats_raw[:1], k, axis=0)

    if len(centers_raw) < 2:
        return np.repeat(centers_raw[:1], k, axis=0), np.repeat(quats_raw[:1], k, axis=0)

    seg_len = np.linalg.norm(np.diff(centers_raw, axis=0), axis=1)
    dist = np.concatenate(([0.0], np.cumsum(seg_len)))
    total = dist[-1]

    if total == 0.0:
        return np.repeat(centers_raw[:1], k, axis=0), np.repeat(quats_raw[:1], k, axis=0)

    targets = np.linspace(0.0, total, k)

    centers_out = []
    quats_out = []

    j = 0
    for td in targets:
        while j + 1 < len(dist) and dist[j + 1] < td:
            j += 1

        if j + 1 == len(dist):
            centers_out.append(centers_raw[-1])
            quats_out.append(quats_raw[-1])
        else:
            t0, t1 = dist[j], dist[j + 1]
            alpha = (td - t0) / (t1 - t0) if t1 > t0 else 0.0

            p = (1.0 - alpha) * centers_raw[j] + alpha * centers_raw[j + 1]
            q = quat_slerp(quats_raw[j], quats_raw[j + 1], alpha)

            centers_out.append(p)
            quats_out.append(q)

    return np.vstack(centers_out), np.vstack(quats_out)


def compute_splines_from_poses(centers_raw, quats_raw):
    """
    Build:
      - Transducer spline points (centers)
      - Direction spline points (centers offset along rotated tool axis)

    If only one pose exists:
      - duplicate pose
      - create a micro segment along the rotated tool-forward axis
    """
    centers_raw = np.asarray(centers_raw, float)
    quats_raw = np.asarray(quats_raw, float)

    tool_axis_local = _axis_to_vec(TOOL_FORWARD_AXIS_LOCAL)

    # Single-pose case -> create a tiny 2-point segment aligned with tool forward axis in WORLD
    if len(centers_raw) == 1:
        p0 = centers_raw[0]
        q0 = quats_raw[0]

        forward_world = quat_rotate_vec(q0, tool_axis_local)
        forward_world = forward_world / (np.linalg.norm(forward_world) + 1e-12)

        p1 = p0 + forward_world * STATIC_EPS_MM

        centers_raw = np.vstack([p0, p1])
        quats_raw = np.vstack([q0, q0])  # same orientation for both points

    # Optional position smoothing (orientation smoothing is not applied here)
    centers_smooth = smooth_moving_average(centers_raw, SMOOTH_WINDOW)

    # Resample positions + quaternions along arc-length
    centers, quats = resample_positions_and_quats_along_curve(centers_smooth, quats_raw, N_FRAMES)

    # Direction spline: per-point offset along rotated (tilted) tool axis
    dirs = []
    for p, q in zip(centers, quats):
        forward_world = quat_rotate_vec(q, tool_axis_local)
        n = np.linalg.norm(forward_world)
        if n > 0:
            forward_world = forward_world / n
        else:
            forward_world = np.array([0.0, 0.0, 1.0])

        dirs.append(p + forward_world * DIRECTION_OFFSET_MM)

    dirs = np.vstack(dirs)
    return centers, dirs


# -------- ImFusion spline creation --------

def get_spline_type():
    """Pick a spline-like annotation type supported by the current ImFusion version."""
    at = imfusion.Annotation.AnnotationType
    for name in ("SPLINE_3D", "SPLINE", "SMART_SPLINE", "POLY_LINE"):
        if hasattr(at, name):
            return getattr(at, name)
    raise RuntimeError("No supported spline annotation type found in ImFusion.Annotation.AnnotationType")


def create_spline(name, points, dataset):
    """Create (or replace) a spline annotation with the given name and 3D points."""
    model = am()
    spline_type = get_spline_type()

    # Remove existing annotation with same name
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

    # Try to attach to dataset (API varies)
    for attr in ("data", "dataset", "parent", "parent_dataset", "parentData"):
        if hasattr(ann, attr):
            try:
                setattr(ann, attr, dataset)
                break
            except Exception:
                pass

    print(f"{name}: {len(ann.points)} points (type={spline_type})")
    return ann


def main():
    ensure_app()
    delete_existing_splines()

    segm = get_or_load_segm()
    print("Loaded segm_relabel")

    centers_raw, quats_raw = load_poses(CSV_PATH, units=CSV_UNITS)
    print(f"Read {len(centers_raw)} pose(s) from CSV")

    centers, dirs = compute_splines_from_poses(centers_raw, quats_raw)

    create_spline("Transducer Spline", centers, segm)
    create_spline("Direction Spline",  dirs,    segm)

    print("OK: Transducer Spline + quaternion-based Direction Spline created.")


if __name__ == "__main__":
    main()
