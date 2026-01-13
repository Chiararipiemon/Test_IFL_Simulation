#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import csv
import numpy as np

import imfusion as imf
import imfusion.ultrasound as us

# ============================================================
# CONFIG
# ============================================================

CSV_PATH  = r"/home/chiararipiemo/Hybrid_simulation_last_try/pose_from_robot_for_imfusion.csv"
CSV_UNITS = "mm"   # "mm" or "m"

SWEEP_NAME   = "Sweep from CSV (quaternion driven, convex)"
SWEEP_TIME_S = 3.0

# Dummy image size (pixels)
IMG_W = 128
IMG_H = 128

# --------- Convex geometry params ----------
USE_CONVEX_GEOMETRY = True
PROBE_WIDTH_MM      = 38.0
DEPTH_MM            = 120.0
OPENING_ANGLE_DEG   = 60.0
LONG_RADIUS_MM      = 100.0
# ------------------------------------------

# Make the TOP of the image start at the probe tip:
# If the tracking pose is at the probe tip, the image center must be shifted by ~DEPTH/2 along the beam.
START_AT_PROBE_TIP = True

# If START_AT_PROBE_TIP=True, we override this with DEPTH_MM/2 automatically.
OFFSET_ALONG_BEAM_MM = 30.0

# Probe axes mapping (from RViz frame):
# - beam/depth is along +Z (blue) in your case
# - lateral choose +X (red) first; flip if mirrored
PROBE_DEPTH_AXIS_LOCAL   = np.array([0.0, 0.0, 1.0], dtype=float)  # beam = +Z (blue)
PROBE_LATERAL_AXIS_LOCAL = np.array([1.0, 0.0, 0.0], dtype=float)  # lateral = +X (red)

# Single-row CSV handling
DUPLICATE_SINGLE_POSE = True
DUPLICATE_SHIFT_MM    = 0.1


# ============================================================
# Helpers
# ============================================================

def ensure_app():
    if not getattr(imf, "app", None):
        imf.app = imf.ConsoleController()

def normalize(v):
    v = np.asarray(v, dtype=float)
    return v / (np.linalg.norm(v) + 1e-12)

def quat_to_rotmat(qx, qy, qz, qw):
    q = np.array([qx, qy, qz, qw], dtype=float)
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.eye(3, dtype=float)
    q /= n
    x, y, z, w = q

    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z

    R = np.array([
        [1.0 - 2.0*(yy + zz), 2.0*(xy - wz),       2.0*(xz + wy)],
        [2.0*(xy + wz),       1.0 - 2.0*(xx + zz), 2.0*(yz - wx)],
        [2.0*(xz - wy),       2.0*(yz + wx),       1.0 - 2.0*(xx + yy)],
    ], dtype=float)
    return R

def load_csv_poses(csv_path, units="mm"):
    if not os.path.exists(csv_path):
        raise RuntimeError("CSV not found: %s" % csv_path)

    out = []
    with open(csv_path, "r") as f:
        r = csv.reader(f)
        _ = next(r, None)  # header
        for row in r:
            if not row or len(row) < 7:
                continue
            try:
                vals = [float(v) for v in row[:7]]
            except ValueError:
                continue

            x, y, z, qx, qy, qz, qw = vals
            if units.lower() == "m":
                x, y, z = 1000.0*x, 1000.0*y, 1000.0*z

            R = quat_to_rotmat(qx, qy, qz, qw)
            p = np.array([x, y, z], dtype=float)
            out.append({"p": p, "R": R})

    if not out:
        raise RuntimeError("No valid poses found in CSV.")

    if len(out) == 1 and DUPLICATE_SINGLE_POSE:
        p0 = out[0]["p"].copy()
        R0 = out[0]["R"].copy()
        lateral_world = normalize(R0 @ PROBE_LATERAL_AXIS_LOCAL)
        p1 = p0 + lateral_world * float(DUPLICATE_SHIFT_MM)
        out.append({"p": p1, "R": R0})

    return out

def create_empty_image():
    # spacing just for visualization; geometry (convex) is carried by FrameGeometryMetadata if available
    img_desc = imf.ImageDescriptor(imf.PixelType.UBYTE, IMG_W, IMG_H, 1, 1)
    img_desc.spacing = np.array([0.5, 0.5, 1.0], dtype=float)
    img_desc.is_metric = True
    return imf.SharedImage(imf.MemImage(img_desc))

def remove_existing_by_name(name):
    to_remove = [d for d in imf.app.data_model if getattr(d, "name", "") == name]
    for d in to_remove:
        try:
            imf.app.data_model.remove(d)
        except Exception:
            pass

def get_components(obj):
    try:
        return list(obj.components)
    except Exception:
        return []

def add_component_best_effort(sweep, comp):
    for m in ("add_component", "addComponent", "add"):
        if hasattr(sweep, m):
            try:
                getattr(sweep, m)(comp)
                return True
            except Exception:
                pass
    # last resort: try append if components is a list
    try:
        sweep.components.append(comp)
        return True
    except Exception:
        return False

def get_or_create_frame_geometry_metadata(sweep):
    # 1) find existing
    for c in get_components(sweep):
        if isinstance(c, us.FrameGeometryMetadata):
            return c

    # 2) try to create and attach
    try:
        fgm = us.FrameGeometryMetadata()
    except Exception as e:
        print("Geometry: cannot construct FrameGeometryMetadata:", e)
        return None

    ok = add_component_best_effort(sweep, fgm)
    if not ok:
        print("Geometry: could not attach FrameGeometryMetadata to sweep (API mismatch).")
        return None

    return fgm

def configure_convex_geometry(sweep):
    fgm = get_or_create_frame_geometry_metadata(sweep)
    if fgm is None:
        print("Geometry: no FrameGeometryMetadata available on this sweep.")
        print("        You may need to set Convex geometry in GUI (Frame Geometry Properties).")
        return

    # get frame_geometry reference
    fg = None
    if hasattr(fgm, "frame_geometry"):
        fg = fgm.frame_geometry
    elif hasattr(fgm, "frameGeometry"):
        fg = fgm.frameGeometry

    if fg is None:
        print("Geometry: FrameGeometryMetadata has no frame_geometry (None).")
        print("        Set Convex in GUI: right click sweep -> Frame Geometry Properties.")
        return

    # If this fg already supports convex fields, set them
    set_any = False

    if hasattr(fg, "depth"):
        fg.depth = float(DEPTH_MM); set_any = True

    if hasattr(fg, "opening_angle"):
        fg.opening_angle = float(OPENING_ANGLE_DEG); set_any = True
    if hasattr(fg, "long_radius"):
        fg.long_radius = float(LONG_RADIUS_MM); set_any = True
    if hasattr(fg, "short_radius") and hasattr(fg, "opening_angle"):
        fg.short_radius = float(PROBE_WIDTH_MM) / (2.0 * np.sin(np.deg2rad(float(OPENING_ANGLE_DEG))))
        set_any = True

    # keep width if present
    if hasattr(fg, "width"):
        fg.width = float(PROBE_WIDTH_MM); set_any = True

    if hasattr(fg, "top_down"):
        fg.top_down = True; set_any = True

    if set_any:
        print("Geometry: convex parameters applied where supported.")
    else:
        print("Geometry: this frame_geometry does not expose convex fields in Python.")
        print("        Please set Convex in GUI: Frame Geometry Properties on the sweep.")

def build_image_pose(p_world, R_probe_to_world, offset_along_beam_mm):
    # image axes in world
    x_axis = normalize(R_probe_to_world @ PROBE_LATERAL_AXIS_LOCAL)  # image X
    y_axis = normalize(R_probe_to_world @ PROBE_DEPTH_AXIS_LOCAL)    # image Y (beam)
    z_axis = normalize(np.cross(x_axis, y_axis))
    x_axis = normalize(np.cross(y_axis, z_axis))

    T = np.eye(4, dtype=float)
    T[:3, 0] = x_axis
    T[:3, 1] = y_axis
    T[:3, 2] = z_axis

    # place image center so that top is at probe tip
    T[:3, 3] = p_world + y_axis * float(offset_along_beam_mm)
    return T


# ============================================================
# Main
# ============================================================

def main():
    ensure_app()

    poses = load_csv_poses(CSV_PATH, units=CSV_UNITS)
    n = len(poses)
    print("Loaded poses:", n)

    if n < 2:
        raise RuntimeError("Need at least 2 poses (enable DUPLICATE_SINGLE_POSE or provide more rows).")

    # choose offset so that the TOP of the image starts at the probe tip
    offset_mm = float(OFFSET_ALONG_BEAM_MM)
    if START_AT_PROBE_TIP:
        offset_mm = 0.5 * float(DEPTH_MM)  # key change

    remove_existing_by_name(SWEEP_NAME)

    sweep = us.UltrasoundSweep()
    sweep.name = SWEEP_NAME

    tracking = imf.TrackingSequence()
    dt = float(SWEEP_TIME_S) / float(n - 1)

    for i in range(n):
        t = i * dt
        p = poses[i]["p"]
        R = poses[i]["R"]

        T_img = build_image_pose(p, R, offset_mm)
        tracking.add(T_img, t, 1.0)

        sweep.add(create_empty_image())
        try:
            sweep.set_timestamp(t, sweep.size - 1)
        except Exception:
            pass

    sweep.add_tracking(tracking)

    try:
        sweep.properties.set_param("topDown", True)
    except Exception:
        pass

    if USE_CONVEX_GEOMETRY:
        configure_convex_geometry(sweep)

    imf.app.data_model.add(sweep)

    print("OK: sweep added:", SWEEP_NAME)
    print("Offset along beam (mm):", offset_mm)
    print("If the image is mirrored, flip PROBE_LATERAL_AXIS_LOCAL sign.")


if __name__ == "__main__":
    main()

