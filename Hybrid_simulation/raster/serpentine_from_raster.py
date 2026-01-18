#!/usr/bin/env python3
"""Convert a noisy zig-zag raster pose CSV into an *ideal smooth serpentine*.

Target shape (like your sketch): straight parallel passes connected by smooth U-turns.

Input CSV columns required:
  x,y,z,qx,qy,qz,qw

What it does:
  1) Estimates the raster plane & axes via PCA (u = scan direction, v = step-over).
  2) Segments the original path into passes by detecting sign changes of du.
  3) For each pass, estimates its v-level (median v), endpoints (u_start/u_end), and w (median).
  4) Rebuilds an ideal path: straight lines + semicircular U-turns of radius |dv|/2.
  5) Preserves orientation progression by SLERP over original arclength.

Usage example:
  python3 serpentine_from_raster.py \
    --input  /home/chiararipiemo/iiwa_csv/us_poses_1768554200.csv \
    --output /home/chiararipiemo/iiwa_stack_ws/src/iiwa_probe_utils/Hybrid_simulation/raster/us_poses_1768554200_serpentine.csv

Notes:
  - If segmentation is too nervous: increase --sign-window or --min-pass-points.
  - If you get too many points: increase --spacing or reduce --max-points.
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.signal import savgol_filter
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp


COLS = ["x", "y", "z", "qx", "qy", "qz", "qw"]


@dataclass
class Pass:
    start: int
    end: int
    direction: int  # +1 (u increasing) or -1 (u decreasing)
    u0: float
    u1: float
    v: float
    w: float


def _unit(v: np.ndarray, eps: float = 1e-12) -> np.ndarray:
    n = np.linalg.norm(v)
    return v / n if n > eps else np.zeros_like(v)


def _dedup_consecutive(xyz: np.ndarray, quat: np.ndarray, eps: float = 1e-9):
    if len(xyz) <= 1:
        return xyz, quat
    d = np.linalg.norm(xyz[1:] - xyz[:-1], axis=1)
    keep = np.ones(len(xyz), dtype=bool)
    keep[1:] = d > eps
    return xyz[keep], quat[keep]


def _cumulative_arclength(p: np.ndarray) -> np.ndarray:
    if len(p) == 0:
        return np.array([], dtype=float)
    d = np.linalg.norm(np.diff(p, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(d)])


def _pca_frame(xyz: np.ndarray):
    """Return (mean, basis) where basis columns are e1,e2,e3."""
    mu = xyz.mean(axis=0)
    X = xyz - mu
    C = np.cov(X.T)
    vals, vecs = np.linalg.eigh(C)
    order = np.argsort(vals)[::-1]
    vecs = vecs[:, order]

    e1 = _unit(vecs[:, 0])
    e2 = _unit(vecs[:, 1])
    e3 = _unit(np.cross(e1, e2))
    e2 = _unit(np.cross(e3, e1))

    B = np.column_stack([e1, e2, e3])
    return mu, B


def _project(mu: np.ndarray, B: np.ndarray, xyz: np.ndarray) -> np.ndarray:
    return (xyz - mu) @ B


def _unproject(mu: np.ndarray, B: np.ndarray, uvw: np.ndarray) -> np.ndarray:
    return uvw @ B.T + mu


def _smooth_1d(x: np.ndarray, window: int, poly: int) -> np.ndarray:
    if window <= 0:
        return x
    if window % 2 == 0:
        window += 1
    if len(x) < window:
        return x
    return savgol_filter(x, window_length=window, polyorder=poly, mode="interp")


def _segment_passes(u: np.ndarray, sign_window: int, min_pass_points: int):
    """Segment by robust sign changes of du."""
    if len(u) < 4:
        return [slice(0, len(u))]

    us = _smooth_1d(u, window=sign_window, poly=2) if sign_window >= 5 else u
    du = np.diff(us)
    sgn = np.sign(du)

    # replace zeros with previous non-zero
    for i in range(len(sgn)):
        if sgn[i] == 0:
            sgn[i] = sgn[i - 1] if i > 0 else 0
    for i in range(len(sgn) - 1, -1, -1):
        if sgn[i] == 0:
            sgn[i] = sgn[i + 1] if i < len(sgn) - 1 else 0

    # turn indices where sign flips
    flips = np.where((sgn[1:] != 0) & (sgn[:-1] != 0) & (sgn[1:] != sgn[:-1]))[0] + 1

    # build segments
    cuts = [0] + flips.tolist() + [len(u) - 1]
    segs = []
    for a, b in zip(cuts[:-1], cuts[1:]):
        segs.append((a, b))

    # merge short segments
    merged = []
    i = 0
    while i < len(segs):
        a, b = segs[i]
        if (b - a + 1) < min_pass_points and i > 0:
            # merge into previous
            pa, pb = merged[-1]
            merged[-1] = (pa, b)
        else:
            merged.append((a, b))
        i += 1

    # convert to slices
    return [slice(a, b + 1) for a, b in merged if b > a]


def _estimate_passes(uvw: np.ndarray, sign_window: int, min_pass_points: int) -> list[Pass]:
    u, v, w = uvw[:, 0], uvw[:, 1], uvw[:, 2]
    seg_slices = _segment_passes(u, sign_window=sign_window, min_pass_points=min_pass_points)

    passes: list[Pass] = []
    for sl in seg_slices:
        uu = u[sl]
        vv = v[sl]
        ww = w[sl]
        if len(uu) < 2:
            continue
        dirn = 1 if (uu[-1] - uu[0]) >= 0 else -1
        passes.append(
            Pass(
                start=sl.start,
                end=sl.stop - 1,
                direction=dirn,
                u0=float(uu[0]),
                u1=float(uu[-1]),
                v=float(np.median(vv)),
                w=float(np.median(ww)),
            )
        )

    # ensure passes are in traversal order and v is mostly monotone; keep order but smooth v levels
    if len(passes) >= 3:
        v_levels = np.array([p.v for p in passes], dtype=float)
        v_smooth = _smooth_1d(v_levels, window=min(9, len(v_levels) // 2 * 2 + 1), poly=2)
        for i, p in enumerate(passes):
            p.v = float(v_smooth[i])

    return passes


def _median_step(uvw: np.ndarray) -> float:
    d = np.linalg.norm(np.diff(uvw[:, :2], axis=0), axis=1)
    d = d[d > 1e-12]
    return float(np.median(d)) if len(d) else 0.005


def _build_ideal_serpentine(
    passes: list[Pass],
    spacing: float,
    arc_min_points: int,
) -> np.ndarray:
    """Build ideal path in (u,v,w) coordinates."""
    if not passes:
        return np.zeros((0, 3), dtype=float)

    out = []

    # global edges in u (helps decide whether a turn bulges to +u or -u)
    u_all: list[float] = []
    for pp in passes:
        u_all.extend([pp.u0, pp.u1])
    u_min = float(np.min(u_all))
    u_max = float(np.max(u_all))

    for i, p in enumerate(passes):
        u0, u1, v0, w0 = p.u0, p.u1, p.v, p.w

        # straight segment
        L = abs(u1 - u0)
        n = max(2, int(math.floor(L / spacing)) + 1)
        uu = np.linspace(u0, u1, n)
        vv = np.full_like(uu, v0)
        ww = np.full_like(uu, w0)

        seg = np.column_stack([uu, vv, ww])
        if out:
            seg = seg[1:]  # avoid duplicate point
        out.append(seg)

        # u-turn to next pass
        if i < len(passes) - 1:
            p_next = passes[i + 1]
            v1 = p_next.v
            w1 = p_next.w
            dv = v1 - v0
            if abs(dv) < 1e-12:
                continue

            Rv = abs(dv) / 2.0
            v_mid = (v0 + v1) / 2.0

            # turn side depends on which edge we are at (u_end)
            u_end = u1

            # choose bulge direction: if we're closer to the max-u edge -> bulge +u, else bulge -u
            side = 1.0 if abs(u_end - u_max) <= abs(u_end - u_min) else -1.0

            arc_len = math.pi * Rv
            m = max(arc_min_points, int(math.floor(arc_len / spacing)) + 1)

            # parameter theta from +90 to -90 gives endpoints at u=u_end, v=v0 and v1
            if dv > 0:
                theta = np.linspace(math.pi / 2, -math.pi / 2, m)
            else:
                theta = np.linspace(-math.pi / 2, math.pi / 2, m)

            uu_arc = u_end + side * (Rv * np.cos(theta))
            vv_arc = v_mid + (Rv * np.sin(theta))
            ww_arc = np.linspace(w0, w1, m)

            arc = np.column_stack([uu_arc, vv_arc, ww_arc])
            out.append(arc[1:])  # avoid duplicate

    return np.vstack(out)


def _resample_uniform(p: np.ndarray, spacing: float, max_points: int) -> tuple[np.ndarray, np.ndarray]:
    """Resample positions at ~uniform spacing; returns (p_new, s_new)."""
    if len(p) < 2:
        return p, np.zeros(len(p))

    s = _cumulative_arclength(p)
    total = s[-1]
    if total < 1e-12:
        return p, s

    # number of points from requested spacing
    n = int(math.floor(total / spacing)) + 1
    n = max(2, n)
    if n > max_points:
        n = max_points
        spacing = total / (n - 1)

    s_new = np.linspace(0.0, total, n)

    # linear interpolation (more than enough; path is already smooth)
    p_new = np.empty((n, 3), dtype=float)
    for k in range(3):
        p_new[:, k] = np.interp(s_new, s, p[:, k])

    return p_new, s_new


def _interp_quat_by_arclength(xyz_orig: np.ndarray, quat_orig: np.ndarray, s_new: np.ndarray) -> np.ndarray:
    s0 = _cumulative_arclength(xyz_orig)
    if len(s0) < 2:
        return np.tile(quat_orig[:1], (len(s_new), 1))

    # make s strictly increasing
    ds = np.diff(s0)
    keep = np.ones(len(s0), dtype=bool)
    keep[1:] = ds > 1e-12
    s0 = s0[keep]
    quat = quat_orig[keep]

    rot = R.from_quat(quat)
    slerp = Slerp(s0, rot)

    s_new_clip = np.clip(s_new, s0[0], s0[-1])
    return slerp(s_new_clip).as_quat()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--input",
        default="/home/chiararipiemo/iiwa_csv/us_poses_1768554200.csv",
        help="Input CSV path (x,y,z,qx,qy,qz,qw).",
    )
    ap.add_argument(
        "--output",
        default="/home/chiararipiemo/iiwa_stack_ws/src/iiwa_probe_utils/Hybrid_simulation/raster/us_poses_1768554200_serpentine.csv",
        help="Output CSV path.",
    )
    ap.add_argument(
        "--spacing",
        type=float,
        default=None,
        help="Desired point spacing in meters (or your units). Default: max(median_step, --min-spacing).",
    )
    ap.add_argument(
        "--min-spacing",
        type=float,
        default=0.002,
        help="Lower bound for default spacing (default 0.002).",
    )
    ap.add_argument(
        "--max-points",
        type=int,
        default=200000,
        help="Hard cap on output points (default 200000).",
    )
    ap.add_argument(
        "--sign-window",
        type=int,
        default=11,
        help="Odd window for smoothing u before sign detection (default 11).",
    )
    ap.add_argument(
        "--min-pass-points",
        type=int,
        default=30,
        help="Minimum points per pass when segmenting (default 30).",
    )
    ap.add_argument(
        "--arc-min-points",
        type=int,
        default=40,
        help="Minimum points per U-turn arc (default 40).",
    )
    args = ap.parse_args()

    in_path = Path(args.input)
    if not in_path.exists():
        raise FileNotFoundError(f"Input not found: {in_path}")

    df = pd.read_csv(in_path)
    missing = [c for c in COLS if c not in df.columns]
    if missing:
        raise ValueError(f"CSV missing required columns: {missing}. Found: {list(df.columns)}")

    xyz = df[["x", "y", "z"]].to_numpy(dtype=float)
    quat = df[["qx", "qy", "qz", "qw"]].to_numpy(dtype=float)

    # normalize quaternions
    qn = np.linalg.norm(quat, axis=1, keepdims=True)
    qn[qn < 1e-12] = 1.0
    quat = quat / qn

    xyz, quat = _dedup_consecutive(xyz, quat)

    mu, B = _pca_frame(xyz)
    uvw = _project(mu, B, xyz)

    passes = _estimate_passes(uvw, sign_window=int(args.sign_window), min_pass_points=int(args.min_pass_points))

    med = _median_step(uvw)
    spacing = float(args.spacing) if args.spacing is not None else max(med, float(args.min_spacing))

    uvw_ideal = _build_ideal_serpentine(
        passes,
        spacing=spacing,
        arc_min_points=int(args.arc_min_points),
    )

    # resample (also enforces max_points)
    uvw_ideal, s_new = _resample_uniform(uvw_ideal, spacing=spacing, max_points=int(args.max_points))

    xyz_ideal = _unproject(mu, B, uvw_ideal)
    quat_ideal = _interp_quat_by_arclength(xyz_orig=xyz, quat_orig=quat, s_new=s_new)

    out_df = pd.DataFrame(
        np.column_stack([xyz_ideal, quat_ideal]),
        columns=COLS,
    )

    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_df.to_csv(out_path, index=False)

    print(f"Wrote: {out_path}")
    print(f"  input points (dedup): {len(xyz)}")
    print(f"  output points:       {len(out_df)}")
    print(f"  spacing:             {spacing:g}")
    print(f"  passes detected:     {len(passes)}")


if __name__ == "__main__":
    main()
