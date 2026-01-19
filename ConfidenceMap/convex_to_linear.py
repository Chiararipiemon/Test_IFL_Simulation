"""
convex_to_linear.py
===================

This module provides a function to convert ultrasound images stored
with a convex frame geometry into a linear frame geometry.  The
mapping is implemented using basic trigonometric relationships
between the fan‑shaped acquisition geometry and the desired rectangular
representation.  The code expects the user to supply the geometry
parameters for both the input (convex) and output (linear) frames.

The motivation for this script stems from the way ImFusion models
ultrasound frame geometries.  Convex geometries describe a ring
sector bounded by a short and long radius and centred around a
virtual apex; the transducer offset lies at the intersection of the
short radius and the vertical symmetry axis.  In
contrast, a linear frame geometry is defined by a width and a depth,
with the offset at the centre of the transducer array and the
steering angle controlling any lateral tilt.

Note that this script relies on the `imfusion` Python package to
read/write ImFusion datasets and to create frame geometry objects.
Refer to the ImFusion SDK documentation for details.  The numerical
resampling is done with NumPy only, which makes the code independent
of any proprietary GPU functionality.  Bilinear interpolation is
implemented manually for clarity.

"""



from __future__ import annotations

import math
from typing import Dict, Tuple

import numpy as np

try:
    import imfusion
except ImportError:
    raise RuntimeError(
        "The imfusion package is required to run this script. "
        "Install it from the ImFusion SDK or run inside ImFusionSuite."
    )


def _bilinear_sample(img: np.ndarray, x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """Sample the 2D image `img` at floating point coordinates (x, y)."""
    h, w = img.shape
    x0 = np.floor(x).astype(int)
    x1 = x0 + 1
    y0 = np.floor(y).astype(int)
    y1 = y0 + 1

    # Clip coordinates to image bounds for safe indexing
    x0_clip = np.clip(x0, 0, w - 1)
    x1_clip = np.clip(x1, 0, w - 1)
    y0_clip = np.clip(y0, 0, h - 1)
    y1_clip = np.clip(y1, 0, h - 1)

    Ia = img[y0_clip, x0_clip]
    Ib = img[y1_clip, x0_clip]
    Ic = img[y0_clip, x1_clip]
    Id = img[y1_clip, x1_clip]

    wa = (x1 - x) * (y1 - y)
    wb = (x1 - x) * (y - y0)
    wc = (x - x0) * (y1 - y)
    wd = (x - x0) * (y - y0)

    out = wa * Ia + wb * Ib + wc * Ic + wd * Id

    # opzionale: metti a zero i punti davvero fuori bounds (non solo clippati)
    inside = (x >= 0) & (x <= w - 1) & (y >= 0) & (y <= h - 1)
    out = np.where(inside, out, 0)

    return out


def _convex_to_linear_frame(
    frame: np.ndarray,
    apex: Tuple[float, float],
    opening_angle_rad: float,
    r0: float,
    r1: float,
    out_width: int,
    out_depth: int,
) -> np.ndarray:
    """Convert a single 2D ultrasound frame from convex to linear geometry."""
    apex_x, apex_y = apex

    phi = np.linspace(-opening_angle_rad, opening_angle_rad, out_width)
    r = np.linspace(r0, r1, out_depth)

    phi_grid, r_grid = np.meshgrid(phi, r, indexing="xy")

    x_src = apex_x + r_grid * np.sin(phi_grid)
    y_src = apex_y + r_grid * np.cos(phi_grid)

    return _bilinear_sample(frame, x_src, y_src)


def convert_convex_us_sweep(
    input_path: str,
    output_path: str,
    convex_params: Dict[str, float],
    linear_params: Dict[str, float],
) -> None:
    """Convert an ultrasound sweep from convex to linear geometry."""
    imageset, *_ = imfusion.load(input_path)

    origin_x, origin_y = convex_params["origin"]
    opening_angle_deg = float(convex_params["opening_angle_deg"])
    opening_angle_rad = math.radians(opening_angle_deg)
    short_radius = float(convex_params["short_radius_px"])
    long_radius = float(convex_params["long_radius_px"])

    # ATTENZIONE: a seconda della convenzione del tuo dataset, potrebbe essere + oppure -
    apex_x = origin_x
    apex_y = origin_y + short_radius

    out_origin_x, out_origin_y = linear_params["origin"]
    out_width = int(round(linear_params["width_px"]))
    out_depth = int(round(linear_params["depth_px"]))
    steering_angle_deg = float(linear_params["steering_angle_deg"])

    converted_images = []

    for idx in range(len(imageset)):
        img = imageset[idx]
        arr = np.array(img).squeeze()

        resampled = _convex_to_linear_frame(
            arr,
            apex=(apex_x, apex_y),
            opening_angle_rad=opening_angle_rad,
            r0=short_radius,
            r1=long_radius,
            out_width=out_width,
            out_depth=out_depth,
        )

        new_img = imfusion.SharedImage(resampled.astype(arr.dtype))

        # copia spacing originale (se presente)
        try:
            sp = img.spacing()
            new_img.setSpacing(sp[0], sp[1], sp[2])
        except Exception:
            pass

        # Costruisci geometria lineare (pixel)
        coord_sys = imfusion.US.FrameGeometry.CoordinateSystem.Pixels
        geom = imfusion.US.FrameGeometryLinear(coord_sys)
        geom.setWidth(float(linear_params["width_px"]))
        geom.setDepth(float(linear_params["depth_px"]))
        geom.setSteeringAngle(steering_angle_deg)
        geom.setOffset(imfusion.vec2(out_origin_x, out_origin_y))

        new_img.attach(imfusion.US.FrameGeometryMetadata(geom))

        converted_images.append(new_img)

    out_imageset = imfusion.SharedImageSet(converted_images)
    imfusion.write(out_imageset, output_path)


if __name__ == "__main__":
    input_file = "/home/chiara_piemontese/iiwa_stack_ws/src/confidence_map/Ultrasound.imf"
    output_file = "/home/chiara_piemontese/iiwa_stack_ws/src/confidence_map/Ultrasound_from_convex_to_linear.imf"

    convex_params = {
        "origin": (324.5, 49.8),
        "opening_angle_deg": 35.35,
        "short_radius_px": 184.02,
        "long_radius_px": 649.46,
    }

    linear_params = {
        "origin": (345.1, 6.1),
        "width_px": 673.99,
        "depth_px": 508.00,
        "steering_angle_deg": 0.0,
    }

    convert_convex_us_sweep(input_file, output_file, convex_params, linear_params)

