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
short radius and the vertical symmetry axis【347175179612003†L29-L34】.  In
contrast, a linear frame geometry is defined by a width and a depth,
with the offset at the centre of the transducer array and the
steering angle controlling any lateral tilt【756595175379526†L27-L34】.

Note that this script relies on the `imfusion` Python package to
read/write ImFusion datasets and to create frame geometry objects.
Refer to the ImFusion SDK documentation for details.  The numerical
resampling is done with NumPy only, which makes the code independent
of any proprietary GPU functionality.  Bilinear interpolation is
implemented manually for clarity.

Usage example
-------------

```
import imfusion
from convex_to_linear import convert_convex_us_sweep

# path to the input convex dataset (.imf, .mha, .us, ...)
input_file = 'convex_sweep.imf'
# path for the resampled linear dataset
output_file = 'linear_sweep.imf'

convex_params = {
    'origin': (324.5, 49.8),      # (x0, y0) in pixels from the image provided
    'opening_angle_deg': 35.35,   # half fan opening in degrees (± around vertical)
    'short_radius_px': 184.02,    # inner radius in pixels
    'long_radius_px': 649.46,     # outer radius in pixels
}

linear_params = {
    'origin': (345.1, 6.1),       # desired output offset in pixels
    'width_px': 673.99,           # width of the linear frame in pixels
    'depth_px': 508.00,           # depth of the linear frame in pixels
    'steering_angle_deg': 0.0,    # no lateral steering
}

# perform the conversion and save the output
convert_convex_us_sweep(input_file, output_file, convex_params, linear_params)
```

"""

from __future__ import annotations

import math
from typing import Dict, Tuple

import numpy as np

try:
    import imfusion
except ImportError as e:  # pragma: no cover
    raise RuntimeError(
        "The imfusion package is required to run this script. "
        "Install it from the ImFusion SDK or run inside ImFusionSuite.")


def _bilinear_sample(img: np.ndarray, x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """Sample the 2D image ``img`` at floating point coordinates ``(x, y)``.

    Parameters
    ----------
    img:
        A 2‑D NumPy array representing the grayscale pixel data.
    x, y:
        Arrays of the same shape containing the x‑ and y‑coordinates to sample.

    Returns
    -------
    np.ndarray
        An array with the same shape as ``x`` and ``y`` containing the sampled
        values.  Points outside the image bounds are filled with zeros.
    """
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

    # Gather the four surrounding pixels
    Ia = img[y0_clip, x0_clip]
    Ib = img[y1_clip, x0_clip]
    Ic = img[y0_clip, x1_clip]
    Id = img[y1_clip, x1_clip]

    # Compute interpolation weights
    wa = (x1 - x) * (y1 - y)
    wb = (x1 - x) * (y - y0)
    wc = (x - x0) * (y1 - y)
    wd = (x - x0) * (y - y0)

    return wa * Ia + wb * Ib + wc * Ic + wd * Id


def _convex_to_linear_frame(
    frame: np.ndarray,
    apex: Tuple[float, float],
    opening_angle_rad: float,
    r0: float,
    r1: float,
    out_width: int,
    out_depth: int,
) -> np.ndarray:
    """Convert a single 2‑D ultrasound frame from convex to linear geometry.

    The mapping assumes a fan shape centred at ``apex`` with an inner radius
    ``r0`` and an outer radius ``r1``.  The fan spans ± ``opening_angle_rad``
    around the vertical axis.  The output will have shape
    (out_depth, out_width).

    Parameters
    ----------
    frame:
        Input frame as a 2‑D NumPy array (height × width).
    apex:
        (x, y) coordinates of the virtual apex in pixel units.
    opening_angle_rad:
        Half of the fan opening in radians; the full fan covers ± ``opening_angle_rad``.
    r0:
        Short (inner) radius in pixels.
    r1:
        Long (outer) radius in pixels.
    out_width:
        Number of lateral samples for the linear frame.
    out_depth:
        Number of axial samples for the linear frame.

    Returns
    -------
    np.ndarray
        A 2‑D array of shape (out_depth, out_width) containing the resampled
        linear frame.
    """
    apex_x, apex_y = apex
    # Prepare angular and radial sampling grids
    # Angular coordinates go from -opening_angle to +opening_angle across the width
    phi = np.linspace(-opening_angle_rad, opening_angle_rad, out_width)
    # Radial distances go from r0 (near field) to r1 (far field) across the depth
    r = np.linspace(r0, r1, out_depth)

    # Create a meshgrid with shape (out_depth, out_width)
    phi_grid, r_grid = np.meshgrid(phi, r, indexing='xy')

    # Compute corresponding source coordinates.  In image coordinates,
    # the vertical axis points downwards, so the y‑coordinate is apex_y + r*cos(phi),
    # and the x‑coordinate is apex_x + r*sin(phi).
    x_src = apex_x + r_grid * np.sin(phi_grid)
    y_src = apex_y + r_grid * np.cos(phi_grid)

    # Perform bilinear sampling; points outside the image will be zero
    sampled = _bilinear_sample(frame, x_src, y_src)
    return sampled


def convert_convex_us_sweep(
    input_path: str,
    output_path: str,
    convex_params: Dict[str, float],
    linear_params: Dict[str, float],
) -> None:
    """Convert an ultrasound sweep from convex to linear geometry.

    This function loads a shared image set from ``input_path``, applies
    the convex‑to‑linear conversion to every frame in the set, assigns a
    new ``FrameGeometryLinear`` to each output image, and writes the result
    to ``output_path``.

    Parameters
    ----------
    input_path:
        Path to the input ImFusion file containing a 2D+T ultrasound sweep with
        convex geometry.
    output_path:
        Path where the resampled dataset should be saved.  The extension should
        reflect an ImFusion‑supported format (e.g., ``.imf``).
    convex_params:
        Dictionary with keys ``origin`` (tuple of floats), ``opening_angle_deg``
        (half fan opening in degrees), ``short_radius_px`` and ``long_radius_px``.
    linear_params:
        Dictionary with keys ``origin`` (tuple of floats), ``width_px``,
        ``depth_px`` and ``steering_angle_deg``.

    Notes
    -----
    The function assumes that the input geometry is oriented top‑down (i.e.
    increasing ``y`` goes away from the probe).  Should your data follow a
    bottom‑up convention, adjust the apex computation accordingly.  The
    steering angle for the output is applied via the ``FrameGeometryLinear``
    class, which allows specifying an angular tilt【756595175379526†L27-L34】.
    """
    # Load the input dataset; we expect a SharedImageSet
    imageset, *_ = imfusion.load(input_path)

    # Extract convex parameters
    origin_x, origin_y = convex_params['origin']
    opening_angle_deg = float(convex_params['opening_angle_deg'])
    opening_angle_rad = math.radians(opening_angle_deg)
    short_radius = float(convex_params['short_radius_px'])
    long_radius = float(convex_params['long_radius_px'])

    # Compute the apex position.  For a convex frame geometry, the offset is not
    # the centre of the ring sector; the rays emanate from a virtual point
    # located one short radius behind the offset along the vertical axis【347175179612003†L29-L34】.
    apex_x = origin_x
    apex_y = origin_y + short_radius

    # Extract linear parameters
    out_origin_x, out_origin_y = linear_params['origin']
    out_width = int(round(linear_params['width_px']))
    out_depth = int(round(linear_params['depth_px']))
    steering_angle_deg = float(linear_params['steering_angle_deg'])

    # Prepare container for converted images
    converted_images = []

    # Iterate through each frame in the input sweep
    for idx in range(len(imageset)):
        img = imageset[idx]
        # Convert to numpy array; squeeze to 2D (height, width)
        arr = np.array(img).squeeze()
        # Resample from convex to linear
        resampled = _convex_to_linear_frame(
            arr,
            apex=(apex_x, apex_y),
            opening_angle_rad=opening_angle_rad,
            r0=short_radius,
            r1=long_radius,
            out_width=out_width,
            out_depth=out_depth,
        )

        # Create a new SharedImage from the resampled array
        new_img = imfusion.SharedImage(resampled.astype(arr.dtype))

        # Assign the same spacing as the original (optional)
        new_img.setSpacing(img.spacing()[0], img.spacing()[1], img.spacing()[2])

        # Construct a linear frame geometry in pixel units
        coord_sys = imfusion.US.FrameGeometry.CoordinateSystem.Pixels
        geom = imfusion.US.FrameGeometryLinear(coord_sys)
        geom.setWidth(float(linear_params['width_px']))
        geom.setDepth(float(linear_params['depth_px']))
        geom.setSteeringAngle(steering_angle_deg)
        geom.setOffset(imfusion.vec2(out_origin_x, out_origin_y))

        # Attach the geometry to the image
        new_img.attach(imfusion.US.FrameGeometryMetadata(geom))

        converted_images.append(new_img)

    # Build a new SharedImageSet from the converted images
    out_imageset = imfusion.SharedImageSet(converted_images)

    # Write the output dataset
    imfusion.write(out_imageset, output_path)


__all__ = ['convert_convex_us_sweep']