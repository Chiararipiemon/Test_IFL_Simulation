# Lanciare il file di Alejandro
exec("""
import imfusion as imf
import imfusion.ultrasound as us
import numpy as np

def sliceSizeFromGeometry(frameGeometry):
    if frameGeometry.is_linear:
        extent = [frameGeometry.img_desc.width / 2.0,
                  frameGeometry.img_desc.depth / 2.0]
    elif frameGeometry.is_convex:
        sx = 2 * frameGeometry.long_radius * np.sin(np.deg2rad(frameGeometry.opening_angle))
        sy = frameGeometry.long_radius - frameGeometry.short_radius * np.cos(np.deg2rad(frameGeometry.opening_angle))
        extent = [sx.item(), sy.item()]
    return extent

def createConvexGeometry(probe_width, long_radius, depth, opening_angle):
    fg = us.FrameGeometryConvex(us.CoordinateSystem.IMAGE)
    fg.img_desc = imf.ImageDescriptor(imf.PixelType.UBYTE,128,128,1,1)
    fg.opening_angle = opening_angle
    fg.short_radius = probe_width / (2 * np.sin(np.deg2rad(opening_angle))).item()
    fg.long_radius = long_radius
    fg.depth = depth
    fg.top_down = True
    return fg

def createLinearGeometry(probe_width, depth, opening_angle):
    fg = us.FrameGeometryLinear(us.CoordinateSystem.IMAGE)
    fg.img_desc = imf.ImageDescriptor(imf.PixelType.UBYTE,128,128,1,1)
    fg.width = probe_width
    fg.depth = depth
    fg.top_down = True
    return fg

def setCenter(fg, extent):
    if fg.is_convex:
        sign = 1 if fg.top_down else 0
        y = -extent[0] + sign * fg.short_radius * (1 - np.cos(np.deg2rad(fg.opening_angle))).item()
        fg.offset = np.array([0, y])
    if fg.is_linear:
        fg.offset = np.array([0, -extent[1]])

def createEmptyImage(extent):
    img_desc = imf.ImageDescriptor(imf.PixelType.UBYTE,128,128,1,1)
    img_desc.spacing = np.array([extent[0] * 2 / 128, extent[1] * 2 / 128, 1])
    img_desc.is_metric = True
    im = imf.SharedImage(imf.MemImage(img_desc))
    return im

trackings = [el for el in imf.app.data_model if type(el) == imf.TrackingSequence]
if len(trackings) == 0:
    print("Error, no trackings found in the data model")
else:
    tracking = trackings[0]

    sweep = us.UltrasoundSweep()
    sweep.name = "Synthetic Sweep from csv poses"

    tracking_copy = imf.TrackingSequence()

    opening_angle = 30
    probe_width = 30   # <-- metti un valore sensato, NON 0
    long_radius = 100
    depth = 50

    fg = createConvexGeometry(probe_width, long_radius, depth, opening_angle)
    extent = sliceSizeFromGeometry(fg)
    setCenter(fg, extent)

    sweep_time = 3
    nframes = tracking.size
    timestep = sweep_time / (nframes - 1)
    perpendicularSlices = True

    for i in range(1, nframes-1):
        m = tracking.raw_matrix(i)
        t = i * timestep

        d = np.array([0, -60, 0])
        depth_axis = d / np.linalg.norm(d)

        sweep_tangent = tracking.raw_matrix(i+1)[:3,3] - tracking.raw_matrix(i-1)[:3,3]
        sweep_tangent /= np.linalg.norm(sweep_tangent)

        normalSlice = np.cross(sweep_tangent, depth_axis)
        if perpendicularSlices:
            normalSlice = np.cross(depth_axis, normalSlice)

        outMat = np.eye(4)
        outMat[:3,0] = np.cross(depth_axis, normalSlice)
        outMat[:3,1] = depth_axis
        outMat[:3,2] = normalSlice
        outMat[:3,3] = m[:3,3] + (depth_axis * extent[1])

        tracking_copy.add(outMat, t, 1.0)
        sweep.add(createEmptyImage(extent))
        sweep.set_timestamp(t, sweep.size - 1)

    sweep.add_tracking(tracking_copy)
    sweep.properties.set_param("topDown", True)

    fgm = [el for el in sweep.components if type(el) == us.FrameGeometryMetadata][0]
    fgm.frame_geometry = fg

    imf.app.data_model.add(sweep)
""", globals())
**Problemi**:
AttributeError: module 'imfusion.ultrasound' has no attribute 'CoordinateSystem'
