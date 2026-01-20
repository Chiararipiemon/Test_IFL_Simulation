import h5py			# I'm not sure if it's the best fformat but for now im using .h5
import numpy as np

from monai.transforms import UltrasoundConfidenceMapTransform	# monai wrapper
# Based on https://github.com/Project-MONAI/MONAI/blob/57fdd594ac905ab2ea778aa4bb79ccd9a0a03b22/monai/transforms/intensity/array.py#L4
# FILES 
in_h5 = "input.h5"  				#/home/chiara_piemontese/iiwa_stack_ws/src/confidence_map/monai
out_h5 = "output_with_conf_padded.h5"

# DATASETS 
DATASET_IN = "Ultrasound 4"         #HDF5 key containing image frames (shape (1500, 1, 512, 128)
DATASET_OUT = "confidence"


N_TEST = None  # <-- set to None for all frames (e.g., 1500)

# TRANSFORM 
t = UltrasoundConfidenceMapTransform(
    alpha=2.0,
    beta=90.0,
    gamma=0.05,
    mode="B",
    sink_mode="all",                #let MONAI choose sink points automatically
    use_cg=False,                   #solve linear with psolver
)
# in my case the dataset format is N, 1, H, W (second case)
def get_frame(ds, i):
    """
    ds: h5py dataset
    i: frame index
    returns a single frame as np.ndarray of shape (H, W) in float32
    """
    shp = ds.shape

    # Case (N, H, W)
    if len(shp) == 3:
        frame = ds[i, :, :]
        return frame.astype(np.float32, copy=False)

    # Case (N, 1, H, W)
    if len(shp) == 4 and shp[1] == 1:
        frame = ds[i, 0, :, :]
        return frame.astype(np.float32, copy=False)         # it returns a 2D frame n float32
    # this is because monai wants images shaped like [1, H, W] 
    # Case (N, H, W, 3) RGB
    if len(shp) == 4 and shp[-1] == 3:
        rgb = ds[i, :, :, :]
        frame = np.mean(rgb, axis=-1)
        return frame.astype(np.float32, copy=False)

    # Case (H, W, N) (less common / ambiguous)
    if len(shp) == 3 and shp[-1] > 1 and shp[0] != shp[-1]:
        frame = ds[:, :, i]
        return frame.astype(np.float32, copy=False)

    raise ValueError(f"Unsupported dataset shape: {shp}")

# This part is also because i have .h5 file format
with h5py.File(in_h5, "r") as f_in, h5py.File(out_h5, "w") as f_out:
    ds_in = f_in[DATASET_IN]                                 # it's the dataset object for ultrasound 4

    # Determine N, H, W, for debugging
    if len(ds_in.shape) == 3:
        # (N, H, W)
        N, H, W = ds_in.shape
    elif len(ds_in.shape) == 4 and ds_in.shape[1] == 1:
        # (N, 1, H, W)
        N, _, H, W = ds_in.shape
    elif len(ds_in.shape) == 4 and ds_in.shape[-1] == 3:
        # (N, H, W, 3)
        N, H, W, _ = ds_in.shape
    else:
        # (H, W, N)
        H, W, N = ds_in.shape

    # Number of frames to process
    N_out = N if N_TEST is None else min(N, int(N_TEST))

    # --- Metadata for ImFusion ---
    # ImFusion typically links spacing/shiftScale to the dataset name, so use "confidence_*"
    if "Ultrasound 4_spacing" in f_in:
        f_out.create_dataset("confidence_spacing", data=f_in["Ultrasound 4_spacing"][:])
    if "Ultrasound 4_shiftScale" in f_in:
        f_out.create_dataset("confidence_shiftScale", data=f_in["Ultrasound 4_shiftScale"][:])

    # PADDING SETUP: preprocessing to avoid a degenerate internal configuration.
    # Required for narrow frames (e.g., 512x128) to avoid degenerate confidence maps in MONAI's implementation. MONAI’s internal seed/sink heuristics for random-walk confidence maps can become degenerate for such narrow width, leading to a constant output
    # With 512x512 the algorithm is working then i crop back to the original width
    target_W = 512
    if W > target_W:
        raise ValueError(f"W={W} > target_W={target_W}. Reduce target_W or change strategy.")
    pad_left    = (target_W - W) // 2
    pad_right   = target_W - W - pad_left

    print(f"Input frames: N={N}, H={H}, W={W}")
    print(f"Processing frames: {N_out}/{N}")
    print(f"Padding width: left={pad_left}, right={pad_right} -> padded W={target_W}")
    print(f"Output file: {out_h5}")

    # Create output dataset: (N_out, H, W), storage
    ds_out = f_out.create_dataset(
        DATASET_OUT,
        shape=(N_out, H, W),
        dtype=np.float32,
        chunks=(1, H, W),
        compression="gzip",
        compression_opts=4,
    ) # Creates confidence dataset with shape (N_out, 512, 128) in float32

    for i in range(N_out):                              # loop for all the frames
        # 1) Read frame as (H, W)
        frame = get_frame(ds_in, i)                     # returns (H,W) float32

        # 2) Normalize uint8 -> [0, 1]
        frame = frame / 255.0
        frame = np.clip(frame, 0.0, 1.0)

        # 3) Pad width to target_W: (H, target_W)
        frame_p = np.pad(frame, ((0, 0), (pad_left, pad_right)), mode="edge")

        # 4) Compute confidence on padded frame
        conf_p = np.asarray(t(frame_p[None, ...]))      # t wants input shape (1, H, W) -> output shape (1, 1, H, target_W)
        conf_p = conf_p[0] if conf_p.ndim == 3 else conf_p  # -> (H, target_W)

        # 5) Crop back to original width: (H, W)
        conf2d = conf_p[:, pad_left : pad_left + W]

        # 6) Save
        ds_out[i, :, :] = conf2d.astype(np.float32)

        if i % 50 == 0:
            print(f"Processed {i}/{N_out} frames")

print("Done.")

