import h5py
with h5py.File("input.h5","r") as f:
    for k in f.keys():
        print(k, type(f[k]), getattr(f[k], "shape", None), getattr(f[k], "dtype", None))
