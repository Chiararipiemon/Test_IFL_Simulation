#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Data: 5 Novembre
# Codice creato al volo per rinominare le label affinchè siano compatibili con ImFusion
import numpy as np
import nibabel as nib
from pathlib import Path

def main():
    #  Inserire il percorso e file
    in_path = Path("/home/chiararipiemo/iiwa_stack_ws/src/iiwa_probe_utils/Segmantations/segm.nii.gz")
    if not in_path.exists():
        raise FileNotFoundError(f"File non trovato: {in_path}")

    # Caricamento dell'immagine
    img = nib.load(str(in_path))
    # Legge i dati senza cambiarne i valori (evita cast a float)
    orig = np.asarray(img.dataobj)
    # Crea un volume di output inizialmente tutto a 1: background = 1
    new_data = np.ones(orig.shape, dtype=np.int16)

    # 16 -> 3
    new_data[orig == 20] = 3 # fat

    # 25..36 -> 13
    new_data[(orig >= 3) & (orig <= 14)] = 13 #bones

    # 86..89 -> 8
    new_data[(orig >= 16) & (orig <= 19)] = 8 # muscle

    # 2..3 -> 6
    new_data[(orig >= 1) & (orig <= 2)] = 6 # reni

    # 79 -> 12
    new_data[orig == 15] = 12 #soft tissues

    # Se è .nii.gz, rimuove entrambe le estensioni correttamente
    if in_path.name.endswith(".nii.gz"):
        out_name = in_path.name[:-7] + "_relabel.nii.gz"
    else:
        out_name = in_path.stem + "_relabel" + "".join(in_path.suffixes)
    out_path = in_path.parent / out_name

    hdr = img.header.copy()
    hdr.set_data_dtype(np.int16)
    out_img = nib.Nifti1Image(new_data, img.affine, hdr)
    nib.save(out_img, str(out_path))

    print(f"Salvato: {out_path}")

if __name__ == "__main__":
    main()
