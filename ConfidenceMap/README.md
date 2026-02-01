Compute an ultrasound confidence map for each 2D frame using MONAI’s UltrasoundConfidenceMapTransform, which implements the random-walks approach of Karamalis et al. (MedIA 2012). For each frame, we provide an input array of shape [1, H, W] (single channel), and the transform returns a confidence map of the same spatial size, where each pixel encodes the probability that a random walk starting at that pixel reaches the source (high confidence) before reaching any sink (low confidence). Internally, MONAI builds a weighted pixel graph, forms a sparse Laplacian, and solves a linear system to obtain these probabilities; the parameters α, β, γ control the edge weighting/smoothness and sensitivity to intensity transitions, while sink_mode defines how sink locations are chosen.
Our raw frames are stored as (H, W) = (512, 128) and are normalized to [0, 1] prior to processing. We observed that MONAI’s default sink/source heuristics can become degenerate for very narrow images, producing a constant confidence map. To avoid this, we pad each frame laterally to 512×512, compute the confidence map on the padded image, and then crop back to the original width (512×128) before saving. For correct visualization and overlay in ImFusion, we store the output together with matching metadata (confidence_spacing, confidence_shiftScale) so that the confidence map is rendered with the same physical spacing as the original ultrasound frames.

TO DO:
```
cd /home/chiara_piemontese/iiwa_stack_ws/src/confidence_map/monai
python3 test_2_first_monai.py
python3 confidence_map_monai.py
python3 test_monai.py

```
Some other results: 
<img width="283" height="424" alt="image" src="https://github.com/user-attachments/assets/fe63d7d8-441e-4bd1-a135-07bdb6da4a05" />

<img width="1388" height="1000" alt="image" src="https://github.com/user-attachments/assets/7234f215-70b5-4947-9998-e2db10b4bafa" />
