Per ottenere una serpentina corretta: 
```
ROS_NAMESPACE=iiwa \
python3 ~/iiwa_stack_ws/src/iiwa_probe_utils/scripts/18Jan/raster_serpentine_scan_MLS.py \
  _cloud_topic:=/skin_cloud \
  _raster_enable:=true \
  _raster_style:=cross \
  _sweep_length:=0.20 \
  _raster_width:=0.06 \
  _raster_line_spacing:=0.04 \
  _samples_per_line:=40 \
  _raster_bridge_samples:=10 \
  _sweep_pref_dir:='[0,1,0]' \
  _fixed_z_dir:='[0,0,-1]' \
  _fixed_flip_to_face_target:=true \
  _approach_dist:=0.03 \
  _retreat_dist:=0.06 \
  _ik_timeout:=3.0 \
  _allow_partial:=true \
  _min_partial_fraction:=0.10
```
 La traiettoria è:
 
<img width="596" height="455" alt="output(4)" src="https://github.com/user-attachments/assets/c87c3cfe-2ea5-4a66-8da2-03cb173471ca" />
<img width="795" height="719" alt="Schermata del 2026-01-18 17-01-38" src="https://github.com/user-attachments/assets/86977823-1f7e-4557-89f1-bee5ac348dff" />
Poi dentro la console python imfusion:
```
import runpy; runpy.run_path("/home/chiararipiemo/iiwa_stack_ws/src/iiwa_probe_utils/Hybrid_simulation/sweeps_for_hus.py", run_name="__main__")
```
