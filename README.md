# Test_IFL_Simulation
## Set up and requirements
This repo works with Ubuntu 20.04 and ROS Noetic. Most of the files are connected and based on ***https://github.com/IFL-CAMP/iiwa_stack***.
### Load iiwa_stack
#### Clone this repository to your workspace:
```
mkdir iiwa_stack_ws && cd iiwa_stack_ws && mkdir src
catkin_init_workspace
git clone https://github.com/IFL-CAMP/iiwa_stack.git src/iiwa_stack
```
#### Download the dependences :
```
rosdep install --from-paths src --ignore-src -r -y
```
#### Build the workspace :
```
catkin build
```
#### Source the workspace :
```
source devel/setup.bash
```
#### Load the directory:
Load the directory ***iiwa_probe_utils***. The directory should be inside iiwa_stack/src. Load also the directory ***cloudpoint*** inside iiwa_probe_utils.

My **Segmentation_decimated_better.stl** file is loaded inside ~/Documents. This file is a mesh.

## Spawning of manipulator + bed + robot pedestal
```
source ~/iiwa_stack_ws/devel/setup.bash
roslaunch iiwa_probe_utils demo_with_tool_env_iiwa_stack.launch \
  model:=iiwa14 robot_name:=iiwa rviz:=true L_tip:=0.24 \
  table_yaw:=1.5708
```
La lunghezza L_Tip è settata a 0.24 per simulare una sorta di pressione del probe sulla pelle del paziente.
There's problably collision between base link and robot pedestal, to fix it.

Note: the pedestal and the table spawn below MoveIt!’s “virtual” floor level. From code I managed to raise only the pedestal and the table, but the robot stays anchored to the floor because that’s how it was configured by the creators of iiwa_stack (which I’m using some files from), and I haven’t found a way to unpin it. This is definitely something to tackle in the future if I want to polish everything; for now I’m fine with it.

<img width="453" height="360" alt="immagine" src="https://github.com/user-attachments/assets/1d08c3eb-37e9-4888-aecd-88282d81d0d4" />

-----------------------------------------------------------------------------------------------------
## Attach the probe 
Change the _mesh_path
```
source ~/iiwa_stack_ws/devel/setup.bash
ROS_NAMESPACE=iiwa rosrun iiwa_probe_utils attach_tool_mesh.py \
  _mesh_path:=/home/chiararipiemo/iiwa_stack_ws/src/iiwa_probe_utils/probe_urdf/IFL_FrankaHolder.dae \
  _link_name:=iiwa_link_ee \
  _name:=probe_holder \
  _x:=0.00 _y:=0.00 _z:=-0.10 \
  _roll_deg:=0 _pitch_deg:=0 _yaw_deg:=0 \
  _scale_x:=1.0 _scale_y:=1.0 _scale_z:=1.0
```
-----------------------------------------------------------------------------------------------------
## Load skin cloudpoint 
Change pdc_path with yours
```
rosrun iiwa_probe_utils add_patient_cloud_on_table.py \
  _pcd_path:=/home/chiararipiemo/iiwa_stack_ws/src/iiwa_probe_utils/cloudpoint/Segmentation_decimated_better_points.pcd \
  _frame_id:=world \
  _x:=0.65 _y:=-0.20 \
  _roll_deg:=90 _pitch_deg:=0 \
  _align_mesh_yaw_with_table:=false _table_yaw:=1.5708 \
  _z_lift:=0.12
```
If the point cloud doesn’t appear, go to the MoveIt! GUI → Add → PointCloud2 and select the topic: /cloud_with_normals. Then File --> save config
### Overlay the normal arrows
The normal arrows are the blue arrows that you can visualize in the image below
```
source ~/iiwa_stack_ws/devel/setup.bash
rosrun iiwa_probe_utils normals_markers_from_cloud.py \
  _topic_in:=/cloud_with_normals \
  _step:=25 \
  _scale:=0.03 \
  _max_markers:=500
```
<img width="1849" height="1028" alt="immagine" src="https://github.com/user-attachments/assets/db837fcd-7828-4cbb-b427-32354cdedf74" />

-----------------------------------------------------------------------------------------------------
## Execute raster scan
Aggiornamento ***4 Novembre ore 10:34***: 
- robot che va da home > pre.approach > si posiziona normale al punto p0: il contatto avviene tra frame probe_tip e punto p0.Il frame probe_tip è circa coincidente con la punta finale del probe.
```
ROS_NAMESPACE=iiwa \
~/iiwa_stack_ws/src/iiwa_probe_utils/scripts/4_nov/pre_to_pose_and_touch.py \
  _group_name:=manipulator _ee_link:=iiwa_link_ee _ref_frame:=world \
  _speed_scale:=0.2 \
  _pre_joints:="[-2.529, 0.271, -0.268, 1.141, 2.932, 1.581, 0.174]" \
  _target_joints:="[-0.176, 0.675, 0.008, -0.789, -0.004, 1.669, -0.169]" \
  _cloud_topic:=/cloud_with_normals \
  _tip_frame:=probe_tip \
  _contact_margin:=0.006 \
  _approach_dist:=0.05 _retreat_dist:=0.00 \
  _far_steps:=1 _pre_steps:=12 _approach_steps:=18 \
  _step_time:=0.25 _ik_timeout:=1.5 \
  _allow_partial:=true _min_partial_fraction:=0.2 _pos_tol_final:=0.02 \
  _ik_service:=/iiwa/compute_ik
```
Aggiornamento **4 Novembre ore 13:55**:
- sto lavorando ad un codice che facccia scorrere il probe su una linea retta (sweep lineare) P0→Pdes, Z_tool = −normale in ogni campione, roll bloccato sulla direzione della linea, lift e rientro in pre_approach

Aggiornamento **4 Novembre 15:44**:

Approfondimenti nella cartella 4_nov
```
cd /home/chiararipiemo/iiwa_stack_ws
source devel/setup.bash
ROS_NAMESPACE=/iiwa \
python3 src/iiwa_probe_utils/scripts/4_nov/pre_to_pose_touch_and_sweep_new.py \
  _cloud_topic:=/cloud_with_normals \
  _sweep_length:=0.20 \
  _sweep_samples:=40 \
  _sweep_pref_dir:='[0,1,0]' \
  _approach_dist:=0.03 \
  _retreat_dist:=0.06 \
  _ik_timeout:=3.0 \
  _allow_partial:=true \
  _min_partial_fraction:=0.10
```
Aggiornamento **4 Novembre 16:33**
- visti i precedenti probemi riscontrati nel precedente aggiornamento, sto lavorando ad un codice che tenga sempre fisso l'orientamento del probe e non consideri più la nromale ad ogni punto
```
ROS_NAMESPACE=/iiwa \
python3 src/iiwa_probe_utils/scripts/4_nov/pre_to_pose_touch_and_sweep_fixedZ.py \
  _cloud_topic:=/cloud_with_normals \
  _sweep_length:=0.20 \
  _sweep_samples:=40 \
  _sweep_pref_dir:='[0,1,0]' \
  _use_fixed_z:=true \
  _fixed_z_dir:='[0,0,-1]' \
  _fixed_flip_to_face_target:=true \
  _approach_dist:=0.03 \
  _retreat_dist:=0.06 \
  _ik_timeout:=3.0 \
  _allow_partial:=true \
  _min_partial_fraction:=0.10
```
