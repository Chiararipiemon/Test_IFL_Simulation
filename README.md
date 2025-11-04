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
## Start loading file rosbag
Work in progress, but for now is not the main problem
```
source ~/iiwa_stack_ws/devel/setup.bash
rosbag record -O sweep_$(date +%F_%H%M%S).bag --lz4 \
  /tf /tf_static \
  /joint_states /iiwa/joint_states \
  /planning_scene /iiwa/planning_scene \
  /iiwa/move_group/display_planned_path \
  /iiwa/move_group/monitored_planning_scene
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
**Work in progress**

Voglio in qualche modo ottenere un codice che faccia queste cose:

- legge la cloud (con normali) da /cloud_with_normals
- fa n modo che il robot si posizioni in una posa di pre-appproach con probe orientato verso la nuvola di punti
- prende P0 e Pdes (via parametri oppure cliccando due punti in RViz su /clicked_point),
- campiona la retta in N punti, per ogni punto prende il nearest sulla cloud (e la sua normale),
- costruisce le pose (pos = punto_surface − backoff·normale, orientamento: asse Z del tool allineato a −normale, asse X allineato alla direzione di scan proiettata sul piano tangente),
- pianifica ed esegue un percorso cartesiano con MoveIt.

Problemi riscontrati fino ad ora:
- fallisco ogni approccio perchè in qualche modo il robot va in collisione o fa dei movimenti strani tali per cui impazzisce e non riesce a raggiungere la posa desiderata.
- ho provato a lanciare banalmente un codice in cui semplicemente a partire dalla home pose il robot deve ragggiungere una sola posa di pre approach, riesco ad ottenere la posa pianificata ma poi fallisco ogni tentativo di approccio CORRETTO al punto p0 --> approfondire su **issues.md** e **README.md** nella cartella ***3_nov***.

Cosa sono riuscita ad ottenere almeno, aggiornamento **3 Novembre ore 17:36**, tutti i file py sono nella cartella 3_nov:
- robot che da home va in una posa di pre-approach e poi si approccia al cloudpoint ma senza consapevolezza del cloudpoint, impongo solo i valori in radian che ciascun joint dele assumere:
```
source ~/iiwa_stack_ws/devel/setup.bash
ROS_NAMESPACE=iiwa \
~/iiwa_stack_ws/src/iiwa_probe_utils/scripts/3_nov/move_pre_to_pose.py \
  _group_name:=manipulator _ee_link:=iiwa_link_ee \
  _speed_scale:=0.2 \
  _fallback_steps:=50 _fallback_dt:=0.20
```

Aggiornamento ***4 Novembre ore 10:34**: 
- robot che va da home > pre.approach > si posiziona normale al punto p0: il contatto avviene tra frame probe_tip e punto p0.
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

