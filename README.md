# Test_IFL_Simulation
## Set up and requirements
This repo works with Ubuntu 20.04 and ROS Noetic.
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
#### Load the directory
Load the directory ***iiwa_probe_utils***. The directory should be inside iiwa_stack/src.

My Segmentation_decimated_better.stl file is loaded inside ~/Documents

## Spawning of manipulator + bed + robot pedestal
There's problably collision between base link and robot pedestal, to fix it
```
source ~/iiwa_stack_ws/devel/setup.bash

roslaunch iiwa_probe_utils demo_with_tool_env_iiwa_stack.launch \
  model:=iiwa14 robot_name:=iiwa rviz:=true L_tip:=0.12 \
  table_yaw:=1.5708
```

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
## Spawning patient's skin surface mesh (around 4000 triangles) on the bad
Change the mesh path
```
ROS_NAMESPACE=iiwa rosrun iiwa_probe_utils add_patient_mesh_on_table.py \
_mesh_path:=/home/$USER/Documenti/Segmentation_decimated_better.stl \
_frame_id:=world \
_x:=0.65 _y:=-0.20 \
_roll_deg:=90 _pitch_deg:=0 \
_align_mesh_yaw_with_table:=true \
_z_lift:=0.22 \
_scale_x:=1.0 _scale_y:=1.0 _scale_z:=1.0
```
<img width="1850" height="1031" alt="immagine" src="https://github.com/user-attachments/assets/a892c7d0-ecb9-48fb-a5c3-05e8b80cd3d8" />

-----------------------------------------------------------------------------------------------------
## Execute rastr scan
```
source ~/iiwa_stack_ws/devel/setup.bash
ROS_NAMESPACE=iiwa rosrun iiwa_probe_utils go_to_preapproach_diagnose.py
```
