# Test_IFL_Simulation
## Spawning of manipulator + bed + robot pedestal
```source ~/iiwa_stack_ws/devel/setup.bash

roslaunch iiwa_probe_utils demo_with_tool_env_iiwa_stack.launch \
  model:=iiwa14 robot_name:=iiwa rviz:=true L_tip:=0.12 \
  table_yaw:=1.5708```

-----------------------------------------------------------------------------------------------------
## Attach the probe 

source ~/iiwa_stack_ws/devel/setup.bash
ROS_NAMESPACE=iiwa rosrun iiwa_probe_utils attach_probe_collision.py \
  _link:=tool0 \
  _length:=0.16 \
  _radius:=0.03 \
  _z_offset:=0.03

-----------------------------------------------------------------------------------------------------
## Load file rosbag

source ~/iiwa_stack_ws/devel/setup.bash
rosbag record -O sweep_$(date +%F_%H%M%S).bag --lz4 \
  /tf /tf_static \
  /joint_states /iiwa/joint_states \
  /planning_scene /iiwa/planning_scene \
  /iiwa/move_group/display_planned_path \
  /iiwa/move_group/monitored_planning_scene

-----------------------------------------------------------------------------------------------------
## Spawning patient's skin surface mesh (around 4000 triangles) on the bad

ROS_NAMESPACE=iiwa rosrun iiwa_probe_utils add_patient_mesh_on_table.py \
_mesh_path:=/home/$USER/Documenti/Segmentation_decimated_better.stl \
_frame_id:=world \
_x:=0.90 _y:=0.00 \
_roll_deg:=90 _pitch_deg:=0 \
_align_mesh_yaw_with_table:=true \
_z_lift:=0.12 \
_scale_x:=1.0 _scale_y:=1.0 _scale_z:=1.0
<img width="1850" height="1031" alt="immagine" src="https://github.com/user-attachments/assets/a892c7d0-ecb9-48fb-a5c3-05e8b80cd3d8" />
