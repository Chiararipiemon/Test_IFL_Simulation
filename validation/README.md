# Preliminary tests and validation pipeline
## Set up the environment: spawn robot + bed + robot pedestal
```
source ~/iiwa_stack_ws/devel/setup.bash
roslaunch iiwa_probe_utils demo_with_tool_env_iiwa_stack.launch \
  model:=iiwa14 robot_name:=iiwa rviz:=true L_tip:=0.24 \
  table_yaw:=1.5708
```
## Attach the probe 
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
## Load skin cloudpoint 
```
rosrun iiwa_probe_utils add_patient_cloud_on_table.py \
  _pcd_path:=/home/chiararipiemo/iiwa_stack_ws/src/iiwa_probe_utils/cloudpoint/Segmentation_decimated_better_points.pcd \
  _frame_id:=world \
  _x:=0.65 _y:=-0.20 \
  _roll_deg:=90 _pitch_deg:=0 \
  _align_mesh_yaw_with_table:=false _table_yaw:=1.5708 \
  _z_lift:=0.12 \
  _topic_name:=/skin_cloud
```
