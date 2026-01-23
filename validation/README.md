# Preliminary tests and validation pipeline
## Set up
### Set up the environment: spawn robot + bed + robot pedestal
```
source ~/iiwa_stack_ws/devel/setup.bash
roslaunch iiwa_probe_utils demo_with_tool_env_iiwa_stack.launch \
  model:=iiwa14 robot_name:=iiwa rviz:=true L_tip:=0.24 \
  table_yaw:=1.5708
```
### Attach the probe 
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
### Load skin cloudpoint 
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
### Publish the organs
```
rosrun us_planner publish_skin_seg_on_table.py \
  _skin_pcd_path:=/home/chiararipiemo/iiwa_stack_ws/src/us_planner/skin_patient_m.pcd \
  _seg_pcd_path:=/home/chiararipiemo/iiwa_stack_ws/src/us_planner/organs_13_8_6_3_patient_sub32.pcd \
  _frame_id:=world \
  _x:=0.65 _y:=-0.20 \
  _roll_deg:=90 _pitch_deg:=0 \
  _align_mesh_yaw_with_table:=false _table_yaw:=1.5708 \
  _z_lift:=0.12 \
  _skin_topic:=/skin_cloud \
  _seg_topic:=/segmentation_cloud
```
## Target selection, robot control and planner
### Start apex_projector and obtain the first apex point
```
cd ~/iiwa_stack_ws
source devel/setup.bash

rosrun us_planner apex_projector.py
```
### Open ImFusion and copy this inside the python console
```
import imfusion, subprocess; code = """
am = imfusion.app.annotation_model
ann = am.create_annotation(imfusion.Annotation.AnnotationType.POINT)
ann.color = (1.0, 0.0, 0.0)

def on_done():
    if not ann.points:
        print('No points defined')
        return
    x_mm, y_mm, z_mm = ann.points[0]
    print('ImFusion clicked point (mm):', x_mm, y_mm, z_mm)
    cmd = [
        'bash', '-lc',
        'source /opt/ros/noetic/setup.bash && source ~/iiwa_stack_ws/devel/setup.bash && rosrun us_planner imfusion_us_target_pub.py {} {} {}'.format(x_mm, y_mm, z_mm)
    ]
    subprocess.Popen(cmd)
    print('Sent point to ROS via rosrun (bash -lc)')

ann.on_editing_finished(on_done)
ann.start_editing()
print('Now click one POINT in the view to define the US target.')
"""; exec(code)
```
### Go to us_apex with robo on Moveit!
```
ROS_NAMESPACE=iiwa \
~/iiwa_stack_ws/src/us_planner/scripts/go_to_us_apex.py \
  _group_name:=manipulator _ee_link:=iiwa_link_ee _ref_frame:=world \
  _speed_scale:=0.2 \
  _pre_joints:="[-2.529, 0.271, -0.268, 1.141, 2.932, 1.581, 0.174]" \
  _target_joints:="[-0.176, 0.675, 0.008, -0.789, -0.004, 1.669, -0.169]" \
  _tip_frame:=probe_tip \
  _contact_margin:=0.006 \
  _apex_topic:=/us_apex \
  _apex_timeout:=10.0
```
second command to test:
```
ROS_NAMESPACE=iiwa \
python3 ~/iiwa_stack_ws/src/us_planner/scripts/go_to_us_apex.py \
  _group_name:=manipulator \
  _ref_frame:=world \
  _speed_scale:=0.2 \
  _pre_joints:="[-2.529, 0.271, -0.268, 1.141, 2.932, 1.581, 0.174]" \
  _target_joints:="[-0.176, 0.675, 0.008, -0.789, -0.004, 1.669, -0.169]" \
  _tip_frame:=probe_tip \
  _apex_topic:=/us_apex \
  _apex_pose_topic:=/us_apex_pose \
  _apex_timeout:=10.0 \
  _contact_margin:=0.0 \
  _planning_time:=15.0 \
  _planning_attempts:=20 \
  _pos_tol:=0.001 \
  _ori_tol_deg:=2.0
```
right one:
```
ROS_NAMESPACE=iiwa \
python3 ~/iiwa_stack_ws/src/us_planner/scripts/go_to_us_apex.py \
  _group_name:=manipulator \
  _ee_link:=iiwa_link_ee \
  _ref_frame:=world \
  _speed_scale:=0.2 \
  _pre_joints:="[-2.529, 0.271, -0.268, 1.141, 2.932, 1.581, 0.174]" \
  _target_joints:="[-0.176, 0.675, 0.008, -0.789, -0.004, 1.669, -0.169]" \
  _tip_frame:=probe_tip \
  _apex_topic:=/us_apex \
  _apex_pose_topic:=/us_apex_pose \
  _apex_timeout:=10.0 \
  _contact_margin:=0.0 \
  _planning_time:=15.0 \
  _planning_attempts:=20 \
  _pos_tol:=0.001 \
  _ori_tol_deg:=2.0
```
### Record .csv of the probe_tip on the apex point
```
 rosrun us_planner export_probe_pose_Tfixed_for_imfusion.py \
     _probe_frame:=probe_tip \
     _base_frame:=world \
     _output_csv:=/home/chiararipiemo/pose_from_robot_for_imfusion.csv
```
right one: 
```
python3 record_us_apex_pose_imfusion_csv.py \
  _topic:=/us_apex_pose \
  _base_frame:=world \
  _output_csv:=/home/chiararipiemo/validation/us_apex_pose_imfusion.csv \
  _max_samples:=2000
```
### Hybrid simulation
Change the file get_sweep_from_csv_quaternion.py with the new .csv and run inside the python console:
```
exec(open("/home/chiararipiemo/Hybrid_simulation_last_try/get_sweep_from_csv_quaternion.py", encoding="utf-8").read()); main()
```
### Get us_best_pose with interaction mode
```
ROS_NAMESPACE=/iiwa \
python3 ~/iiwa_stack_ws/src/us_planner/scripts/us_pose_planner_confidence.py \
  _cloud_topic:=/skin_cloud \
  _conf_mha_path:=/home/chiararipiemo/iiwa_stack_ws/src/us_planner/volume_confidence.mha \
  _conf_volume_frame:=imfusion \
  _T_imfusion_from_world_mm:="[[1,0,0,-642],[0,0,1,-364],[0,-1,0,-200],[0,0,0,1]]" \
  _patch_radius_m:=0.030 \
  _n_apex_samples:=20 \
  _n_tilt_samples:=8 \
  _alpha_max_deg:=12.0 \
  _theta_align_deg:=12.0 \
  _w_align:=1.2 \
  _w_move:=0.05 \
  _w_tilt:=0.15 \
  _conf_agg:=p95 \
  _conf_p_quantile:=0.98 \
  _conf_target_radius_mm:=8.0 \
  _conf_target_samples:=20 \
  _interactive_mode:=console \
  _max_user_options:=5
```
For hard targets:
```
ROS_NAMESPACE=/iiwa \
python3 ~/iiwa_stack_ws/src/us_planner/scripts/us_pose_planner_confidence.py \
  _cloud_topic:=/skin_cloud \
  _conf_mha_path:=/home/chiararipiemo/iiwa_stack_ws/src/us_planner/volume_confidence.mha \
  _conf_volume_frame:=imfusion \
  _T_imfusion_from_world_mm:="[[1,0,0,-642],[0,0,1,-364],[0,-1,0,-200],[0,0,0,1]]" \
  _patch_radius_m:=0.1 \
  _n_apex_samples:=100 \
  _n_tilt_samples:=100 \
  _alpha_max_deg:=40.0 \
  _theta_align_deg:=40.0 \
  _w_align:=1.2 \
  _w_move:=0.05 \
  _w_tilt:=0.15 \
  _conf_agg:=p95 \
  _conf_p_quantile:=0.98 \
  _conf_target_radius_mm:=8.0 \
  _conf_target_samples:=20 \
  _interactive_mode:=console \
  _max_user_options:=5
```
### Go to /us_best_pose
```
ROS_NAMESPACE=iiwa \
python3 ~/iiwa_stack_ws/src/us_planner/scripts/go_to_us_best_pose.py \
  _group_name:=manipulator _ee_link:=iiwa_link_ee _ref_frame:=world \
  _speed_scale:=0.2 \
  _pre_joints:="[-2.529, 0.271, -0.268, 1.141, 2.932, 1.581, 0.174]" \
  _target_joints:="[-0.176, 0.675, 0.008, -0.789, -0.004, 1.669, -0.169]" \
  _tip_frame:=probe_tip \
  _contact_margin:=0.006 \
  _approach_dist:=0.03 \
  _two_stage:=true \
  _best_pose_topic:=/us_best_pose \
  _best_pose_timeout:=10.0

```
### Record .csv with /us_best_pose
right one:
```
python3 record_us_best_pose_imfusion_csv.py \
  _topic:=/us_best_pose \
  _base_frame:=world \
  _output_csv:=/home/chiararipiemo/validation/us_best_pose_imfusion.csv \
  _flush_every:=1
```
```
rosrun us_planner export_probe_pose_Tfixed_for_imfusion.py \
  _probe_frame:=probe_tip \
  _base_frame:=world \
  _output_csv:=/home/chiararipiemo/pose_best_from_robot_for_imfusion.csv

```
### Run the validator recorder 
```
ROS_NAMESPACE=/iiwa \
python3 ~/iiwa_stack_ws/src/us_planner/scripts/us_pose_planner_validator.py \
  _cloud_topic:=/skin_cloud \
  _target_topic:=/us_target_viz \
  _apex_pose_topic:=/us_apex_pose \
  _best_pose_topic:=/us_best_pose \
  _conf_mha_path:=/home/chiararipiemo/iiwa_stack_ws/src/us_planner/volume_confidence.mha \
  _conf_volume_frame:=imfusion \
  _T_imfusion_from_world_mm:="[[1,0,0,-642],[0,0,1,-364],[0,-1,0,-200],[0,0,0,1]]" \
  _conf_agg:=p95 \
  _conf_p_quantile:=0.98 \
  _ray_step_mm:=2.0 \
  _conf_target_radius_mm:=8.0 \
  _conf_target_samples:=20 \
  _baseline_mode:=pca \
  _normal_k:=30 \
  _csv_path:=/home/chiararipiemo/iiwa_stack_ws/src/us_planner/validation_results.csv
```
