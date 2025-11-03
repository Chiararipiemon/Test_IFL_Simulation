cmake_minimum_required(VERSION 3.0.2)
project(iiwa_probe_utils)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  moveit_commander
  tf
  tf2_ros
  visualization_msgs
  sensor_msgs
  tf2_geometry_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/attach_probe_collision.py
  scripts/add_patient_mesh.py
  scripts/plan_execute_sweep.py
  scripts/log_probe_tip_csv.py
  scripts/3_nov/scan_line_to_cartesian_path.py
  scripts/3_nov/pre_touch_p0.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

