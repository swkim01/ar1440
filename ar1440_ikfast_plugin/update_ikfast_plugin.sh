search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=ar1440.srdf
robot_name_in_srdf=ar1440
moveit_config_pkg=ar1440_moveit_config
robot_name=ar1440
planning_group_name=manipulator
ikfast_plugin_pkg=ar1440_ikfast_plugin
base_link_name=base_link
eef_link_name=tool0
ikfast_output_path=/home/ubuntu/catkin_ws/src/ar1440/ar1440_ikfast_plugin/src/ar1440_manipulator_ikfast_solver.cpp

rosrun moveit_kinematics create_ikfast_moveit_plugin.py\
  --search_mode=$search_mode\
  --srdf_filename=$srdf_filename\
  --robot_name_in_srdf=$robot_name_in_srdf\
  --moveit_config_pkg=$moveit_config_pkg\
  $robot_name\
  $planning_group_name\
  $ikfast_plugin_pkg\
  $base_link_name\
  $eef_link_name\
  $ikfast_output_path
