cmake_minimum_required(VERSION 3.0.2)
project(bim_aubo_adapter)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/bim_target_bridge.py
  scripts/moveit_pose_executor.py
  scripts/publish_components_json.py
  scripts/aubo_canbus_stream_node.py
  scripts/aubo_cartesian_force_servo_node.py
  scripts/aubo_i10_kinematics.py
  scripts/cartesian_admittance_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
