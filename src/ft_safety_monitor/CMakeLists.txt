cmake_minimum_required(VERSION 3.0.2)
project(ft_safety_monitor)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  std_msgs
  std_srvs
  actionlib
  control_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/ft_safety_monitor.py
  scripts/publish_fake_wrench.py
  scripts/mock_trajectory_action_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
