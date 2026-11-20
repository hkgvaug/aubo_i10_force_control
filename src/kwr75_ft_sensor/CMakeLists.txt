cmake_minimum_required(VERSION 3.0.2)
project(kwr75_ft_sensor)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  std_srvs
)

catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS
  scripts/kwr75_ft_sensor_node.py
  scripts/kwr75_probe.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY urdf meshes launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Parser tests are plain unittest and do not need ROS at build time:
#   cd src/kwr75_ft_sensor && python3 -m unittest discover -s test -v
