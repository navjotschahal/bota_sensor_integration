cmake_minimum_required(VERSION 3.0.2)
project(bota_ft_sensor)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  diagnostic_updater
  diagnostic_msgs
)

# Installs src/bota_ft_sensor as an importable python module (stream protocol
# shared by the host node and, via COPY in the Dockerfile, the container).
catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy geometry_msgs sensor_msgs std_msgs std_srvs diagnostic_updater diagnostic_msgs
)

catkin_install_python(PROGRAMS
  scripts/bota_stream_node.py
  scripts/bota_tare.py
  scripts/bota_stream_probe.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/bota_container.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config docker
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
