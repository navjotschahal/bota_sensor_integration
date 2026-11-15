cmake_minimum_required(VERSION 3.0.2)
project(bota_ft_sensor)

find_package(catkin REQUIRED)

# Installs src/bota_ft_sensor (stream protocol + client helpers) as an importable
# python module; ROS clients in other packages import it, the Dockerfile COPYs it.
catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/bota_stream_probe.py
  scripts/bota_sensors.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/bota_container.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY docker config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
