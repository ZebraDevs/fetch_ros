cmake_minimum_required(VERSION 3.0.2)
project(fetch_calibration)

find_package(catkin)
catkin_package()

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(
  PROGRAMS scripts/camera_reconfigure.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(
   PROGRAMS scripts/calibrate_robot
   DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
