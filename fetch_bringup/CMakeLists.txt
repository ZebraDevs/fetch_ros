cmake_minimum_required(VERSION 2.8.3)
project(fetch_bringup)

find_package(catkin)
catkin_package()

install(
  DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  PROGRAMS scripts/controller_reset.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
