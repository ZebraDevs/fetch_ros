cmake_minimum_required(VERSION 3.0.2)
project(fetch_maps)

find_package(catkin REQUIRED)
catkin_package()

install(
  DIRECTORY maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
