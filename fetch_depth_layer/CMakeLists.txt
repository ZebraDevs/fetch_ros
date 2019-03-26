cmake_minimum_required(VERSION 2.8.12)
project(fetch_depth_layer)

find_package(catkin REQUIRED 
  COMPONENTS
    pluginlib
    roscpp
    image_transport
    cv_bridge
    sensor_msgs
    geometry_msgs
    tf2_ros
    costmap_2d
    nav_msgs
)

find_package(OpenCV 3.2 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    fech_depth_layer
  CATKIN_DEPENDS
    image_transport
    cv_bridge
    roscpp
    sensor_msgs
    tf2_ros
    geometry_msgs
    costmap_2d
    nav_msgs
  DEPENDS
    OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(fetch_depth_layer src/depth_layer.cpp)

target_link_libraries(fetch_depth_layer
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

install(FILES costmap_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS fetch_depth_layer
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

