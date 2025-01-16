cmake_minimum_required(VERSION 3.0.2)
project(follow_me)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  rospy
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  visualization_msgs
  genmsg
  tf
)

find_package(PCL REQUIRED)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs rospy nav_msgs tf2 tf2_ros tf2_geometry_msgs
)

## Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(laser_data src/laser_data.cpp)
add_executable(tracker src/tracker.cpp)
add_executable(scan_to_map src/scan_to_map.cpp)
add_executable(sensor_model_node src/sensor_model_node.cpp src/localization.cpp)
add_executable(localization_node src/localization_node.cpp src/localization.cpp)

## Add dependencies to ensure proper build order
add_dependencies(laser_data ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(tracker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(scan_to_map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Link the executable to the required libraries
target_link_libraries(laser_data
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(tracker
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(scan_to_map
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(sensor_model_node ${catkin_LIBRARIES})
target_link_libraries(localization_node ${catkin_LIBRARIES})