cmake_minimum_required(VERSION 3.0.2)
project(first_mission)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pluginlib
  rosbag
  roscpp
  rospy
  tf
  std_msgs
  navigation_manager_msgs
  state_machine_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES first_mission
 CATKIN_DEPENDS pluginlib rosbag roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(include
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(first_mission_node src/execute_mission.cpp )
add_executable(first_mission_map_node src/execute_mission_map.cpp )
add_executable(first_mission_map2odom_node src/execute_mission_map2odom.cpp )

add_dependencies(first_mission_node ${catkin_EXPORTED_TARGETS})
add_dependencies(first_mission_map_node ${catkin_EXPORTED_TARGETS})
add_dependencies(first_mission_map2odom_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(first_mission_node
${catkin_LIBRARIES} )
target_link_libraries(first_mission_map_node
${catkin_LIBRARIES} )
target_link_libraries(first_mission_map2odom_node
${catkin_LIBRARIES} )