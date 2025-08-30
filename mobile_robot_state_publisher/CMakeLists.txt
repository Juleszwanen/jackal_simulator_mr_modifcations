cmake_minimum_required(VERSION 2.8.3)
project(mobile_robot_state_publisher)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  tf
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs tf tf2 tf2_ros tf2_geometry_msgs nav_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# ---- Executables ----
add_executable(mobile_robot_state_publisher_node
  src/mobile_robot_state_publisher_node.cpp
)
target_link_libraries(mobile_robot_state_publisher_node
  ${catkin_LIBRARIES}
)
if(catkin_EXPORTED_TARGETS)
  add_dependencies(mobile_robot_state_publisher_node ${catkin_EXPORTED_TARGETS})
endif()

add_executable(mobile_robot_state_publisher_node_mr
  src/mobile_robot_state_publisher_node_mr.cpp
)
target_link_libraries(mobile_robot_state_publisher_node_mr
  ${catkin_LIBRARIES}
)
if(catkin_EXPORTED_TARGETS)
  add_dependencies(mobile_robot_state_publisher_node_mr ${catkin_EXPORTED_TARGETS})
endif()

#############
## Install ##
#############

install(TARGETS
  mobile_robot_state_publisher_node
  mobile_robot_state_publisher_node_mr
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
