################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(cable_reel)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  angles
  message_generation
  message_runtime
  dynamixel_sdk
  geometry_msgs
  tf
  tf2
  tf2_ros
  livox_ros_driver
)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message(STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
message(STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
message(STATUS "PCL_DEFINITIONS:" ${PCL_DEFINITIONS})

################################################################################
# Declare ROS messages, services and actions
################################################################################
## Generate messages in the 'msg' folder
add_message_files(
  FILES
    SetPosition.msg
    GetServo.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GetPosition.srv

)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    std_msgs
)

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    dynamixel_sdk
#  DEPENDS system_lib
)

################################################################################
# Build
################################################################################
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(include)

add_executable(merge_livox_msg src/merge_livox_msg.cpp)
add_dependencies(merge_livox_msg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(merge_livox_msg ${catkin_LIBRARIES})

add_executable(read_write_node src/read_write_node.cpp)
add_dependencies(read_write_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(read_write_node ${catkin_LIBRARIES})

add_executable(servo_node src/servo_node.cpp)
add_dependencies(servo_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(servo_node ${catkin_LIBRARIES})

add_executable(torque_off src/torque_off.cpp)
add_dependencies(torque_off ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(torque_off ${catkin_LIBRARIES})

add_executable(poler_control src/poler_control.cpp)
add_dependencies(poler_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(poler_control ${catkin_LIBRARIES})

add_executable(cable_reel_control src/cable_reel_control.cpp)
add_dependencies(cable_reel_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cable_reel_control ${catkin_LIBRARIES})


################################################################################
# Install
################################################################################
## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS merge_livox_msg read_write_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
