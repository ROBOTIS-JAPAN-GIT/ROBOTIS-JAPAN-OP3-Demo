################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(jp_op3_demo)

# set(CMAKE_CXX_FLAGS "-nostartfiles ${CMAKE_CXX_FLAGS}")

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  sensor_msgs
  geometry_msgs
  robotis_controller_msgs
  op3_walking_module_msgs
  op3_action_module_msgs
  cmake_modules
  robotis_math
  op3_ball_detector
  op3_demo
)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(Eigen3 REQUIRED)

## Resolve system dependency on yaml-cpp, which apparently does not
## provide a CMake find_package() module.
## Insert your header file compatible specified path like '#include <yaml-cpp/yaml.h>'
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
  NAMES yaml_cpp.h
  PATHS ${YAML_CPP_INCLUDE_DIRS}
)
find_library(YAML_CPP_LIBRARY
  NAMES YAML_CPP
  PATHS ${YAML_CPP_LIBRARY_DIRS}
)
link_directories(${YAML_CPP_LIBRARY_DIRS})

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    roslib
    std_msgs
    sensor_msgs
    geometry_msgs
    robotis_controller_msgs
    op3_walking_module_msgs
    op3_action_module_msgs
    cmake_modules
    robotis_math
    op3_ball_detector
    op3_demo
  DEPENDS Boost EIGEN3
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

add_executable(jp_op_demo_node 
  # src/demo_node.cpp
  # src/soccer/soccer_demo.cpp
  # src/soccer/ball_tracker.cpp
  # src/soccer/ball_follower.cpp
  # src/action/action_demo.cpp
  # src/vision/vision_demo.cpp
  # src/vision/face_tracker.cpp
  # src/cmd_vel/cmd_vel_node.cpp
  src/cmd_vel_walk.cpp
  src/cmd_vel_walk_node.cpp
)

add_dependencies(jp_op_demo_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  # op_demo_node
  # op3_demo
)

target_link_libraries(jp_op_demo_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  # op_demo_node
  # op3_demo
)

add_executable(jp_self_test_node 
  # src/test_node.cpp
  # src/soccer/soccer_demo.cpp
  # src/soccer/ball_tracker.cpp
  # src/soccer/ball_follower.cpp
  # src/action/action_demo.cpp
  # src/vision/vision_demo.cpp
  # src/vision/face_tracker.cpp
  # src/test/button_test.cpp
  # src/test/mic_test.cpp
  # src/cmd_vel/cmd_vel_node.cpp
  src/cmd_vel_walk.cpp
  src/cmd_vel_walk_node.cpp
)

add_dependencies(jp_self_test_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  # self_test_node
)

target_link_libraries(jp_self_test_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  # self_test_node
)

################################################################################
# Install
################################################################################
install(TARGETS jp_op_demo_node jp_self_test_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY data launch list 
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
