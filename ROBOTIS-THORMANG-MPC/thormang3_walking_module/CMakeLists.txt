################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(thormang3_walking_module)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cmake_modules
  eigen_conversions
  robotis_math
  robotis_framework_common
  robotis_controller_msgs
  thormang3_kinematics_dynamics
  thormang3_balance_control
  thormang3_walking_module_msgs
  visualization_msgs
)

find_package(Eigen REQUIRED)
pkg_check_modules(yaml_cpp yaml-cpp REQUIRED)

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES thormang3_walking_module
  CATKIN_DEPENDS roscpp robotis_math robotis_framework_common thormang3_balance_control thormang3_kinematics_dynamics visualization_msgs
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${yaml_cpp_INCLUDE_DIRS}
)

add_library(thormang3_walking_module
  src/thormang3_online_walking.cpp
  src/walking_module.cpp
)

add_dependencies(thormang3_walking_module ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(thormang3_walking_module
  ${catkin_LIBRARIES}
  ${yaml_cpp_LIBRARIES}
)

################################################################################
# Install
################################################################################
install(TARGETS thormang3_walking_module
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

################################################################################
# Test
################################################################################
