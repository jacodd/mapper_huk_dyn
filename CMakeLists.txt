################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(mapper_dyn_huk)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  dynamixel_workbench_msgs
  dynamixel_workbench_toolbox
  cmake_modules
  rospy
  std_msgs
  nav_msgs
  message_generation
  pcl_ros
  pcl_conversions
)

find_package(Eigen3 REQUIRED)

# Resolve system dependency on yaml-cpp, which apparently does not
# provide a CMake find_package() module.
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

find_package(octomap REQUIRED)
add_definitions(-DOCTOMAP_NODEBUGOUT)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

add_service_files(
        FILES
        espeleoSaveOctomap.srv
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        sensor_msgs
        nav_msgs
)

################################################################################
## Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs dynamixel_workbench_msgs trajectory_msgs dynamixel_workbench_toolbox cmake_modules
 DEPENDS EIGEN3 OCTOMAP
)

set(LINK_LIBS
  ${OCTOMAP_LIBRARIES}
  ${catkin_LIBRARIES}
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

# add_executable(scan2pc2 src/scan2pc2.cpp)
# target_link_libraries(scan2pc2 ${catkin_LIBRARIES})

add_executable(pcl_concatenate_360_hok src/pcl_concatenate.cpp)
target_link_libraries(pcl_concatenate_360_hok ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(espeleo_save_octomap_service_hok src/octomap_saver_service.cpp)
target_link_libraries(espeleo_save_octomap_service_hok ${catkin_LIBRARIES} ${LINK_LIBS})

add_executable(point_cloud_converter_360_hok src/pc2_to_pc_converter.cpp)
target_link_libraries(point_cloud_converter_360_hok ${catkin_LIBRARIES})

#################################################################################
## Install
#################################################################################
# install(TARGETS dynamixel_workbench_controllers find_dynamixel
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )

# install(DIRECTORY launch config
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

################################################################################
# Test
################################################################################
