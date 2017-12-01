cmake_minimum_required(VERSION 2.8.3)
project(tree_point_and_click)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  actionlib
  cmake_modules
  eigen_conversions
  geometry_msgs
  interactive_markers
  pcl_conversions
  pcl_ros
  rail_grasp_calculation_msgs
  rail_manipulation_msgs
  roscpp
  sensor_msgs
  tf
  tf_conversions
  tf2_geometry_msgs
  tf2_ros
)

find_package(Eigen REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################


###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(generator src/generator.cpp)
add_executable(clicker src/clicker.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(generator ${catkin_LIBRARIES} ${EIGEN_INCLUDE_DIRS})
target_link_libraries(clicker ${catkin_LIBRARIES})

## Add cmake target dependencies of the executable/library
add_dependencies(generator rail_manipulation_msgs_gencpp rail_grasp_calculation_msgs_gencpp)
#add_dependencies(clicker rail_manipulation_msgs_gencpp rail_grasp_calculation_msgs_gencpp)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS generator clicker
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Copy header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

## Copy launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
