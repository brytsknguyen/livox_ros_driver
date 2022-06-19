cmake_minimum_required(VERSION 3.0.2)
project(livox_viz)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  livox_ros_driver
  roscpp
  rospy
  std_msgs
)

## System dependencies are found with CMake's conventions
find_package(OpenMP REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES livox_viz
#  CATKIN_DEPENDS livox_ros_driver roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

# include directories
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

# link directories
link_directories(
	include
	${PCL_LIBRARY_DIRS}
)

## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME} src/livox_viz.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} ${OpenMP_CXX_FLAGS}
)
