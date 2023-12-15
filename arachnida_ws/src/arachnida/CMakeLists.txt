cmake_minimum_required(VERSION 3.0.2)
project(arachnida)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  std_msgs
  message_generation
)

# find_package(CUDA REQUIRED)
find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INLCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN3_LIBRARIES})
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(PCL REQUIRED)

# set(VTK_DIR "/usr/include/vtk-6.3")
set(Ceres_DIR "/usr/local/lib/cmake/Ceres")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # Creates the build/compile_commands.json file so that intellisense works with neovim (clangd + lspconfig + mason.nvim + nvim-cmp + etc)

find_package(VTK REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${VTK_USE_FILE}
  ${CERES_INCLUDE_DIRS}
)

add_message_files(
	FILES 
	Obstacle.msg
	ObstacleList.msg
)

generate_messages(
	DEPENDENCIES 
	std_msgs
)

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
	# INCLUDE_DIRS include
  # LIBRARIES arachnida_nodelets
#  CATKIN_DEPENDS nodelet roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
#include_directories(
#  include
#  ${catkin_INCLUDE_DIRS}
#)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(publisher_sample_node src/publisher_sample_node.cpp)
add_executable(velodyneReaderNode
              src/lidar/velodyneReaderNode.cpp 
              src/lidar/velodyneSocketReader.cpp 
              src/lidar/velodynePCAPReader.cpp 
              src/lidar/velodyneUtils.cpp
              src/floam_cpu/laserMappingClass.cpp 
              src/floam_cpu/laserProcessingClass.cpp 
              src/floam_cpu/odomEstimationClass.cpp 
              src/floam_cpu/lidarOptimisation.cpp
              src/object_detection/objCluster.cpp
              src/object_detection/objPointCloudProcessor.cpp
              src/object_detection/objRansac.cpp
              src/object_detection/objRender.cpp
	      src/path_planning/artificialPotentialField.cpp
)
add_executable(robotStateNode src/robot_state/robotStateNode.cpp)
add_executable(gaitEnergeticsNode src/gait_energetics/gait_energetics.cpp
              src/gait_energetics/main.cpp)


target_link_libraries(publisher_sample_node
  ${catkin_LIBRARIES}
)

target_link_libraries(velodyneReaderNode
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  Ceres::ceres
)

target_link_libraries(robotStateNode
  ${catkin_LIBRARIES}
)

target_link_libraries(gaitEnergeticsNode
  ${catkin_LIBRARIES}
)

## Declare Arachnida plugin library for nodelets
add_library(arachnida_nodelets 
  # Add the actual nodelet .cpp files
  src/hello_world.cpp # Sample nodelet
  src/nodelets/floamNodelet.cpp
  src/nodelets/objectDetectionNodelet.cpp
  src/nodelets/pathPlanningNodelet.cpp
  src/nodelets/robotStateNodelet.cpp
  # Add the F-LOAM .cpp files that are used by the the F-LOAM nodelet (failing to add these will cause the nodelets to break)
  src/floam_cpu/laserMappingClass.cpp 
  src/floam_cpu/laserProcessingClass.cpp 
  src/floam_cpu/odomEstimationClass.cpp 
  src/floam_cpu/lidarOptimisation.cpp
  # Add the object detection .cpp files that are used by the object detection nodelet (failing to add these will cause the nodelets to break)
  src/object_detection/objCluster.cpp
  src/object_detection/objPointCloudProcessor.cpp
  src/object_detection/objRansac.cpp
  src/object_detection/objRender.cpp
)

## Link Arachnida plugin library
target_link_libraries(arachnida_nodelets 
	${catkin_LIBRARIES} 
	${PCL_LIBRARIES} 
	Ceres::ceres
	${EIGEN3_LIBRARIES}
)
