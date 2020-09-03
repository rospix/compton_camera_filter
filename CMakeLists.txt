cmake_minimum_required(VERSION 2.8.3)
project(compton_camera_filter)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fno-diagnostics-color")

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nodelet
  cmake_modules
  gazebo_rad_msgs
  mrs_lib
  rad_msgs
  dynamic_reconfigure
  message_generation
)

generate_dynamic_reconfigure_options(
  cfg/compton_filter.cfg
  cfg/trajectory_planner.cfg
  )

# messages for MPC_TRACKER
add_message_files(DIRECTORY msg FILES

  Swarm.msg
  )

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES

  std_msgs
  geometry_msgs
  sensor_msgs
  )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gdax_interface
   CATKIN_DEPENDS nodelet rospy std_msgs message_runtime gazebo_rad_msgs mrs_lib rad_msgs
   LIBRARIES ComptonFilter TrajectoryPlanner
   DEPENDS Eigen
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  )

add_library(ComptonFilter src/compton_filter.cpp)

add_dependencies(ComptonFilter
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

## Specify libraries to link a library or executable target against
target_link_libraries(ComptonFilter
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

add_library(TrajectoryPlanner src/trajectory_planner.cpp)

add_dependencies(TrajectoryPlanner
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

## Specify libraries to link a library or executable target against
target_link_libraries(TrajectoryPlanner
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )
