cmake_minimum_required(VERSION 2.8.3)
project(interbotix_xsarm_gazebo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  effort_controllers
  gazebo_ros
  gazebo_ros_control
  interbotix_xsarm_descriptions
  joint_state_controller
  joint_trajectory_controller
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(
  CATKIN_DEPENDS controller_manager effort_controllers gazebo_ros gazebo_ros_control interbotix_xsarm_descriptions joint_state_controller joint_trajectory_controller
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## Specify libraries to link a library or executable target against
## Add cmake target dependencies of the executable
add_executable(gravcomp_torques src/gravcomp_torques.cpp)
target_link_libraries(gravcomp_torques ${catkin_LIBRARIES})
add_dependencies(gravcomp_torques ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})