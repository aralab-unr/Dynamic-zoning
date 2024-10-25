cmake_minimum_required(VERSION 3.0.2)
project(isaac_sim_zone)


find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  move_base_msgs
  rospy
  sensor_msgs
  std_msgs
  message_generation
)

catkin_python_setup()

add_message_files(
   FILES
   stringarr.msg
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
#  INCLUDE_DIRS include
#  LIBRARIES isaac_ros_navigation_goal
  CATKIN_DEPENDS actionlib geometry_msgs move_base_msgs rospy sensor_msgs std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS 
  isaac_ros_navigation_goal/set_goal.py
  isaac_ros_navigation_goal/robot_tasks.py
  isaac_ros_navigation_goal/Monitor.py
  isaac_ros_navigation_goal/Control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
