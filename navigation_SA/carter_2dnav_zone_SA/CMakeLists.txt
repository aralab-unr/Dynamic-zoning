cmake_minimum_required(VERSION 3.0.2)
project(carter_2dnav_zone_SA)

find_package(catkin REQUIRED COMPONENTS
  move_base
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

