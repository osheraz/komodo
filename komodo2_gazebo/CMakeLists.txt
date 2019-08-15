cmake_minimum_required(VERSION 2.8.3)
project(komodo2_gazebo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)