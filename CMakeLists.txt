cmake_minimum_required(VERSION 2.8.3)
project(odometry_motion_model)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS nav_msgs roscpp)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES nav_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES odometry_motion_model
  CATKIN_DEPENDS nav_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

## Declare a C++ library
add_library(odometry_motion_model src/odometrymotionmodel.cpp)
add_executable(odometry_motion_model_node src/main.cpp)
target_link_libraries(odometry_motion_model_node odometry_motion_model ${catkin_LIBRARIES} ${Eigen_LIBRARIES})
