cmake_minimum_required(VERSION 2.8.3)
project(agvs_keyboard_controller)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs

  std_msgs
  sensor_msgs  
  diagnostic_msgs
  diagnostic_updater
  robotnik_msgs
  ackermann_msgs
  std_srvs)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS
  LIBRARIES 
  CATKIN_DEPENDS roscpp geometry_msgs robotnik_msgs ackermann_msgs 
  DEPENDS


)

###########
## Build ##
###########

add_executable(vel_publisher src/publisher.cpp)
target_link_libraries(vel_publisher ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
install(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS vel_publisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark all other useful stuff for installation
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

