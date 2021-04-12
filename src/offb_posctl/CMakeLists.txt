cmake_minimum_required(VERSION 2.8.3)
project(offb_posctl)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  mavros
  roscpp
  rospy
  message_generation
  dynamic_reconfigure
)

 add_message_files(
   FILES
   controlstate.msg
 )
# Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   geometry_msgs#   std_msgs
 )

generate_dynamic_reconfigure_options(
        cfg/offb_cfg.cfg
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES offb_posctl
#  CATKIN_DEPENDS geometry_msgs mavros roscpp rospy
#  DEPENDS system_lib
   CATKIN_DEPENDS message_runtime
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        src/
        include/
)

add_library(thelib
        src/PID.h
        src/PID.cpp
        src/Parameter.h
        src/Parameter.cpp
        include/offb_Config.h
        )


add_executable(offb_posctl src/offb_posctl.cpp )
target_link_libraries(offb_posctl ${catkin_LIBRARIES} thelib)





