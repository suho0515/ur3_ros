cmake_minimum_required(VERSION 2.8.3)
project(ur3_rg2_moveit)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  ur_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

 add_service_files(
   FILES
   RG2.srv
   RG2_Grip.srv
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ur5_control
#  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS system_lib
    roscpp
    std_msgs
    message_runtime
    ur_msgs
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

