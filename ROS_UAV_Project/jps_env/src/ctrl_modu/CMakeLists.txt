cmake_minimum_required(VERSION 2.8.3)
project(ctrl_modu)

SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  plan_env
  modu_msgs
)
find_package(Eigen3 REQUIRED)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ctrl_modu
  CATKIN_DEPENDS roscpp rospy std_msgs  plan_env
  DEPENDS modu_msgs#system_lib 
)

include_directories(
  SYSTEM 
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_executable(
  ctrl_modu
  src/ctrl_modu.cpp
)

 target_link_libraries(ctrl_modu
   ${catkin_LIBRARIES}
 )

