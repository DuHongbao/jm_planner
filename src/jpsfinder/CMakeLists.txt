PROJECT(jpsfinder)
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}-std=c++11")


find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)

find_package(catkin REQUIRED COMPONENTS
	roscpp roslib   plan_env  cmake_modules cv_bridge image_transport pcl_ros sensor_msgs geometry_msgs nav_msgs quadrotor_msgs dynamic_reconfigure)


catkin_package(
    DEPENDS OpenCV Eigen Boost
    INCLUDE_DIRS include 
    LIBRARIES jpsfinder
    CATKIN_DEPENDS plan_env  roscpp roslib image_transport pcl_ros   )

include_directories(
  SYSTEM 
  include 
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall")
add_library( jpsfinder
src/jpsfinder.cpp)
# add_executable(
#   JPSPlus_finder
#   src/JPSPlus_finder.cpp
# )
target_link_libraries( jpsfinder
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

# include_directories(
#   SYSTEM 
#   ${catkin_INCLUDE_DIRS}
#   ${Eigen_INCLUDE_DIRS}
# )
