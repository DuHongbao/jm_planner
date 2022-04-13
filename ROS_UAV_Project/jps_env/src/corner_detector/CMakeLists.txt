PROJECT(local_corner)
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")




find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)

find_package(catkin REQUIRED COMPONENTS
    roscpp roslib cmake_modules cv_bridge image_transport pcl_ros sensor_msgs geometry_msgs nav_msgs quadrotor_msgs dynamic_reconfigure)

catkin_package(
    DEPENDS OpenCV Eigen Boost
    CATKIN_DEPENDS roscpp roslib image_transport pcl_ros  )

include_directories(
  SYSTEM 
  #include 
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)


add_executable(
  local_corner
  src/local_corner.cpp
)
target_link_libraries( local_corner
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

include_directories(
  SYSTEM 
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

