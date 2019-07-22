cmake_minimum_required(VERSION 2.8.3)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
project(octomaptest)

SET(CMAKE_CXX_FLAGS -std=c++11)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#INCLUDE_DIRECTORIES ( "$ENV{EIGEN3_INCLUDE_DIR}" )
find_package(Eigen3 REQUIRED)
find_package(GeographicLib REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  std_srvs
  std_msgs
  message_generation
  pcl_conversions
  pcl_ros
  image_transport
  sensor_msgs
  octomap_ros
  octomap_msgs
)


FIND_PACKAGE( PCL REQUIRED COMPONENTS common io filters)
ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
FIND_PACKAGE( octomap REQUIRED )
INCLUDE_DIRECTORIES( ${OCTOMAP_INCLUDE_DIRS} )
find_package(OpenCV REQUIRED)


catkin_package(
#   LIBRARIES drone_sensor
   CATKIN_DEPENDS  message_runtime
#  DEPENDS system_lib
)


include_directories(
${PCL_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIR}
${catkin_INCLUDE_DIRS}
${GeographicLib_INCLUDE_DIRS}
)

ADD_EXECUTABLE(octotest
  src/main.cpp
  src/pchandle.cpp
)
target_link_libraries(octotest
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

set_target_properties(  octotest
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY
    ${PROJECT_SOURCE_DIR}/bin
)

