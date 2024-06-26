cmake_minimum_required(VERSION 2.8.3)
project(apf_planner)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(CMAKE_PREFIX_PATH /opt/ros/noetic;/usr/local;$ENV{HOME}/Prometheus/devel/;$ENV{HOME}/planner_ws/devel;$ENV{HOME}/mapping_ws/devel;$ENV{HOME}/mapping_ws/devel_isolated;$ENV{HOME}/tool_ws/devel)
message(STATUS "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
       roscpp
       rospy
       std_msgs
	   sensor_msgs
	   geometry_msgs
	   nav_msgs
       visualization_msgs
	   prometheus_msgs
          mavros_msgs
	)

catkin_package(
       INCLUDE_DIRS include
       LIBRARIES Local_Planning
       #  DEPENDS system_lib
	)

include_directories(
       SYSTEM
       include
       ${PROJECT_SOURCE_DIR}/include
       ${catkin_INCLUDE_DIRS}
       ${Eigen3_INCLUDE_DIRS}
       ${PCL_INCLUDE_DIRS}
		$ENV{HOME}/Prometheus/Modules/common/include
     )
link_directories(${PCL_LIBRARY_DIRS})

add_executable(local_planner_main 
	src/planner_node.cpp
	src/local_planning.cpp
	src/apf.cpp
)

target_link_libraries(local_planner_main
       ${catkin_LIBRARIES}
	   ${PCL_LIBRARIES}
	)
