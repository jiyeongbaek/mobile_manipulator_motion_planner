cmake_minimum_required(VERSION 2.8.3)
project(ros_rrt_planner CXX)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  trac_ik_lib
  kdl_parser
)

LIST( APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/../rbdl" )
LIST( APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMake" )
INCLUDE(CMake/FindRBDL.cmake)

find_package (Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(RBDL REQUIRED)
catkin_package(
  CATKIN_DEPENDS
    kdl_parser
    trac_ik_lib
)

link_directories(${Boost_LIBRARY_DIR})
set(SRCS
    src/controller.cpp
    src/RRT_planner.cpp
    src/util.cpp
    src/ROS_bridge.cpp
    src/NHRRT_planner.cpp
    src/traj.cpp
)

set(HEADS
    include/controller.h
    include/fwd.h
    include/NodeTree.h
    include/RRT_planner.h
    include/NHRRT_planner.h
    include/RRTNode.h
    include/util.h
    include/ROS_bridge.hpp
  
)

add_library(ros_controller 
  ${SRCS} 
  ${HEADS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
  ${RBDL_INCLUDE_DIRS}
)
add_definitions (-Ddof=7)
add_executable(${PROJECT_NAME} 
  src/main.cpp 
)

target_link_libraries(${PROJECT_NAME}
  ros_controller
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  rbdl
  trajectory
  )
