cmake_minimum_required(VERSION 3.0.2)
project(ur_am)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
)

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    roscpp
  DEPENDS
    EIGEN3
)

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_executable(demo src/demo.cpp)

target_link_libraries(demo ${catkin_LIBRARIES})