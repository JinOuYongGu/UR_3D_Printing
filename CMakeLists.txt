cmake_minimum_required(VERSION 3.0.2)
project(ur_am)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  serial
  moveit_visual_tools
)

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    roscpp
    serial
    moveit_visual_tools
  DEPENDS
    EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  )

add_executable(arduino_serial src/arduino_serial.cpp)
add_executable(normal_print src/normal_print.cpp)

target_link_libraries(arduino_serial ${catkin_LIBRARIES})
target_link_libraries(normal_print ${catkin_LIBRARIES})

SET(CMAKE_BUILD_TYPE Release)