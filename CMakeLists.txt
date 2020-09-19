cmake_minimum_required(VERSION 3.0.2)
project(ur_am)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  serial
)

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    roscpp
    serial
  DEPENDS
    EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  )

add_executable(arduino_sender src/arduino_sender.cpp)
add_executable(position_editor src/position_editor.cpp)
add_executable(position_monitor src/position_monitor.cpp)
add_executable(arduino_serial src/arduino_serial.cpp)
add_executable(normal_print src/normal_print.cpp)
add_executable(circle_demo src/circle_demo.cpp)
add_executable(cubic_demo src/cubic_demo.cpp)

target_link_libraries(arduino_sender ${catkin_LIBRARIES})
target_link_libraries(position_editor ${catkin_LIBRARIES})
target_link_libraries(position_monitor ${catkin_LIBRARIES})
target_link_libraries(arduino_serial ${catkin_LIBRARIES})
target_link_libraries(normal_print ${catkin_LIBRARIES})
target_link_libraries(circle_demo ${catkin_LIBRARIES})
target_link_libraries(cubic_demo ${catkin_LIBRARIES})