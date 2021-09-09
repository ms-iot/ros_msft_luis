cmake_minimum_required(VERSION 2.8.3)
project(ros_msft_luis_move_base)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  move_base_msgs
  ros_msft_luis_msgs
  actionlib_msgs
  actionlib
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES
    ros_msft_luis
  CATKIN_DEPENDS
    roscpp
    move_base_msgs
    ros_msft_luis_msgs
    actionlib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/ros_msft_luis_move_base.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node 
${catkin_LIBRARIES}
)