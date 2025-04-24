cmake_minimum_required(VERSION 3.0)
project(topic_preprocessing)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  pcl_conversions
  message_generation
  livox_ros_driver
)
find_package(PCL REQUIRED QUIET)

add_message_files(
    FILES
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
  DEPENDS PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(topic_preprocessing src/main.cpp)
add_dependencies(topic_preprocessing ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(topic_preprocessing ${catkin_LIBRARIES} ${PCL_LIBRARIES})