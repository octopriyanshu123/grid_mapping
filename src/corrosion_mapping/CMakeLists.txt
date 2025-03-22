cmake_minimum_required(VERSION 2.8.3)
project(corrosion_mapping)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  behaviortree_cpp_v3
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs std_srvs behaviortree_cpp_v3 geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#imu_subscriber_node executable
add_executable(trigger_servo_node src/main.cpp)
target_link_libraries(trigger_servo_node ${catkin_LIBRARIES})