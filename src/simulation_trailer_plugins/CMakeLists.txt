cmake_minimum_required(VERSION 3.8)
project(simulation_trailer_plugins)

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(prius_hitch_joint_plugin SHARED src/hitch_joint_plugin.cpp)
target_compile_features(prius_hitch_joint_plugin PUBLIC cxx_std_17)
target_include_directories(prius_hitch_joint_plugin PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(prius_hitch_joint_plugin ${GAZEBO_LIBRARIES})
ament_target_dependencies(prius_hitch_joint_plugin gazebo_ros rclcpp std_msgs)

install(TARGETS prius_hitch_joint_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()
