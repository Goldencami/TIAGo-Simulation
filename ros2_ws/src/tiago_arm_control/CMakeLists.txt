cmake_minimum_required(VERSION 3.8)
project(tiago_arm_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_common REQUIRED)
find_package(gazebo_msgs REQUIRED)

add_executable(arm_control src/arm_control.cpp)
ament_target_dependencies(arm_control
  rclcpp
  std_msgs
  moveit_ros_planning_interface
  moveit_msgs
  moveit_core
  moveit_common
  gazebo_msgs
)

install(
  TARGETS arm_control
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

add_executable(gripper_control src/gripper_control.cpp)
ament_target_dependencies(gripper_control
  rclcpp
  std_msgs
  moveit_ros_planning_interface
  moveit_msgs
  moveit_core
  moveit_common
  gazebo_msgs
)

install(
  TARGETS gripper_control
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
