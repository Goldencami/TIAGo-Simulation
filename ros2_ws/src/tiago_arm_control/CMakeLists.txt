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
find_package(linkattacher_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# generate service files
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PickObject.srv"
)

ament_export_dependencies(rosidl_default_runtime)

# add executable
add_executable(TIAGOController src/TIAGOController.cpp)
ament_target_dependencies(TIAGOController
  rclcpp
  std_msgs
  moveit_ros_planning_interface
  moveit_msgs
  moveit_core
  moveit_common
  gazebo_msgs
  linkattacher_msgs
)

rosidl_target_interfaces(TIAGOController
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

install(
  TARGETS TIAGOController
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
