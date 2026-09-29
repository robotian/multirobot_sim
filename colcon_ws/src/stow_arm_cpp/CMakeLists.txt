cmake_minimum_required(VERSION 3.8)
project(stow_arm_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Assuming these are your custom message packages
find_package(plant_cutter_msgs REQUIRED)
find_package(serial_interfaces REQUIRED)

# ==========================================
# 1. Target: Stow Arm Node
# ==========================================
add_executable(stow_arm_node src/stow_arm_node.cpp)
target_include_directories(stow_arm_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(stow_arm_node
  rclcpp
  moveit_ros_planning_interface
)

# ==========================================
# 2. Target: Grid Cutter Action Server
# ==========================================
add_executable(grid_cutter_action_server src/grid_cutter_action_server.cpp)
target_include_directories(grid_cutter_action_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(grid_cutter_action_server
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  geometry_msgs
  control_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
  plant_cutter_msgs
  serial_interfaces
)

# ==========================================
# Install Instructions
# ==========================================
install(TARGETS 
  stow_arm_node 
  grid_cutter_action_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()