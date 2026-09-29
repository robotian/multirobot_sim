cmake_minimum_required(VERSION 3.8)
project(plant_cutter_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

# Generate the action interface
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/CutStem.action"
  DEPENDENCIES action_msgs
)

ament_package()