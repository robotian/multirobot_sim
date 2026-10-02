cmake_minimum_required(VERSION 3.20)
project(status_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Harvest.action"
  "action/ManipulatorTask.action"
  "action/OperateUnloader.action"
  "msg/BasicStatus.msg"
  "msg/BatteryStatus.msg"
  "msg/ImageDetection.msg"
  "msg/ImageDetectionPose.msg"
  "msg/Job.msg"
  "msg/NodeStatus.msg"
  "msg/NodeDetails.msg"
  "msg/RobotStatus.msg"
  "msg/Status.msg"
  "msg/TopicDetails.msg"
  "msg/TopicStatus.msg"
  "msg/SubTask.msg"
  "msg/Task.msg"
  "msg/WayPoint.msg"
  "msg/DockGoal.msg"
  "msg/UndockGoal.msg"
  "msg/DriveFeedback.msg"
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
