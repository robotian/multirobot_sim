#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  
  auto move_group_node = rclcpp::Node::make_shared("stow_arm_node", node_options);

  // Same parameter names/defaults as grid_cutter_action_server; stow.launch.py feeds them from the same
  // config/grid_cutter_params.yaml + config/robots/<namespace>.yaml, so both nodes agree per robot.
  auto param = [&](const std::string & name, const auto & def) {
      if (!move_group_node->has_parameter(name)) {move_group_node->declare_parameter(name, def);}
      return move_group_node->get_parameter(name);
    };
  const std::string group_name = param("move_group", std::string("arm_0")).as_string();
  const std::string stow_pose = param("stow_pose", std::string("stow")).as_string();
  // YAML may give "15" (integer) for a double parameter, as the shared config does.
  auto as_double = [](const rclcpp::Parameter & p) {
      return p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER ?
             static_cast<double>(p.as_int()) : p.as_double();
    };
  const double vel_scale = as_double(param("moveit_vel_scale", 0.5));
  const double acc_scale = as_double(param("moveit_acc_scale", 0.5));
  const double planning_time = as_double(param("moveit_planning_time", 5.0));

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread spinner = std::thread([&executor]() { executor.spin(); });

  RCLCPP_INFO(move_group_node->get_logger(), "Node initialized. Namespace is: %s", move_group_node->get_namespace());

  RCLCPP_INFO(move_group_node->get_logger(), "Configuring MoveGroupInterface Options...");
  moveit::planning_interface::MoveGroupInterface::Options opt(
      group_name,
      "robot_description", 
      move_group_node->get_namespace()
  );

  RCLCPP_INFO(move_group_node->get_logger(), "Constructing MoveGroupInterface (Waiting for Action Server)...");
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, opt);

  RCLCPP_INFO(move_group_node->get_logger(), "MoveGroupInterface successfully constructed and connected!");

  RCLCPP_INFO(
    move_group_node->get_logger(), "Group '%s', pose '%s', velocity/acceleration scaling %.2f/%.2f.",
    group_name.c_str(), stow_pose.c_str(), vel_scale, acc_scale);
  move_group.setMaxVelocityScalingFactor(vel_scale);
  move_group.setMaxAccelerationScalingFactor(acc_scale);
  move_group.setPlanningTime(planning_time);

  if (!move_group.setNamedTarget(stow_pose)) {
    RCLCPP_ERROR(
      move_group_node->get_logger(), "Named target '%s' is not defined for group '%s' (check the SRDF).",
      stow_pose.c_str(), group_name.c_str());
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  RCLCPP_INFO(move_group_node->get_logger(), "Planning trajectory...");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  int rc = 1;

  if (success) {
    RCLCPP_INFO(move_group_node->get_logger(), "Valid plan found! Executing...");
    if (move_group.execute(my_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(move_group_node->get_logger(), "Arm successfully stowed.");
      rc = 0;
    } else {
      RCLCPP_ERROR(move_group_node->get_logger(), "Execution of the stow trajectory failed.");
    }
  } else {
    RCLCPP_ERROR(move_group_node->get_logger(), "Failed to find a valid plan to '%s'.", stow_pose.c_str());
  }

  rclcpp::shutdown();
  spinner.join();
  return rc;
}
