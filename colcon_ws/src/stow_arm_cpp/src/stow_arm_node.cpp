#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  
  auto move_group_node = rclcpp::Node::make_shared("stow_arm_node", node_options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread spinner = std::thread([&executor]() { executor.spin(); });

  RCLCPP_INFO(move_group_node->get_logger(), "Node initialized. Namespace is: %s", move_group_node->get_namespace());

  RCLCPP_INFO(move_group_node->get_logger(), "Configuring MoveGroupInterface Options...");
  moveit::planning_interface::MoveGroupInterface::Options opt(
      "arm_0", 
      "robot_description", 
      move_group_node->get_namespace()
  );

  RCLCPP_INFO(move_group_node->get_logger(), "Constructing MoveGroupInterface (Waiting for Action Server)...");
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, opt);

  RCLCPP_INFO(move_group_node->get_logger(), "MoveGroupInterface successfully constructed and connected!");

  // --- INCREASE SPEED HERE ---
  // Factors range from 0.0 to 1.0 (1.0 = 100% of the max limits defined in joint_limits.yaml)
  RCLCPP_INFO(move_group_node->get_logger(), "Setting speed and acceleration to 100%% of limits...");
  move_group.setMaxVelocityScalingFactor(1.0);
  move_group.setMaxAccelerationScalingFactor(1.0);
  // ---------------------------

  RCLCPP_INFO(move_group_node->get_logger(), "Setting target pose to 'stow'...");
  move_group.setNamedTarget("stow");

  RCLCPP_INFO(move_group_node->get_logger(), "Planning trajectory...");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success) {
    RCLCPP_INFO(move_group_node->get_logger(), "Valid plan found! Executing...");
    move_group.execute(my_plan);
    RCLCPP_INFO(move_group_node->get_logger(), "Arm successfully stowed.");
  } else {
    RCLCPP_ERROR(move_group_node->get_logger(), "Failed to find a valid plan to 'stow'.");
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}