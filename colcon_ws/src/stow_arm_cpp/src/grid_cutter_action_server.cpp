// grid_cutter_action_server.cpp
//
// CutStem action server for a Clearpath robot with a Kinova Gen3 / Gen3 Lite arm.
// All patch/approach/drop positions are positions of the TOOL frame (tool_link, a static transform from the arm's
// end-effector link, see tool_xyz/tool_rpy), not of the end-effector link itself: the gripper fingers sit at a
// different place on each arm/gripper combination.
// Sweeps a grid of patches: MoveIt to an approach pose, MoveIt Servo (twist
// streaming) into the patch, close gripper, run the pruner, retract, drop.

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <future>
#include <iomanip>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "moveit/robot_state/robot_state.hpp"
#include <Eigen/Geometry>

// MoveIt Planning Scene headers
#include "moveit/planning_scene_interface/planning_scene_interface.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"

// Action headers
#include "control_msgs/action/gripper_command.hpp"
#include "serial_interfaces/action/send_integer.hpp"
#include "plant_cutter_msgs/action/cut_stem.hpp"

using namespace std::chrono_literals;


enum class PatchStatus { Pending, Active, Cut, Failed };

struct Patch
{
  int id;
  double x;
  double y;
  PatchStatus status;
};

struct GridConfig
{
  // Zone / geometry
  double x_min, x_max, y_min, y_max;
  double z_cut, safe_z;
  double dx, dy;
  double approach_offset;
  bool approach_from_patch_row;
  std::array<double, 4> grasp_q;  // x, y, z, w (normalized)
  std::vector<double> preferred_joints;  // IK seed, in move group joint order (empty = current state)

  // Reachable circle at z_cut (patches outside are dropped)
  bool limit_to_reach_circle;
  double reach_max_radius, reach_radial_step, reach_ik_timeout, reach_margin;
  int reach_angle_samples;

  // Servo
  double servo_kp, servo_max_vel, pose_tol, servo_timeout;

  // Gripper / pruner
  double gripper_open, gripper_close, gripper_max_effort, gripper_timeout;
  bool verify_grasp;
  int pruner_command;
  double pruner_timeout;
  double pushing_dist;

  // Sequencing
  int max_patch_attempts;
  int max_consecutive_failures;
  int moveit_attempts;
  std::string drop_pose, stow_pose;
  std::vector<double> drop_joints;  // optional joint-space drop configuration (empty = use the named drop_pose)
  double drop_lower_distance;       // how far the tool is servoed down into the unloader box before releasing (m)
};

class GridCutterActionServer : public rclcpp::Node
{
public:
  using CutStem = plant_cutter_msgs::action::CutStem;
  using GoalHandleCutStem = rclcpp_action::ServerGoalHandle<CutStem>;
  using GripperCommand = control_msgs::action::GripperCommand;
  using SendInteger = serial_interfaces::action::SendInteger;

  explicit GridCutterActionServer(const rclcpp::NodeOptions & options)
  : Node("grid_cutter_action_server", options)
  {
    declare_all_parameters();

    base_frame_ = get_string("base_frame");
    ee_link_ = get_string("ee_link");
    tool_link_ = get_string("tool_link");
    load_tool_offset();

    // Dedicated sub-node for MoveIt to prevent executor deadlocks
    moveit_node_ = rclcpp::Node::make_shared(
      "grid_cutter_moveit_worker", this->get_namespace(), options);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

    // tool_link: static transform ee_link -> tool_link (lands in the robot's tf_static via the launch remapping)
    if (get_bool("publish_tool_tf")) {
      tool_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = this->get_clock()->now();
      t.header.frame_id = ee_link_;
      t.child_frame_id = tool_link_;
      t.transform.translation.x = ee_T_tool_.translation().x();
      t.transform.translation.y = ee_T_tool_.translation().y();
      t.transform.translation.z = ee_T_tool_.translation().z();
      const Eigen::Quaterniond q(ee_T_tool_.rotation());
      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();
      tool_tf_broadcaster_->sendTransform(t);
      RCLCPP_INFO(
        this->get_logger(), "Published static TF %s -> %s (xyz %.3f %.3f %.3f).", ee_link_.c_str(),
        tool_link_.c_str(), t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);
    }

    // Publishers
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      get_string("servo_twist_topic"), 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "cutting_grid_markers", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());

    // Action clients
    cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    gripper_client_ = rclcpp_action::create_client<GripperCommand>(
      this, get_string("gripper_action"), cb_group_);
    pruner_client_ = rclcpp_action::create_client<SendInteger>(
      this, get_string("pruner_action"), cb_group_);

    // Servo timer
    servo_timer_ = this->create_wall_timer(
      10ms, std::bind(&GridCutterActionServer::servo_timer_callback, this), cb_group_);

    // MoveGroupInterface on the worker node
    RCLCPP_INFO(this->get_logger(), "Initializing MoveGroupInterface on dedicated worker node...");
    moveit::planning_interface::MoveGroupInterface::Options opt(
      get_string("move_group"), "robot_description", moveit_node_->get_namespace());
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(moveit_node_, opt);
    move_group_->setMaxVelocityScalingFactor(get_double("moveit_vel_scale"));
    move_group_->setMaxAccelerationScalingFactor(get_double("moveit_acc_scale"));
    move_group_->setPlanningTime(get_double("moveit_planning_time"));
    move_group_->setNumPlanningAttempts(static_cast<unsigned int>(get_int("moveit_planning_attempts")));
    move_group_->setGoalPositionTolerance(get_double("goal_position_tolerance"));
    move_group_->setGoalOrientationTolerance(get_double("goal_orientation_tolerance"));

    // Initialize Planning Scene Interface
    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    // Load initial parameters and sync scene objects
    load_and_add_collision_objects();

    action_server_ = rclcpp_action::create_server<CutStem>(
      this, "cut_stem",
      std::bind(&GridCutterActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&GridCutterActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&GridCutterActionServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Grid Cutter Action Server initialized and waiting for goals.");
  }

  ~GridCutterActionServer() override
  {
    shutting_down_ = true;
    stop_servo();
    if (execution_thread_.joinable()) {
      execution_thread_.join();
    }
  }

  rclcpp::Node::SharedPtr get_moveit_node() const { return moveit_node_; }

private:
  enum class Outcome { Succeeded, Canceled, Failed };
  using StopFn = std::function<bool()>;

  struct GripperResult
  {
    bool ok{false};
    bool reached_goal{false};
    bool stalled{false};
  };

  class ServoStream
  {
  public:
    explicit ServoStream(GridCutterActionServer & n) : n_(n) { n_.streaming_ = true; }
    ~ServoStream()
    {
      n_.stop_servo();
      n_.streaming_ = false;
    }
    ServoStream(const ServoStream &) = delete;
    ServoStream & operator=(const ServoStream &) = delete;

  private:
    GridCutterActionServer & n_;
  };

  // ---- Members ----
  rclcpp::Node::SharedPtr moveit_node_;
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tool_tf_broadcaster_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_client_;
  rclcpp_action::Client<SendInteger>::SharedPtr pruner_client_;
  rclcpp_action::Server<CutStem>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr servo_timer_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;

  std::string base_frame_;
  std::string ee_link_;
  std::string tool_link_;
  Eigen::Isometry3d ee_T_tool_{Eigen::Isometry3d::Identity()};  // pose of tool_link in ee_link

  geometry_msgs::msg::TwistStamped current_twist_;
  std::mutex twist_mutex_;
  std::thread execution_thread_;

  std::atomic<bool> is_executing_{false};
  std::atomic<bool> shutting_down_{false};
  std::atomic<bool> streaming_{false};

  GridConfig cfg_{};
  bool arm_engaged_{false};
  // Joint configuration of the last approach pose that MoveIt reached successfully.
  std::vector<double> last_good_approach_joints_;

  // Reachable circle at z_cut in base_frame_ (radius <= 0 = not computed).
  double reach_cx_{0.0}, reach_cy_{0.0}, reach_radius_{0.0};
  std::vector<std::string> loaded_object_ids_;

  // ---- Dynamic Parameter Parsing & Collision Scene Management ----
  void load_and_add_collision_objects()
  {
    declare_if_not_declared("collision_objects", std::vector<std::string>{});
    const auto object_names = this->get_parameter("collision_objects").as_string_array();

    if (object_names.empty()) {
      return;
    }

    // Retrieve names of objects already existing in the planning scene
    const auto known_objects = planning_scene_interface_->getKnownObjectNames();
    std::vector<moveit_msgs::msg::CollisionObject> new_objects;

    for (const auto & name : object_names) {
      // Check if object already exists in MoveIt
      if (std::find(known_objects.begin(), known_objects.end(), name) != known_objects.end()) {
        RCLCPP_INFO(this->get_logger(), "Collision object '%s' already exists in scene. Skipping.", name.c_str());
        continue;
      }

      // Declare object-specific parameters
      declare_if_not_declared(name + ".type", std::string("box"));
      declare_if_not_declared(name + ".dimensions", std::vector<double>{0.1, 0.1, 0.1});
      declare_if_not_declared(name + ".pose", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0});
      declare_if_not_declared(name + ".frame_id", base_frame_);

      std::string type = get_string(name + ".type");
      auto dims = get_double_array(name + ".dimensions");
      auto pose_vec = get_double_array(name + ".pose");
      std::string frame_id = get_string(name + ".frame_id");

      if (pose_vec.size() != 7) {
        RCLCPP_ERROR(this->get_logger(), "Object '%s' pose requires 7 elements [x,y,z, qx,qy,qz,qw]. Skipping.", name.c_str());
        continue;
      }

      moveit_msgs::msg::CollisionObject obj;
      obj.header.frame_id = frame_id;
      obj.id = name;

      shape_msgs::msg::SolidPrimitive primitive;

      if (type == "box") {
        if (dims.size() < 3) {
          RCLCPP_ERROR(this->get_logger(), "Box '%s' requires 3 dimensions [x, y, z]. Skipping.", name.c_str());
          continue;
        }
        primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
        primitive.dimensions = {dims[0], dims[1], dims[2]};
      } else if (type == "cylinder") {
        if (dims.size() < 2) {
          RCLCPP_ERROR(this->get_logger(), "Cylinder '%s' requires 2 dimensions [height, radius]. Skipping.", name.c_str());
          continue;
        }
        primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
        primitive.dimensions.resize(2);
        primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = dims[0];
        primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = dims[1];
      } else {
        RCLCPP_WARN(this->get_logger(), "Unsupported shape type '%s' for object '%s'. Skipping.", type.c_str(), name.c_str());
        continue;
      }

      geometry_msgs::msg::Pose pose;
      pose.position.x = pose_vec[0];
      pose.position.y = pose_vec[1];
      pose.position.z = pose_vec[2];
      pose.orientation.x = pose_vec[3];
      pose.orientation.y = pose_vec[4];
      pose.orientation.z = pose_vec[5];
      pose.orientation.w = pose_vec[6];

      obj.primitives.push_back(primitive);
      obj.primitive_poses.push_back(pose);
      obj.operation = obj.ADD;

      new_objects.push_back(obj);
      loaded_object_ids_.push_back(name);
    }

    if (!new_objects.empty()) {
      planning_scene_interface_->applyCollisionObjects(new_objects);
      RCLCPP_INFO(this->get_logger(), "Added %zu new collision objects to MoveIt planning scene.", new_objects.size());
    }
  }

  void remove_loaded_collision_objects()
  {
    if (loaded_object_ids_.empty()) {
      return;
    }
    planning_scene_interface_->removeCollisionObjects(loaded_object_ids_);
    RCLCPP_INFO(this->get_logger(), "Removed %zu collision objects from MoveIt planning scene.", loaded_object_ids_.size());
    loaded_object_ids_.clear();
  }

  // ---- Parameters ----
  template <typename T>
  void declare_if_not_declared(const std::string & name, const T & default_value)
  {
    if (!this->has_parameter(name)) {
      this->declare_parameter(name, default_value);
    }
  }

  void declare_all_parameters()
  {
    declare_if_not_declared("zone_x_min", 0.2);
    declare_if_not_declared("zone_x_max", 0.3);
    declare_if_not_declared("zone_y_min", 0.2);
    declare_if_not_declared("zone_y_max", 0.3);
    declare_if_not_declared("zone_z_height", 0.03);
    declare_if_not_declared("safe_z_height", 0.15);
    declare_if_not_declared("grid_dx", 0.06);
    declare_if_not_declared("grid_dy", 0.05);
    declare_if_not_declared("approach_offset", 0.10);
    declare_if_not_declared("pushing_dist", 0.01);

    declare_if_not_declared("approach_from_patch_row", false);
    declare_if_not_declared("grasp_orientation", std::vector<double>{-0.5, 0.5, 0.5, 0.5});
    declare_if_not_declared(
      "preferred_joint_positions",
      std::vector<double>{-0.491424061204615, -1.7174645975088305, 1.3383140815397754,
        -0.5025198555703216, -1.5730738621591325, -1.4791010273023781});

    declare_if_not_declared("limit_to_reach_circle", true);
    declare_if_not_declared("reach_max_radius", 1.0);
    declare_if_not_declared("reach_radial_step", 0.01);
    declare_if_not_declared("reach_ik_timeout", 0.05);
    declare_if_not_declared("reach_angle_samples", 19);
    declare_if_not_declared("reach_margin", 0.0);

    declare_if_not_declared("servo_kp", 1.5);
    declare_if_not_declared("servo_max_vel", 0.15);
    declare_if_not_declared("pose_tolerance", 0.005);
    declare_if_not_declared("servo_timeout", 15.0);

    declare_if_not_declared("gripper_open", 0.0);
    declare_if_not_declared("gripper_close", 0.9);
    declare_if_not_declared("gripper_max_effort", 0.0);
    declare_if_not_declared("gripper_timeout", 10.0);
    declare_if_not_declared("verify_grasp", false);
    declare_if_not_declared("pruner_command", 42);
    declare_if_not_declared("pruner_timeout", 30.0);
    

    

    declare_if_not_declared("max_patch_attempts", 3);
    declare_if_not_declared("max_consecutive_failures", 3);
    declare_if_not_declared("moveit_attempts", 3);
    declare_if_not_declared("drop_pose", std::string("drop"));
    // Optional: a joint configuration (move group joint order) to use INSTEAD of the named drop_pose, for robots whose
    // SRDF (generated from their robot.yaml poses) has no "drop" state. Empty = use the named pose.
    declare_if_not_declared("drop_joint_positions", std::vector<double>{});
    // The tool is lowered this far (servo, straight down) after reaching the drop pose, then released and raised again.
    // 0 = release right at the drop pose (for robots without an unloader box to reach into).
    declare_if_not_declared("drop_lower_distance", 0.1);
    declare_if_not_declared("stow_pose", std::string("stow"));

    declare_if_not_declared("move_group", std::string("arm_0"));
    declare_if_not_declared("moveit_vel_scale", 0.5);
    declare_if_not_declared("moveit_acc_scale", 0.5);
    declare_if_not_declared("moveit_planning_time", 5.0);
    declare_if_not_declared("moveit_planning_attempts", 20);
    declare_if_not_declared("goal_position_tolerance", 0.005);
    declare_if_not_declared("goal_orientation_tolerance", 0.05);

    declare_if_not_declared("base_frame", std::string("arm_0_base_link"));
    declare_if_not_declared("ee_link", std::string("arm_0_end_effector_link"));
    // Tool frame: where the cutting/grasping point is, as a static transform from ee_link. Every patch, approach and
    // drop position (and grasp_orientation) refers to this frame; the end-effector target is derived from it.
    // Identity (the default) = tool_link coincides with ee_link, i.e. the old behaviour.
    declare_if_not_declared("tool_link", std::string("tool_link"));
    declare_if_not_declared("tool_xyz", std::vector<double>{0.0, 0.0, 0.0});
    declare_if_not_declared("tool_rpy", std::vector<double>{0.0, 0.0, 0.0});
    declare_if_not_declared("publish_tool_tf", true);
    // Relative names: they resolve under this node's namespace (the robot's), so the same code works for
    // /j100_0921, /a300_00036, ... (these used to be hardcoded to /j100_0921/..., which left the gripper
    // action "not available" on every other robot).
    declare_if_not_declared("servo_twist_topic", std::string("manipulator/delta_twist_cmds"));
    declare_if_not_declared("gripper_action",
      std::string("manipulators/arm_0_gripper_controller/gripper_cmd"));
    declare_if_not_declared("pruner_action", std::string("pruner_action_server"));
  }

  double get_double(const std::string & name) const
  {
    const auto p = this->get_parameter(name);
    if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      return static_cast<double>(p.as_int());
    }
    return p.as_double();
  }

  int get_int(const std::string & name) const
  {
    return static_cast<int>(this->get_parameter(name).as_int());
  }

  bool get_bool(const std::string & name) const { return this->get_parameter(name).as_bool(); }

  std::string get_string(const std::string & name) const
  {
    return this->get_parameter(name).as_string();
  }

  std::vector<double> get_double_array(const std::string & name) const
  {
    const auto p = this->get_parameter(name);
    if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
      const auto ints = p.as_integer_array();
      return std::vector<double>(ints.begin(), ints.end());
    }
    return p.as_double_array();
  }

  void load_tool_offset()
  {
    const auto xyz = get_double_array("tool_xyz");
    const auto rpy = get_double_array("tool_rpy");
    if (xyz.size() != 3 || rpy.size() != 3) {
      RCLCPP_ERROR(this->get_logger(), "tool_xyz and tool_rpy need 3 elements each; using identity.");
      return;
    }
    ee_T_tool_ = Eigen::Isometry3d::Identity();
    ee_T_tool_.translation() = Eigen::Vector3d(xyz[0], xyz[1], xyz[2]);
    ee_T_tool_.linear() =
      (Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX())).toRotationMatrix();
  }

  // The end-effector pose (base frame) that puts tool_link at base_T_tool.
  Eigen::Isometry3d ee_pose_for_tool(const Eigen::Isometry3d & base_T_tool) const
  {
    return base_T_tool * ee_T_tool_.inverse();
  }

  static int grid_count(double lo, double hi, double step)
  {
    return static_cast<int>(std::floor((hi - lo) / step + 1e-6)) + 1;
  }

  bool load_config(GridConfig & c, std::string & err) const
  {
    c.x_min = get_double("zone_x_min");
    c.x_max = get_double("zone_x_max");
    c.y_min = get_double("zone_y_min");
    c.y_max = get_double("zone_y_max");
    c.z_cut = get_double("zone_z_height");
    c.safe_z = get_double("safe_z_height");
    c.dx = get_double("grid_dx");
    c.dy = get_double("grid_dy");
    c.approach_offset = get_double("approach_offset");
    c.pushing_dist=get_double("pushing_dist");
    c.approach_from_patch_row = get_bool("approach_from_patch_row");

    const auto q = get_double_array("grasp_orientation");
    if (q.size() != 4) {
      err = "grasp_orientation must have 4 elements (x, y, z, w)";
      return false;
    }
    const double qn = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (qn < 1e-6) {
      err = "grasp_orientation has zero norm";
      return false;
    }
    for (size_t i = 0; i < 4; ++i) {c.grasp_q[i] = q[i] / qn;}

    c.preferred_joints = get_double_array("preferred_joint_positions");
    const size_t n_joints = move_group_->getVariableCount();
    if (!c.preferred_joints.empty() && c.preferred_joints.size() != n_joints) {
      err = "preferred_joint_positions must have " + std::to_string(n_joints) +
        " elements (or be empty)";
      return false;
    }

    c.limit_to_reach_circle = get_bool("limit_to_reach_circle");
    c.reach_max_radius = get_double("reach_max_radius");
    c.reach_radial_step = get_double("reach_radial_step");
    c.reach_ik_timeout = get_double("reach_ik_timeout");
    c.reach_angle_samples = get_int("reach_angle_samples");
    c.reach_margin = get_double("reach_margin");

    c.servo_kp = get_double("servo_kp");
    c.servo_max_vel = get_double("servo_max_vel");
    c.pose_tol = get_double("pose_tolerance");
    c.servo_timeout = get_double("servo_timeout");

    c.gripper_open = get_double("gripper_open");
    c.gripper_close = get_double("gripper_close");
    c.gripper_max_effort = get_double("gripper_max_effort");
    c.gripper_timeout = get_double("gripper_timeout");
    c.verify_grasp = get_bool("verify_grasp");
    c.pruner_command = get_int("pruner_command");
    c.pruner_timeout = get_double("pruner_timeout");

    c.max_patch_attempts = get_int("max_patch_attempts");
    c.max_consecutive_failures = get_int("max_consecutive_failures");
    c.moveit_attempts = get_int("moveit_attempts");
    c.drop_pose = get_string("drop_pose");
    c.drop_joints = get_double_array("drop_joint_positions");
    c.drop_lower_distance = get_double("drop_lower_distance");
    if (!c.drop_joints.empty() && c.drop_joints.size() != n_joints) {
      err = "drop_joint_positions must have " + std::to_string(n_joints) + " elements (or be empty)";
      return false;
    }
    c.stow_pose = get_string("stow_pose");

    if (c.dx <= 0.0 || c.dy <= 0.0) { err = "grid_dx and grid_dy must be > 0"; return false; }
    if (c.x_max < c.x_min || c.y_max < c.y_min) { err = "zone max must be >= zone min"; return false; }
    if (c.approach_offset < 0.0) { err = "approach_offset must be >= 0"; return false; }
    if (c.reach_max_radius <= 0.0 || c.reach_radial_step <= 0.0 || c.reach_ik_timeout <= 0.0 ||
      c.reach_angle_samples < 1 || c.reach_margin < 0.0)
    {
      err = "reach_max_radius, reach_radial_step, reach_ik_timeout must be > 0, "
        "reach_angle_samples >= 1 and reach_margin >= 0";
      return false;
    }
    if (c.servo_kp <= 0.0 || c.servo_max_vel <= 0.0 || c.pose_tol <= 0.0) {
      err = "servo_kp, servo_max_vel and pose_tolerance must be > 0";
      return false;
    }
    if (c.servo_timeout <= 0.0 || c.gripper_timeout <= 0.0 || c.pruner_timeout <= 0.0) {
      err = "timeouts must be > 0";
      return false;
    }
    if (c.max_patch_attempts < 1 || c.max_consecutive_failures < 1 || c.moveit_attempts < 1) {
      err = "attempt limits must be >= 1";
      return false;
    }
    const double n_patches = static_cast<double>(grid_count(c.x_min, c.x_max, c.dx)) *
                             static_cast<double>(grid_count(c.y_min, c.y_max, c.dy));
    if (n_patches > 5000.0) { err = "grid would contain more than 5000 patches"; return false; }
    return true;
  }

  StopFn make_stop_fn(const std::shared_ptr<GoalHandleCutStem> & gh) const
  {
    return [this, gh]() {
             return gh->is_canceling() || shutting_down_.load() || !rclcpp::ok();
           };
  }

  // Timeouts and pauses run on the node's clock: ROS time, i.e. the simulator's /clock with use_sim_time, else wall
  // time. In a sim slower than real time they then last as long as the robot needs to move, not as long in wall time.
  rclcpp::Time deadline_in(double seconds)
  {
    return this->now() + rclcpp::Duration::from_seconds(seconds);
  }

  void ros_sleep(double seconds)
  {
    this->get_clock()->sleep_for(rclcpp::Duration::from_seconds(seconds));
  }

  void interruptible_sleep(double seconds, const StopFn & stop)
  {
    const auto end = deadline_in(seconds);
    while (this->now() < end && !stop() && rclcpp::ok()) {
      ros_sleep(0.02);
    }
  }

  void publish_status(const std::shared_ptr<GoalHandleCutStem> & gh, const std::string & text)
  {
    auto fb = std::make_shared<CutStem::Feedback>();
    fb->current_state = text;
    gh->publish_feedback(fb);
  }

  void servo_timer_callback()
  {
    if (!streaming_) {return;}
    std::lock_guard<std::mutex> lock(twist_mutex_);
    current_twist_.header.stamp = this->get_clock()->now();
    current_twist_.header.frame_id = base_frame_;
    twist_pub_->publish(current_twist_);
  }

  void set_twist(double vx, double vy, double vz)
  {
    std::lock_guard<std::mutex> lock(twist_mutex_);
    current_twist_.twist = geometry_msgs::msg::Twist();
    current_twist_.twist.linear.x = vx;
    current_twist_.twist.linear.y = vy;
    current_twist_.twist.linear.z = vz;
  }

  void stop_servo()
  {
    try {
      std::lock_guard<std::mutex> lock(twist_mutex_);
      current_twist_.twist = geometry_msgs::msg::Twist();
      current_twist_.header.stamp = this->get_clock()->now();
      current_twist_.header.frame_id = base_frame_;
      twist_pub_->publish(current_twist_);
    } catch (const std::exception &) {
    }
  }

  // Position of the tool frame (tool_link) in base_frame_.
  bool get_ee_position(double & x, double & y, double & z)
  {
    try {
      auto t = tf_buffer_->lookupTransform(base_frame_, tool_link_, tf2::TimePointZero);
      x = t.transform.translation.x;
      y = t.transform.translation.y;
      z = t.transform.translation.z;
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "TF exception in get_ee_position: %s", ex.what());
      return false;
    }
  }

  bool servo_to_pose(double tx, double ty, double tz, const StopFn & stop)
  {
    const auto deadline = deadline_in(cfg_.servo_timeout);
    ServoStream stream(*this);
    double dist = -1.0;

    while (rclcpp::ok()) {
      if (stop()) {return false;}
      if (this->now() > deadline) {
        RCLCPP_ERROR(
          this->get_logger(),
          "servo_to_pose timed out after %.1fs (target %.3f %.3f %.3f, last distance %.4f m).",
          cfg_.servo_timeout, tx, ty, tz, dist);
        return false;
      }

      double cx, cy, cz;
      if (!get_ee_position(cx, cy, cz)) {
        set_twist(0.0, 0.0, 0.0);
        ros_sleep(0.02);
        continue;
      }

      const double ex = tx - cx, ey = ty - cy, ez = tz - cz;
      dist = std::sqrt(ex * ex + ey * ey + ez * ez);
      if (dist <= cfg_.pose_tol) {return true;}

      double vx = cfg_.servo_kp * ex;
      double vy = cfg_.servo_kp * ey;
      double vz = cfg_.servo_kp * ez;
      const double v_mag = std::sqrt(vx * vx + vy * vy + vz * vz);
      if (v_mag > cfg_.servo_max_vel) {
        const double s = cfg_.servo_max_vel / v_mag;
        vx *= s; vy *= s; vz *= s;
      }
      set_twist(vx, vy, vz);
      ros_sleep(0.02);
    }
    return false;
  }

  bool plan_and_execute(const std::string & what, const StopFn & stop)
  {
    for (int attempt = 1; attempt <= cfg_.moveit_attempts; ++attempt) {
      if (stop()) {return false;}
      RCLCPP_INFO(
        this->get_logger(), "Planning '%s' (attempt %d/%d)...",
        what.c_str(), attempt, cfg_.moveit_attempts);

      move_group_->setStartStateToCurrentState();
      moveit::planning_interface::MoveGroupInterface::Plan plan;

      if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        const auto code = move_group_->execute(plan);
        if (code == moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_INFO(this->get_logger(), "Reached '%s'.", what.c_str());
          return true;
        }
        RCLCPP_ERROR(this->get_logger(), "Execution of '%s' failed with code %d.", what.c_str(), code.val);
      } else {
        RCLCPP_WARN(
          this->get_logger(), "Planning '%s' failed (attempt %d/%d).",
          what.c_str(), attempt, cfg_.moveit_attempts);
      }

      if (attempt < cfg_.moveit_attempts) {
        interruptible_sleep(0.5, stop);
      }
    }
    RCLCPP_ERROR(
      this->get_logger(), "All %d attempts failed for '%s'.", cfg_.moveit_attempts, what.c_str());
    return false;
  }

  // The drop configuration: a joint-space target if drop_joint_positions is set, else the named SRDF pose.
  bool trigger_drop_pose(const StopFn & stop)
  {
    if (cfg_.drop_joints.empty()) {return trigger_named_pose(cfg_.drop_pose, stop);}
    stop_servo();
    ros_sleep(0.1);
    move_group_->clearPoseTargets();
    if (!move_group_->setJointValueTarget(cfg_.drop_joints)) {
      RCLCPP_ERROR(this->get_logger(), "drop_joint_positions is outside the joint limits.");
      return false;
    }
    return plan_and_execute("drop configuration", stop);
  }

  bool trigger_named_pose(const std::string & pose_name, const StopFn & stop)
  {
    stop_servo();
    ros_sleep(0.1);

    move_group_->clearPoseTargets();
    if (!move_group_->setNamedTarget(pose_name)) {
      RCLCPP_ERROR(
        this->get_logger(), "Named target '%s' is not defined for this group (check the SRDF).",
        pose_name.c_str());
      return false;
    }
    return plan_and_execute("named pose " + pose_name, stop);
  }

  bool move_to_joint_positions(
    const std::vector<double> & joints, const std::string & what, const StopFn & stop)
  {
    stop_servo();
    ros_sleep(0.1);

    move_group_->clearPoseTargets();
    if (!move_group_->setJointValueTarget(joints)) {
      RCLCPP_ERROR(this->get_logger(), "Invalid joint target for '%s'.", what.c_str());
      return false;
    }
    return plan_and_execute(what, stop);
  }

  // Move to the patch's approach pose. If that fails, go back to the last approach
  // configuration that was reached successfully and retry from there once.
  bool move_to_approach(const Patch & p, const StopFn & stop)
  {
    bool ok = move_to_cartesian_pose(p.x, approach_y(p), cfg_.z_cut, stop);
    if (!ok && !stop() && !last_good_approach_joints_.empty()) {
      RCLCPP_WARN(
        this->get_logger(),
        "Approach for patch %d failed; moving to the last successful approach configuration.", p.id);
      if (!move_to_joint_positions(last_good_approach_joints_, "last good approach", stop)) {
        return false;
      }
      ok = move_to_cartesian_pose(p.x, approach_y(p), cfg_.z_cut, stop);
    }
    if (ok) {
      last_good_approach_joints_ = move_group_->getCurrentJointValues();
    }
    return ok;
  }

  bool move_to_cartesian_pose(double x, double y, double z, const StopFn & stop)
  {
    stop_servo();
    ros_sleep(0.1);

    // (x, y, z) and grasp_orientation describe the TOOL frame; the planner is given the matching end-effector pose.
    Eigen::Isometry3d base_T_tool = Eigen::Isometry3d::Identity();
    base_T_tool.translation() = Eigen::Vector3d(x, y, z);
    base_T_tool.linear() =
      Eigen::Quaterniond(cfg_.grasp_q[3], cfg_.grasp_q[0], cfg_.grasp_q[1], cfg_.grasp_q[2]).toRotationMatrix();
    const Eigen::Isometry3d base_T_ee = ee_pose_for_tool(base_T_tool);
    const Eigen::Quaterniond qe(base_T_ee.rotation());

    geometry_msgs::msg::PoseStamped target;
    target.header.frame_id = base_frame_;
    target.header.stamp = this->get_clock()->now();
    target.pose.position.x = base_T_ee.translation().x();
    target.pose.position.y = base_T_ee.translation().y();
    target.pose.position.z = base_T_ee.translation().z();
    target.pose.orientation.x = qe.x();
    target.pose.orientation.y = qe.y();
    target.pose.orientation.z = qe.z();
    target.pose.orientation.w = qe.w();

    move_group_->clearPoseTargets();
    // move_group_->setPathConstraints(constraints);
    if (!set_ik_target_near_preferred(target)) {
      move_group_->setPoseTarget(target);
    }

    std::ostringstream what;
    what << std::fixed << std::setprecision(3) << "pose (" << x << ", " << y << ", " << z << "," << cfg_.grasp_q[0] << ", " << cfg_.grasp_q[1] << ", " << cfg_.grasp_q[2] << "," << cfg_.grasp_q[3] << ")";
    const bool ok = plan_and_execute(what.str(), stop);

    // Clean up constraints and targets after planning
    move_group_->clearPathConstraints();
    move_group_->clearPoseTargets();
    return ok;
  }
  // Solve IK for the target starting from cfg_.preferred_joints, so the solver returns
  // the solution closest to that configuration, and set it as a joint target.
  // Returns false (leaving the target unset) if no preferred config is set or IK fails.
  bool set_ik_target_near_preferred(const geometry_msgs::msg::PoseStamped & target)
  {
    if (cfg_.preferred_joints.empty()) {return false;}

    // setJointValueTarget(pose) seeds IK from the start state, so temporarily set the
    // start state to the preferred configuration. plan_and_execute resets it to current.
    auto seed = move_group_->getCurrentState(2.0);
    if (!seed) {
      RCLCPP_WARN(this->get_logger(), "No current robot state; falling back to pose target.");
      return false;
    }
    seed->setJointGroupPositions(move_group_->getName(), cfg_.preferred_joints);
    seed->update();
    move_group_->setStartState(*seed);

    const bool ok = move_group_->setJointValueTarget(target, ee_link_);
    move_group_->setStartStateToCurrentState();
    if (!ok) {
      RCLCPP_WARN(
        this->get_logger(), "IK near preferred configuration failed; falling back to pose target.");
      return false;
    }

    std::vector<double> solution;
    move_group_->getJointValueTarget(solution);
    double max_dev = 0.0;
    for (size_t i = 0; i < solution.size(); ++i) {
      max_dev = std::max(max_dev, std::abs(solution[i] - cfg_.preferred_joints[i]));
    }
    RCLCPP_INFO(
      this->get_logger(), "IK solution max deviation from preferred configuration: %.3f rad.", max_dev);
    return true;
  }

  template <typename ActionT>
  std::optional<typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult>
  send_goal_and_wait(
    const typename rclcpp_action::Client<ActionT>::SharedPtr & client,
    const typename ActionT::Goal & goal,
    const std::string & name, double timeout_s, const StopFn & stop)
  {
    using Client = rclcpp_action::Client<ActionT>;

    if (stop()) {return std::nullopt;}
    if (!client->wait_for_action_server(2s)) {
      RCLCPP_ERROR(this->get_logger(), "%s action server not available.", name.c_str());
      return std::nullopt;
    }

    const auto deadline = deadline_in(timeout_s);

    auto goal_future = client->async_send_goal(goal, typename Client::SendGoalOptions());
    if (goal_future.wait_for(5s) != std::future_status::ready) {
      RCLCPP_ERROR(this->get_logger(), "%s: timed out waiting for goal acceptance.", name.c_str());
      return std::nullopt;
    }
    auto handle = goal_future.get();
    if (!handle) {
      RCLCPP_ERROR(this->get_logger(), "%s: goal was rejected.", name.c_str());
      return std::nullopt;
    }

    auto result_future = client->async_get_result(handle);
    while (result_future.wait_for(50ms) != std::future_status::ready) {
      if (stop()) {
        client->async_cancel_goal(handle);
        return std::nullopt;
      }
      if (this->now() > deadline) {
        RCLCPP_ERROR(this->get_logger(), "%s: no result after %.1fs; canceling goal.", name.c_str(), timeout_s);
        client->async_cancel_goal(handle);
        return std::nullopt;
      }
    }
    return result_future.get();
  }

  GripperResult send_gripper_command(double position, const StopFn & stop)
  {
    GripperResult out;
    GripperCommand::Goal goal;
    goal.command.position = position;
    goal.command.max_effort = cfg_.gripper_max_effort;

    auto res = send_goal_and_wait<GripperCommand>(
      gripper_client_, goal, "Gripper", cfg_.gripper_timeout, stop);
    if (!res) {return out;}

    if (res->result) {
      out.reached_goal = res->result->reached_goal;
      out.stalled = res->result->stalled;
    }
    out.ok = (res->code == rclcpp_action::ResultCode::SUCCEEDED);
    return out;
  }

  bool close_gripper_on_stem(const StopFn & stop)
  {
    const GripperResult g = send_gripper_command(cfg_.gripper_close, stop);

    if (g.stalled) {
      RCLCPP_INFO(this->get_logger(), "Gripper stalled: stem grasped.");
      return true;
    }
    if (!g.ok) {return false;}
    if (cfg_.verify_grasp && g.reached_goal) {
      RCLCPP_WARN(this->get_logger(), "Gripper closed fully without stalling: no stem grasped.");
      return false;
    }
    return true;
  }

  bool send_pruner_command(int target, const StopFn & stop)
  {
    SendInteger::Goal goal;
    goal.target_integer = target;

    auto res = send_goal_and_wait<SendInteger>(
      pruner_client_, goal, "Pruner", cfg_.pruner_timeout, stop);
    if (!res) {return false;}
    return res->code == rclcpp_action::ResultCode::SUCCEEDED && res->result && res->result->success;
  }

  // Find the circle (in base_frame_, at height z_cut) that the end effector can reach with
  // the grasp orientation. The center is the axis of the group's first joint; along each
  // direction that crosses the cutting zone the radius is increased until IK fails, then
  // refined by bisection. The circle radius is the smallest of these edge radii.
  bool compute_reach_circle(const StopFn & stop, std::string & err)
  {
    const auto model = move_group_->getRobotModel();
    const auto * jmg = model->getJointModelGroup(move_group_->getName());
    if (!jmg || jmg->getActiveJointModels().empty()) {
      err = "move group '" + move_group_->getName() + "' has no active joints";
      return false;
    }

    moveit::core::RobotState state(model);
    state.setToDefaultValues();
    if (!cfg_.preferred_joints.empty()) {
      state.setJointGroupPositions(jmg, cfg_.preferred_joints);
    }
    state.update();
    if (!state.knowsFrameTransform(base_frame_)) {
      err = "robot model does not know frame '" + base_frame_ + "'";
      return false;
    }
    const Eigen::Isometry3d model_T_base = state.getFrameTransform(base_frame_);
    const std::string & axis_link =
      jmg->getActiveJointModels().front()->getChildLinkModel()->getName();
    const Eigen::Vector3d center =
      (model_T_base.inverse() * state.getGlobalLinkTransform(axis_link)).translation();
    std::vector<double> seed;
    state.copyJointGroupPositions(jmg, seed);

    const Eigen::Quaterniond q(cfg_.grasp_q[3], cfg_.grasp_q[0], cfg_.grasp_q[1], cfg_.grasp_q[2]);
    auto reachable = [&](double r, double th) {
        Eigen::Isometry3d base_T_tool = Eigen::Isometry3d::Identity();
        base_T_tool.translation() =
          Eigen::Vector3d(center.x() + r * std::cos(th), center.y() + r * std::sin(th), cfg_.z_cut);
        base_T_tool.linear() = q.toRotationMatrix();
        const Eigen::Isometry3d base_T_ee = ee_pose_for_tool(base_T_tool);
        state.setJointGroupPositions(jmg, seed);
        return state.setFromIK(jmg, model_T_base * base_T_ee, ee_link_, cfg_.reach_ik_timeout);
      };

    // Angular sector (around the center) spanned by the cutting zone.
    const double ref = std::atan2(
      0.5 * (cfg_.y_min + cfg_.y_max) - center.y(), 0.5 * (cfg_.x_min + cfg_.x_max) - center.x());
    double th_lo = 0.0, th_hi = 0.0;
    const bool center_in_zone =
      center.x() >= cfg_.x_min && center.x() <= cfg_.x_max &&
      center.y() >= cfg_.y_min && center.y() <= cfg_.y_max;
    if (center_in_zone) {
      th_lo = -M_PI; th_hi = M_PI;
    } else {
      for (double cx : {cfg_.x_min, cfg_.x_max}) {
        for (double cy : {cfg_.y_min, cfg_.y_max}) {
          const double d = std::remainder(std::atan2(cy - center.y(), cx - center.x()) - ref, 2.0 * M_PI);
          th_lo = std::min(th_lo, d);
          th_hi = std::max(th_hi, d);
        }
      }
    }

    const int n = (th_hi > th_lo) ? std::max(cfg_.reach_angle_samples, 2) : 1;
    double radius = cfg_.reach_max_radius;
    for (int k = 0; k < n; ++k) {
      if (stop()) {err = "canceled"; return false;}
      const double th = ref + th_lo + (n > 1 ? (th_hi - th_lo) * k / (n - 1) : 0.0);

      double last_ok = -1.0, first_fail = -1.0;
      for (double r = cfg_.reach_radial_step; r <= cfg_.reach_max_radius + 1e-9; r += cfg_.reach_radial_step) {
        if (reachable(r, th)) {
          last_ok = r;
        } else if (last_ok > 0.0) {
          first_fail = r;
          break;
        }
      }
      if (last_ok < 0.0) {
        err = "no reachable point at z=" + std::to_string(cfg_.z_cut) + " along direction " +
          std::to_string(th) + " rad";
        return false;
      }
      if (first_fail > 0.0) {
        while (first_fail - last_ok > 0.002) {
          const double mid = 0.5 * (last_ok + first_fail);
          (reachable(mid, th) ? last_ok : first_fail) = mid;
        }
      }
      RCLCPP_DEBUG(this->get_logger(), "Reach at angle %.3f rad: %.3f m", th, last_ok);
      radius = std::min(radius, last_ok);
    }

    reach_cx_ = center.x();
    reach_cy_ = center.y();
    reach_radius_ = radius - cfg_.reach_margin;
    RCLCPP_INFO(
      this->get_logger(),
      "Reachable circle at z=%.3f in '%s': center (%.3f, %.3f), radius %.3f m (%d directions).",
      cfg_.z_cut, base_frame_.c_str(), reach_cx_, reach_cy_, reach_radius_, n);
    if (reach_radius_ <= 0.0) {
      err = "reachable radius is not positive after reach_margin";
      return false;
    }
    return true;
  }

  bool inside_reach_circle(double x, double y) const
  {
    return !cfg_.limit_to_reach_circle ||
           std::hypot(x - reach_cx_, y - reach_cy_) <= reach_radius_;
  }

  std::vector<Patch> generate_patches() const
  {
    const int nx = grid_count(cfg_.x_min, cfg_.x_max, cfg_.dx);
    const int ny = grid_count(cfg_.y_min, cfg_.y_max, cfg_.dy);
    std::vector<Patch> patches;
    patches.reserve(static_cast<size_t>(nx) * static_cast<size_t>(ny));
    int id = 0;
    for (int j = 0; j < ny; ++j) {
      for (int i = 0; i < nx; ++i) {
        const double x = cfg_.x_min + i * cfg_.dx, y = cfg_.y_min + j * cfg_.dy;
        if (!inside_reach_circle(x, y)) {continue;}
        patches.push_back({id++, x, y, PatchStatus::Pending});
      }
    }
    return patches;
  }

  double approach_y(const Patch & p) const
  {
    return (cfg_.approach_from_patch_row ? p.y : cfg_.y_min) - cfg_.approach_offset;
  }

  void publish_grid_markers(const std::vector<Patch> & patches)
  {
    visualization_msgs::msg::MarkerArray msg;
    const auto stamp = this->get_clock()->now();

    for (const auto & p : patches) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = base_frame_;
      m.header.stamp = stamp;
      m.ns = "cutting_patches";
      m.id = p.id;
      m.type = visualization_msgs::msg::Marker::CUBE;

      if (p.status == PatchStatus::Cut) {
        m.action = visualization_msgs::msg::Marker::DELETE;
      } else {
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = p.x;
        m.pose.position.y = p.y;
        m.pose.position.z = cfg_.z_cut;
        m.pose.orientation.w = 1.0;
        m.scale.x = cfg_.dx;
        m.scale.y = cfg_.dy;
        m.scale.z = 0.005;
        switch (p.status) {
          case PatchStatus::Active:
            m.color.r = 1.0f; m.color.g = 0.0f; m.color.b = 0.0f; m.color.a = 0.8f;
            break;
          case PatchStatus::Failed:
            m.color.r = 1.0f; m.color.g = 0.5f; m.color.b = 0.0f; m.color.a = 0.7f;
            break;
          default:
            m.color.r = 0.0f; m.color.g = 1.0f; m.color.b = 0.0f; m.color.a = 0.4f;
            break;
        }
      }
      msg.markers.push_back(m);
    }

    if (cfg_.limit_to_reach_circle && reach_radius_ > 0.0) {
      visualization_msgs::msg::Marker c;
      c.header.frame_id = base_frame_;
      c.header.stamp = stamp;
      c.ns = "reach_circle";
      c.id = 0;
      c.type = visualization_msgs::msg::Marker::LINE_STRIP;
      c.action = visualization_msgs::msg::Marker::ADD;
      c.pose.orientation.w = 1.0;
      c.scale.x = 0.004;
      c.color.r = 0.1f; c.color.g = 0.4f; c.color.b = 1.0f; c.color.a = 0.9f;
      constexpr int kSegments = 120;
      for (int k = 0; k <= kSegments; ++k) {
        const double th = 2.0 * M_PI * k / kSegments;
        geometry_msgs::msg::Point pt;
        pt.x = reach_cx_ + reach_radius_ * std::cos(th);
        pt.y = reach_cy_ + reach_radius_ * std::sin(th);
        pt.z = cfg_.z_cut;
        c.points.push_back(pt);
      }
      msg.markers.push_back(c);
    }
    marker_pub_->publish(msg);
  }

  void clear_markers()
  {
    try {
      visualization_msgs::msg::MarkerArray msg;
      visualization_msgs::msg::Marker m;
      m.header.frame_id = base_frame_;
      m.action = visualization_msgs::msg::Marker::DELETEALL;
      msg.markers.push_back(m);
      marker_pub_->publish(msg);
    } catch (const std::exception &) {
    }
  }

  bool try_prune_once(const Patch & p, const StopFn & stop, bool & in_field)
  {
    in_field = false;

    if (!move_to_approach(p, stop)) {return false;}

    // open the gripper before approaching
    if (!send_gripper_command(cfg_.gripper_open, stop).ok) {return false;}
    in_field = true;

    if (!servo_to_pose(p.x, p.y - cfg_.pushing_dist, cfg_.z_cut, stop)) {return false;}

    if (!close_gripper_on_stem(stop)) {return false;}

    if (!servo_to_pose(p.x, p.y, cfg_.z_cut, stop)) {return false;}    

    return send_pruner_command(cfg_.pruner_command, stop);
  }

  void recover_from_failure(const Patch & p, bool in_field, const StopFn & stop)
  {
    send_gripper_command(cfg_.gripper_open, stop);
    if (in_field && !stop()) {
      if (!servo_to_pose(p.x, approach_y(p), cfg_.z_cut, stop) && !stop()) {
        RCLCPP_WARN(this->get_logger(), "Could not back out to the approach point after failure.");
      }
    }
  }

  bool prune_patch(
    const Patch & p, const std::shared_ptr<GoalHandleCutStem> & gh, const StopFn & stop,
    size_t index, size_t total)
  {
    for (int attempt = 1; attempt <= cfg_.max_patch_attempts; ++attempt) {
      if (stop()) {return false;}

      std::ostringstream s;
      s << std::fixed << std::setprecision(3) << "Patch " << (index + 1) << "/" << total
        << " at X:" << p.x << " Y:" << p.y
        << " (attempt " << attempt << "/" << cfg_.max_patch_attempts << ")";
      publish_status(gh, s.str());

      bool in_field = false;
      if (try_prune_once(p, stop, in_field)) {return true;}
      if (stop()) {return false;}

      RCLCPP_WARN(
        this->get_logger(), "Patch %d failed (attempt %d/%d).",
        p.id, attempt, cfg_.max_patch_attempts);
      recover_from_failure(p, in_field, stop);
    }
    RCLCPP_ERROR(this->get_logger(), "Max retries reached for patch %d. Skipping.", p.id);
    return false;
  }

  bool retract_and_drop(const Patch & p, const StopFn & stop)
  {
    // if (!servo_to_pose(p.x, approach_y(p), cfg_.z_cut, stop)) {return false;}
    if (!servo_to_pose(p.x, p.y, cfg_.z_cut + 0.07, stop)) {return false;}
    if (!trigger_drop_pose(stop)) {return false;}

    // lower the end effector 5 cm along z before releasing
    ros_sleep(0.2);  // let TF catch up with the final drop pose
    double x, y, z;
    if (!get_ee_position(x, y, z)) {return false;}

    // go further into the unloader box
    if (cfg_.drop_lower_distance > 1e-3 && !servo_to_pose(x, y, z - cfg_.drop_lower_distance, stop)) {return false;}

    if (!send_gripper_command(cfg_.gripper_open, stop).ok) {return false;}

    // rise back up to the drop position
    return cfg_.drop_lower_distance <= 1e-3 || servo_to_pose(x, y, z, stop);
  }

  void safe_state()
  {
    if (!arm_engaged_) {return;}
    const StopFn stop = [this]() {return shutting_down_.load() || !rclcpp::ok();};

    RCLCPP_WARN(this->get_logger(), "Moving arm to a safe state (open gripper, lift).");
    stop_servo();
    send_gripper_command(cfg_.gripper_open, stop);

    double x, y, z;
    if (get_ee_position(x, y, z) && z < cfg_.safe_z - cfg_.pose_tol) {
      servo_to_pose(x, y, cfg_.safe_z, stop);
    }
  }

  Outcome canceled(CutStem::Result & r, size_t n_cut, size_t total)
  {
    r.success = false;
    r.message = "Goal canceled (or node shutting down) after cutting " + std::to_string(n_cut) +
      " of " + std::to_string(total) + " patches.";
    return Outcome::Canceled;
  }

  Outcome run_grid(const std::shared_ptr<GoalHandleCutStem> & gh, CutStem::Result & result)
  {
    const StopFn stop = make_stop_fn(gh);

    // Ensure collision objects defined in params are present prior to grid execution
    load_and_add_collision_objects();

    GridConfig cfg{};
    std::string err;
    if (!load_config(cfg, err)) {
      RCLCPP_ERROR(this->get_logger(), "Invalid configuration: %s", err.c_str());
      result.success = false;
      result.message = "Invalid configuration: " + err;
      return Outcome::Failed;
    }
    cfg_ = cfg;

    reach_radius_ = 0.0;
    if (cfg_.limit_to_reach_circle && !compute_reach_circle(stop, err)) {
      if (stop()) {return canceled(result, 0, 0);}
      RCLCPP_ERROR(this->get_logger(), "Reachable circle computation failed: %s", err.c_str());
      result.success = false;
      result.message = "Reachable circle computation failed: " + err;
      return Outcome::Failed;
    }

    std::vector<Patch> patches = generate_patches();
    const size_t total = patches.size();
    const size_t n_grid = static_cast<size_t>(grid_count(cfg_.x_min, cfg_.x_max, cfg_.dx)) *
      static_cast<size_t>(grid_count(cfg_.y_min, cfg_.y_max, cfg_.dy));
    if (total < n_grid) {
      RCLCPP_WARN(
        this->get_logger(), "Removed %zu of %zu patches outside the reachable circle.",
        n_grid - total, n_grid);
    }
    RCLCPP_INFO(this->get_logger(), "Executing grid sequence: %zu patches.", total);
    publish_grid_markers(patches);
    arm_engaged_ = true;
    last_good_approach_joints_.clear();

    size_t n_cut = 0, n_failed = 0;
    int consecutive_failures = 0;
    std::string failed_ids;

    for (size_t i = 0; i < patches.size(); ++i) {
      Patch & patch = patches[i];
      if (stop()) {return canceled(result, n_cut, total);}

      patch.status = PatchStatus::Active;
      publish_grid_markers(patches);

      const bool cut_ok = prune_patch(patch, gh, stop, i, total);
      if (stop()) {return canceled(result, n_cut, total);}

      if (cut_ok) {
        patch.status = PatchStatus::Cut;
        ++n_cut;
        consecutive_failures = 0;
        publish_grid_markers(patches);

        if (!retract_and_drop(patch, stop)) {
          if (stop()) {return canceled(result, n_cut, total);}
          result.success = false;
          result.message = "Patch " + std::to_string(patch.id) +
            " was cut but the retract/drop sequence failed; aborting.";
          RCLCPP_ERROR(this->get_logger(), "%s", result.message.c_str());
          return Outcome::Failed;
        }
      } else {
        patch.status = PatchStatus::Failed;
        ++n_failed;
        ++consecutive_failures;
        failed_ids += (failed_ids.empty() ? "" : ",") + std::to_string(patch.id);
        publish_grid_markers(patches);

        if (consecutive_failures >= cfg_.max_consecutive_failures) {
          result.success = false;
          result.message = "Aborting after " + std::to_string(consecutive_failures) +
            " consecutive failed patches (failed ids: " + failed_ids + ").";
          RCLCPP_ERROR(this->get_logger(), "%s", result.message.c_str());
          return Outcome::Failed;
        }
      }
    }

    publish_status(gh, "Grid complete. Moving to stow.");
    const bool stowed = trigger_named_pose(cfg_.stow_pose, stop);
    if (stop()) {return canceled(result, n_cut, total);}
    if (stowed) {arm_engaged_ = false;}

    result.success = (n_failed == 0) && stowed;
    result.message = "Cut " + std::to_string(n_cut) + "/" + std::to_string(total) + " patches";
    if (n_failed > 0) {result.message += "; failed patch ids: " + failed_ids;}
    if (!stowed) {result.message += "; failed to move to stow pose";}
    RCLCPP_INFO(this->get_logger(), "%s", result.message.c_str());
    return result.success ? Outcome::Succeeded : Outcome::Failed;
  }

  void execute_grid(const std::shared_ptr<GoalHandleCutStem> gh)
  {
    auto result = std::make_shared<CutStem::Result>();
    Outcome outcome = Outcome::Failed;

    try {
      outcome = run_grid(gh, *result);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Exception in grid execution: %s", e.what());
      result->success = false;
      result->message = std::string("Exception: ") + e.what();
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "Unknown exception in grid execution.");
      result->success = false;
      result->message = "Unknown exception";
    }

    try {
      if (outcome != Outcome::Succeeded) {safe_state();}
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Exception while moving to safe state: %s", e.what());
    }

    stop_servo();
    clear_markers();
    arm_engaged_ = false;

    is_executing_ = false;

    try {
      if (gh->is_canceling()) {
        result->success = false;
        gh->canceled(result);
      } else if (outcome == Outcome::Succeeded) {
        // Goal succeeded: clean up configured collision objects from the planning scene
        remove_loaded_collision_objects();
        gh->succeed(result);
      } else {
        gh->abort(result);
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to finalize goal: %s", e.what());
    }
  }

  // ---- Action server callbacks ----
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const CutStem::Goal>)
  {
    bool expected = false;
    if (shutting_down_ || !is_executing_.compare_exchange_strong(expected, true)) {
      RCLCPP_WARN(this->get_logger(), "Rejecting new goal: server is busy or shutting down.");
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(this->get_logger(), "Received goal request for CutStem");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleCutStem>)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    stop_servo();
    try {
      move_group_->stop();
    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "move_group stop failed: %s", e.what());
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleCutStem> goal_handle)
  {
    if (execution_thread_.joinable()) {
      execution_thread_.join();
    }
    execution_thread_ = std::thread(&GridCutterActionServer::execute_grid, this, goal_handle);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<GridCutterActionServer>(options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(node->get_moveit_node());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}