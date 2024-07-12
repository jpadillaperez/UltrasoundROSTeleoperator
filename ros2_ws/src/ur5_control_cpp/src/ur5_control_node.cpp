#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <Eigen/Geometry>
#include <mutex>
#include <vector>
#include <thread>
#include <cmath>
#include <chrono>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("ur5_human");

class UR5Human : public rclcpp::Node
{
public:
  UR5Human()
    : Node("ur5_human"),
      max_waypoints_(10),
      last_waypoint_time_(std::chrono::steady_clock::now()),
      unity_confirmation_received_(false)
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/body_tracking_data", 10,
      std::bind(&UR5Human::bodyTrackingCallback, this, std::placeholders::_1));

    trajectory_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/trajectory_to_unity", 10);

    unity_confirmation_subscription_ = this->create_subscription<std_msgs::msg::String>(
      "/unity_confirmation", 10,
      std::bind(&UR5Human::unityConfirmationCallback, this, std::placeholders::_1));
  }

  void init()
  {
    static const std::string PLANNING_GROUP = "ur_manipulator";
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), PLANNING_GROUP);
    visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
      shared_from_this(), "base_link", "rviz_visual_tools", move_group_->getRobotModel());
    visual_tools_->deleteAllMarkers();
    visual_tools_->loadRemoteControl();
    
    // Disable waiting for subscribers
    visual_tools_->trigger();
  }

  void bodyTrackingCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    auto current_time = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_waypoint_time_).count() >= 500) {
      if (waypoints_.size() < max_waypoints_) {
        auto joint_positions = processBodyTrackingData(msg->name, msg->position);
        if (joint_positions.count("SHOULDER_RIGHT") == 0 || 
            joint_positions.count("ELBOW_RIGHT")    == 0 || 
            joint_positions.count("WRIST_RIGHT")    == 0 ||
            joint_positions.count("THUMB_RIGHT")    == 0 ||
            joint_positions.count("HANDTIP_RIGHT") == 0) {
          RCLCPP_WARN(LOGGER, "Missing required joint positions.");
          return;
        }

        std::vector<double> joint_angles = calculateJointAngles(joint_positions);
        if (joint_angles.empty()) {
          RCLCPP_WARN(LOGGER, "Joint angles out of bounds, skipping waypoints.");
          return;
        }

        waypoints_.push_back(joint_angles);
        RCLCPP_INFO(LOGGER, "Collected waypoint %zu.", waypoints_.size());
        last_waypoint_time_ = current_time;
      }
    }
  }

  bool isUnityConfirmationReceived() const {
    return unity_confirmation_received_;
  }

  void planAndExecute()
  {
    visual_tools_->publishText(Eigen::Isometry3d::Identity(), "UR5Human", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    visual_tools_->trigger();

    // Define the initial joint positions (arm stretched horizontally)
    std::vector<double> initial_joint_positions = {0.0, -M_PI/2, 0.0, -M_PI/2, 0.0, 0.0};
    move_group_->setJointValueTarget(initial_joint_positions);

    RCLCPP_INFO(LOGGER, "Planning initial trajectory.");
    moveit::planning_interface::MoveGroupInterface::Plan initial_plan;
    bool success = (move_group_->plan(initial_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
      RCLCPP_INFO(LOGGER, "Initial planning succeeded, executing initial plan.");
      move_group_->execute(initial_plan);
      waypoints_.insert(waypoints_.begin(), initial_joint_positions);  // Insert initial position as the first waypoint
    } else {
      RCLCPP_ERROR(LOGGER, "Initial planning failed.");
      return;
    }

    if (waypoints_.empty()) {
      RCLCPP_WARN(LOGGER, "No waypoints collected.");
      return;
    }

    // Interpolate between waypoints to generate a smooth trajectory
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double time_from_start = 0.1; // Adjust this value to control the speed
    trajectory.joint_trajectory.joint_names = move_group_->getJointNames();

    RCLCPP_INFO(LOGGER, "Interpolating waypoints to generate trajectory.");
    for (size_t i = 0; i < waypoints_.size() - 1; ++i) {
      std::vector<double> start_point = waypoints_[i];
      std::vector<double> end_point = waypoints_[i + 1];

      // Interpolate between start_point and end_point
      for (double alpha = 0.0; alpha <= 1.0; alpha += 0.05) {
        std::vector<double> interpolated_point;
        for (size_t j = 0; j < start_point.size(); ++j) {
          double value = start_point[j] * (1 - alpha) + end_point[j] * alpha;
          interpolated_point.push_back(value);
        }
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = interpolated_point;
        point.time_from_start = rclcpp::Duration::from_seconds(trajectory.joint_trajectory.points.size() * time_from_start);
        trajectory.joint_trajectory.points.push_back(point);
      }
    }

    if (trajectory.joint_trajectory.points.size() > 0) {
      RCLCPP_INFO(LOGGER, "Trajectory has %zu points.", trajectory.joint_trajectory.points.size());

      // Time-parameterize the trajectory
      robot_trajectory::RobotTrajectory rt(move_group_->getCurrentState()->getRobotModel(), "ur_manipulator");
      rt.setRobotTrajectoryMsg(*move_group_->getCurrentState(), trajectory);
      rt.getRobotTrajectoryMsg(trajectory);

      moveit::planning_interface::MoveGroupInterface::Plan smooth_plan;
      smooth_plan.trajectory_ = trajectory;

      // Visualize the trajectory
      visual_tools_->publishTrajectoryLine(smooth_plan.trajectory_, move_group_->getCurrentState()->getJointModelGroup("ur_manipulator"));
      visual_tools_->trigger();

      // Publish the trajectory to Unity
      trajectory_publisher_->publish(smooth_plan.trajectory_.joint_trajectory);
      RCLCPP_INFO(LOGGER, "Trajectory published to Unity. Waiting for confirmation...");

      // Wait for confirmation from Unity
			auto start_time = std::chrono::steady_clock::now();
			while (!isUnityConfirmationReceived() && rclcpp::ok()) {
					auto current_time = std::chrono::steady_clock::now();
					if (std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count() > 30) {
							RCLCPP_ERROR(LOGGER, "Timeout waiting for Unity confirmation.");
							return;
					}
					std::this_thread::sleep_for(std::chrono::milliseconds(100));
			}

			if (!rclcpp::ok()) {
					RCLCPP_ERROR(LOGGER, "ROS context is no longer valid. Exiting.");
					return;
			}

      // Reset the confirmation flag
      unity_confirmation_received_ = false;

      // Execute the plan
      RCLCPP_INFO(LOGGER, "Executing smooth plan.");
      bool execution_success = (move_group_->execute(smooth_plan) == moveit::core::MoveItErrorCode::SUCCESS);
      if (!execution_success) {
        RCLCPP_ERROR(LOGGER, "Execution of smooth plan failed.");
      }
    } else {
      RCLCPP_ERROR(LOGGER, "No valid trajectory points found.");
    }
  }

  void unityConfirmationCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (msg->data == "CONFIRMED") {
      RCLCPP_INFO(LOGGER, "Received confirmation from Unity.");
      unity_confirmation_received_ = true;
    }
  }

private:
  std::unordered_map<std::string, geometry_msgs::msg::Point> processBodyTrackingData(const std::vector<std::string>& names, const std::vector<double>& positions)
  {
    std::unordered_map<std::string, geometry_msgs::msg::Point> joint_positions;
    for (size_t i = 0; i < names.size(); ++i) {
      geometry_msgs::msg::Point point;
      point.x = positions[3 * i];
      point.y = positions[3 * i + 1];
      point.z = positions[3 * i + 2];
      joint_positions[names[i]] = point;
    }
    return joint_positions;
  }

  std::vector<double> calculateJointAngles(const std::unordered_map<std::string, geometry_msgs::msg::Point>& joint_positions)
  {
    std::vector<double> joint_angles(6);

    auto shoulder = joint_positions.at("SHOULDER_RIGHT");
    auto elbow = joint_positions.at("ELBOW_RIGHT");
    auto wrist = joint_positions.at("WRIST_RIGHT");
    auto hand = joint_positions.at("HAND_RIGHT");
    auto handtip = joint_positions.at("HANDTIP_RIGHT");
    auto thumb = joint_positions.at("THUMB_RIGHT");

    // Shoulder pan joint (angle between elbow-shoulder projection on XZ plane and X-axis)
    double dx = elbow.x - shoulder.x;
    double dz = elbow.z - shoulder.z;
    
    // Project the elbow-shoulder vector onto the XZ plane
    double projected_length = std::sqrt(dx*dx + dz*dz);
    
    // Calculate the angle, considering the quadrant
    double shoulder_pan_angle = std::atan2(-dz, dx);  // Changed to use dx instead of -dx
    
    // Map the angle to the range [-π/2, π/2]
    if (shoulder_pan_angle > M_PI_2) {
        shoulder_pan_angle = M_PI - shoulder_pan_angle;
    } else if (shoulder_pan_angle < -M_PI_2) {
        shoulder_pan_angle = -M_PI - shoulder_pan_angle;
    }
    
    // Use the projected length to scale the angle
    double scale_factor = std::min(1.0, projected_length / 0.3);  // Assuming 0.3 meters as a typical arm length
    shoulder_pan_angle *= scale_factor;

    // Initial positions: std::vector<double> initial_joint_positions = {0.0, -M_PI/2, 0.0, -M_PI/2, 0.0, 0.0};

    joint_angles[0] = shoulder_pan_angle;
    // Shoulder lift joint (angle on XY plane)
    double shoulder_lift_angle = std::atan2(elbow.y - shoulder.y, std::sqrt(std::pow(elbow.x - shoulder.x, 2) + std::pow(elbow.z - shoulder.z, 2)));
    joint_angles[1] = shoulder_lift_angle - M_PI / 2;  // Shift by -90 degrees to align with initial position

    // Elbow joint
    Eigen::Vector3d shoulder_to_elbow(elbow.x - shoulder.x, elbow.y - shoulder.y, elbow.z - shoulder.z);
    Eigen::Vector3d elbow_to_wrist(wrist.x - elbow.x, wrist.y - elbow.y, wrist.z - elbow.z);
    double elbow_angle = std::acos(shoulder_to_elbow.normalized().dot(elbow_to_wrist.normalized()));
    joint_angles[2] = elbow_angle;

    // Wrist 1 (flexion/extension)
    Eigen::Vector3d wrist_direction = (Eigen::Vector3d(handtip.x, handtip.y, handtip.z) - Eigen::Vector3d(wrist.x, wrist.y, wrist.z)).normalized();
    double wrist_flex = std::atan2(wrist_direction.z(), std::sqrt(wrist_direction.x()*wrist_direction.x() + wrist_direction.y()*wrist_direction.y()));
    joint_angles[3] = -M_PI/2 + wrist_flex;  // Shift by -90 degrees to align with initial position

    // Wrist 2 (ulnar/radial deviation)
    Eigen::Vector3d thumb_to_hand = (Eigen::Vector3d(thumb.x, thumb.y, thumb.z) - Eigen::Vector3d(hand.x, hand.y, hand.z)).normalized();
    double wrist_dev = std::atan2(thumb_to_hand.y(), thumb_to_hand.x());
    joint_angles[4] = wrist_dev;

    // Wrist 3 (forearm rotation)
    joint_angles[5] = 0.0;  // Set to constant 0

    // Print computed angles
    const double rad_to_deg = 180.0 / M_PI;
    RCLCPP_INFO(rclcpp::get_logger("ur5_control"), "Computed angles (in degrees):");
    RCLCPP_INFO(rclcpp::get_logger("ur5_control"), "Shoulder pan angle: %f", joint_angles[0] * rad_to_deg);
    RCLCPP_INFO(rclcpp::get_logger("ur5_control"), "Shoulder lift angle: %f", joint_angles[1] * rad_to_deg);
    RCLCPP_INFO(rclcpp::get_logger("ur5_control"), "--------------------");
    RCLCPP_INFO(rclcpp::get_logger("ur5_control"), "Elbow angle: %f", joint_angles[2] * rad_to_deg);
    RCLCPP_INFO(rclcpp::get_logger("ur5_control"), "Wrist 1 angle: %f", joint_angles[3] * rad_to_deg);
    RCLCPP_INFO(rclcpp::get_logger("ur5_control"), "Wrist 2 angle: %f", joint_angles[4] * rad_to_deg);
    RCLCPP_INFO(rclcpp::get_logger("ur5_control"), "End Effector angle: %f", joint_angles[5] * rad_to_deg);

    // Apply joint limits
    const double deg_to_rad = M_PI / 180.0;
    joint_angles[0] = std::clamp(joint_angles[0], -360.0 * deg_to_rad, 360.0 * deg_to_rad);
    joint_angles[1] = std::clamp(joint_angles[1], -360.0 * deg_to_rad, 360.0 * deg_to_rad);
    joint_angles[2] = std::clamp(joint_angles[2], -180.0 * deg_to_rad, 180.0 * deg_to_rad);
    joint_angles[3] = std::clamp(joint_angles[3], -360.0 * deg_to_rad, 360.0 * deg_to_rad);
    joint_angles[4] = std::clamp(joint_angles[4], -360.0 * deg_to_rad, 360.0 * deg_to_rad);
    joint_angles[5] = std::clamp(joint_angles[5], -360.0 * deg_to_rad, 360.0 * deg_to_rad);

    // Validate the joint angles
    auto robot_model = move_group_->getRobotModel();
    if (!robot_model->satisfiesPositionBounds(joint_angles.data(), robot_model->getActiveJointModelsBounds())) {
        RCLCPP_WARN(rclcpp::get_logger("ur5_control"), "Joint angles out of bounds");
        return {};
    }

    return joint_angles;
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;
  std::vector<std::vector<double>> waypoints_;
  const size_t max_waypoints_;
  std::chrono::steady_clock::time_point last_waypoint_time_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr unity_confirmation_subscription_;
  bool unity_confirmation_received_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UR5Human>();
  node->init();

  rclcpp::executors::SingleThreadedExecutor executor;
	
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // Wait for a few seconds to collect waypoints
  rclcpp::sleep_for(std::chrono::seconds(10));

  node->planAndExecute();

  rclcpp::shutdown();
  return 0;
}
