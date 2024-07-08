#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <Eigen/Geometry>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("simple_move_group");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("simple_move_group_node");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP = "ur_manipulator";

  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(node, "base_link", "rviz_visual_tools", move_group.getRobotModel());
  visual_tools.deleteAllMarkers();

  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

  visual_tools.publishText(Eigen::Isometry3d::Identity(), "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // Define the initial joint positions
  std::vector<double> initial_joint_positions = {0.0, -1.57, 0.0, -1.57, 0.0, 0.0};
  move_group.setJointValueTarget(initial_joint_positions);

  moveit::planning_interface::MoveGroupInterface::Plan initial_plan;
  bool success = (move_group.plan(initial_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success) {
    RCLCPP_INFO(LOGGER, "Initial planning succeeded, executing initial plan");
    move_group.execute(initial_plan);
  } else {
    RCLCPP_ERROR(LOGGER, "Initial planning failed");
    return 1;
  }

  // Define multiple joint waypoints
  std::vector<std::vector<double>> waypoints = {
    {0.0, -1.57, 0.0, 0.0, 0.0, 0.0},
    {0.0, -1.57 + M_PI / 4, 0.0, 0.0, 0.0, 0.0},
    {0.0, -1.57 + M_PI / 2, 0.0, 0.0, 0.0, 0.0},
    {0.0, -1.57 + 3 * M_PI / 4, 0.0, 0.0, 0.0, 0.0},
    {0.0, -1.57 + M_PI, 0.0, 0.0, 0.0, 0.0},
    {0.0, -1.57 + 5 * M_PI / 4, 0.0, 0.0, 0.0, 0.0},
    {0.0, -1.57 + 3 * M_PI / 2, 0.0, 0.0, 0.0, 0.0},
    {0.0, -1.57 + 7 * M_PI / 4, 0.0, 0.0, 0.0, 0.0},
    {0.0, -1.57 + 2 * M_PI, 0.0, 0.0, 0.0, 0.0}
  };

  // Plan and execute the trajectory
  moveit_msgs::msg::RobotTrajectory combined_trajectory;
  moveit::core::RobotState start_state(*move_group.getCurrentState());

  // Define a RobotTrajectory object
  robot_trajectory::RobotTrajectory rt(move_group.getRobotModel(), PLANNING_GROUP);

  for (const auto& waypoint : waypoints) {
    move_group.setJointValueTarget(waypoint);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success) {
      RCLCPP_ERROR(LOGGER, "Planning to waypoint failed");
      return 1;
    }

    // Append the plan to the RobotTrajectory
    robot_trajectory::RobotTrajectory temp_trajectory(move_group.getRobotModel(), PLANNING_GROUP);
    temp_trajectory.setRobotTrajectoryMsg(*move_group.getCurrentState(), plan.trajectory_);
    rt.append(temp_trajectory, 0.01); // Append the new trajectory points to the combined plan

    // Update the start state to the last point of the current trajectory
    start_state.setJointGroupPositions(PLANNING_GROUP, waypoint);
    move_group.setStartState(start_state);
  }

  rt.getRobotTrajectoryMsg(combined_trajectory);

  // Print all the waypoints for debugging
  for (size_t i = 0; i < combined_trajectory.joint_trajectory.points.size(); ++i) {
    RCLCPP_INFO(LOGGER, "Waypoint %zu:", i);
    for (size_t j = 0; j < combined_trajectory.joint_trajectory.points[i].positions.size(); ++j) {
      RCLCPP_INFO(LOGGER, "  Joint %zu: %f", j, combined_trajectory.joint_trajectory.points[i].positions[j]);
    }
  }

  if (combined_trajectory.joint_trajectory.points.size() > 0) {
    moveit::planning_interface::MoveGroupInterface::Plan combined_plan;
    combined_plan.trajectory_ = combined_trajectory;

    // Visualize the combined plan
    visual_tools.publishTrajectoryLine(combined_plan.trajectory_, move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP));
    visual_tools.trigger();

    // Execute the combined plan
    RCLCPP_INFO(LOGGER, "Executing combined plan");
    bool execution_success = (move_group.execute(combined_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!execution_success) {
      RCLCPP_ERROR(LOGGER, "Execution of combined plan failed");
    }
  } else {
    RCLCPP_ERROR(LOGGER, "No valid trajectory points found");
  }

  rclcpp::shutdown();
  return 0;
}
