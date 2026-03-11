#pragma once

#include <memory>
#include <string>

#include "concrete_block_motion_planning/srv/plan_and_compute_trajectory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_behavior_tree/bt_service_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace concrete_block_behavior_tree
{

class PlanAndComputeTrajectoryService
  : public nav2_behavior_tree::BtServiceNode<concrete_block_motion_planning::srv::PlanAndComputeTrajectory>
{
public:
  using ServiceT = concrete_block_motion_planning::srv::PlanAndComputeTrajectory;
  using ResponseT = concrete_block_motion_planning::srv::PlanAndComputeTrajectory_Response;

  PlanAndComputeTrajectoryService(const std::string & service_name, const BT::NodeConfiguration & conf)
  : nav2_behavior_tree::BtServiceNode<ServiceT>(service_name, conf) {}

  void on_tick() override;
  BT::NodeStatus on_completion(std::shared_ptr<ResponseT> response) override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<geometry_msgs::msg::PoseStamped>("start_pose"),
        BT::InputPort<geometry_msgs::msg::PoseStamped>("goal_pose"),
        BT::InputPort<std::string>("target_block_id", "", "Target block id"),
        BT::InputPort<std::string>("reference_block_id", "", "Reference block id"),
        BT::InputPort<bool>("use_world_model", true, "Use current world model"),
        BT::InputPort<std::string>("geometric_method", "POWELL", "Geometric planning method"),
        BT::InputPort<double>("geometric_timeout_s", 5.0, "Geometric planning timeout"),
        BT::InputPort<std::string>("trajectory_method", "ACADOS_PATH_FOLLOWING", "Trajectory method"),
        BT::InputPort<double>("trajectory_timeout_s", 10.0, "Trajectory timeout"),
        BT::InputPort<bool>("validate_dynamics", true, "Require dynamics validation"),
        BT::OutputPort<std::string>("geometric_plan_id"),
        BT::OutputPort<nav_msgs::msg::Path>("cartesian_path"),
        BT::OutputPort<std::string>("trajectory_id"),
        BT::OutputPort<trajectory_msgs::msg::JointTrajectory>("joint_trajectory"),
        BT::OutputPort<bool>("trajectory_ok"),
        BT::OutputPort<std::string>("trajectory_message")
      });
  }
};

}  // namespace concrete_block_behavior_tree
