#pragma once

#include <memory>
#include <string>

#include "concrete_block_motion_planning/srv/compute_trajectory.hpp"
#include "nav2_behavior_tree/bt_service_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace concrete_block_behavior_tree
{

class ComputeTrajectoryService
  : public nav2_behavior_tree::BtServiceNode<concrete_block_motion_planning::srv::ComputeTrajectory>
{
public:
  using ServiceT = concrete_block_motion_planning::srv::ComputeTrajectory;
  using ResponseT = concrete_block_motion_planning::srv::ComputeTrajectory_Response;

  ComputeTrajectoryService(const std::string & service_name, const BT::NodeConfiguration & conf)
  : nav2_behavior_tree::BtServiceNode<ServiceT>(service_name, conf) {}

  void on_tick() override;
  BT::NodeStatus on_completion(std::shared_ptr<ResponseT> response) override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<std::string>("geometric_plan_id", "", "Geometric plan id"),
        BT::InputPort<nav_msgs::msg::Path>("cartesian_path"),
        BT::InputPort<bool>("use_direct_path", false, "Use cartesian_path directly"),
        BT::InputPort<std::string>("method", "ACADOS_PATH_FOLLOWING", "Trajectory method"),
        BT::InputPort<double>("timeout_s", 10.0, "Trajectory timeout"),
        BT::InputPort<bool>("validate_dynamics", true, "Require dynamics validation"),
        BT::OutputPort<std::string>("trajectory_id"),
        BT::OutputPort<trajectory_msgs::msg::JointTrajectory>("joint_trajectory"),
        BT::OutputPort<bool>("trajectory_ok"),
        BT::OutputPort<std::string>("trajectory_message")
      });
  }
};

}  // namespace concrete_block_behavior_tree
