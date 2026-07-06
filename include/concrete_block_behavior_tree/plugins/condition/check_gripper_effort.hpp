#pragma once

#include <chrono>
#include <mutex>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace concrete_block_behavior_tree
{

// Decides whether the gripper has clamped on a block by watching the actuator
// effort published on /joint_states. Mirrors the forestry stack's
// CalcGripState approach (see epsilon_crane_behavior_tree), which does not
// rely on gazebo_grasp_fix's force-angle heuristic.
class CheckGripperEffort : public BT::StatefulActionNode
{
public:
  CheckGripperEffort(const std::string & name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic", "/joint_states", "Joint states topic"),
      BT::InputPort<std::string>("joint", "q9_left_rail_joint", "Gripper actuator joint name"),
      BT::InputPort<double>("min_effort", 1000.0, "Minimum |effort| to consider grip successful"),
      BT::InputPort<int>(
        "min_duration_ms", 500, "Minimum time |effort| must stay above threshold"),
      BT::InputPort<int>("timeout_ms", 5000, "Maximum wait time in milliseconds"),
      BT::InputPort<std::string>(
        "param_prefix", "check_gripper_effort",
        "ROS parameter prefix for deployment-specific overrides"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using JointState = sensor_msgs::msg::JointState;

  void callback(const JointState::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<JointState>::SharedPtr sub_;

  std::mutex mutex_;
  bool have_sample_{false};
  bool above_threshold_{false};
  double last_effort_{0.0};

  std::string param_prefix_{"check_gripper_effort"};
  std::string topic_;
  std::string joint_;
  double min_effort_{1000.0};
  int min_duration_ms_{500};
  int timeout_ms_{5000};
  std::chrono::steady_clock::time_point start_time_;
  std::chrono::steady_clock::time_point above_threshold_since_;
};

}  // namespace concrete_block_behavior_tree
