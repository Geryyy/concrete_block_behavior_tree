#pragma once

#include <chrono>
#include <mutex>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace concrete_block_behavior_tree
{

class WaitForGraspSignal : public BT::StatefulActionNode
{
public:
  WaitForGraspSignal(const std::string & name, const BT::NodeConfig & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic", "/gripper/grasp_detected", "Bool grasp signal topic"),
      BT::InputPort<int>("timeout_ms", 8000, "Maximum wait time in milliseconds"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using Bool = std_msgs::msg::Bool;

  void callback(const Bool::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<Bool>::SharedPtr sub_;

  std::mutex mutex_;
  bool grasp_detected_{false};
  std::string topic_;
  int timeout_ms_{8000};
  std::chrono::steady_clock::time_point start_time_;
};

}  // namespace concrete_block_behavior_tree
