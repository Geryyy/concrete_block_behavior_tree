#pragma once

#include <chrono>
#include <mutex>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "gazebo_grasp_plugin_ros/msg/gazebo_grasp_event.hpp"
#include "rclcpp/rclcpp.hpp"

namespace concrete_block_behavior_tree
{

class WaitForGazeboGrasp : public BT::StatefulActionNode
{
public:
  WaitForGazeboGrasp(const std::string & name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "topic", "/grasp_events", "Gazebo grasp event topic"),
      BT::InputPort<std::string>("arm", "gripper", "Expected Gazebo grasp arm name"),
      BT::InputPort<std::string>(
        "object_contains", "", "Substring expected in the grasped object collision name"),
      BT::InputPort<bool>("attached", true, "Expected attached state"),
      BT::InputPort<int>("timeout_ms", 3000, "Maximum wait time in milliseconds")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using GraspEvent = gazebo_grasp_plugin_ros::msg::GazeboGraspEvent;

  void callback(const GraspEvent::SharedPtr msg);
  bool matches(const GraspEvent & msg) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<GraspEvent>::SharedPtr sub_;

  std::mutex mutex_;
  bool matched_{false};
  GraspEvent last_match_;

  std::string topic_;
  std::string arm_;
  std::string object_contains_;
  bool expected_attached_{true};
  int timeout_ms_{3000};
  std::chrono::steady_clock::time_point start_time_;
};

}  // namespace concrete_block_behavior_tree
