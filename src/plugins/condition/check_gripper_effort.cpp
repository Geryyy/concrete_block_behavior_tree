#include "concrete_block_behavior_tree/plugins/condition/check_gripper_effort.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "behaviortree_cpp_v3/bt_factory.h"

namespace concrete_block_behavior_tree
{

CheckGripperEffort::CheckGripperEffort(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  getInput("topic", topic_);
  if (topic_.empty()) {
    throw std::invalid_argument("CheckGripperEffort input 'topic' must not be empty");
  }

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(
    callback_group_,
    node_->get_node_base_interface());

  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  sub_ = node_->create_subscription<JointState>(
    topic_,
    rclcpp::QoS(10),
    std::bind(&CheckGripperEffort::callback, this, std::placeholders::_1),
    options);

  std::string command_topic;
  getInput("grasp_command_topic", command_topic);
  if (!command_topic.empty()) {
    command_pub_ = node_->create_publisher<GraspEvent>(command_topic, 10);
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "CheckGripperEffort initialized, listening to %s",
    topic_.c_str());
}

BT::NodeStatus CheckGripperEffort::onStart()
{
  getInput("joint", joint_);
  getInput("min_effort", min_effort_);
  getInput("timeout_ms", timeout_ms_);

  {
    std::lock_guard<std::mutex> lock(mutex_);
    have_sample_ = false;
    last_effort_ = 0.0;
  }

  start_time_ = std::chrono::steady_clock::now();
  RCLCPP_INFO(
    node_->get_logger(),
    "Checking gripper grip | joint='%s' min_effort=%.1f timeout_ms=%d",
    joint_.c_str(), min_effort_, timeout_ms_);

  return onRunning();
}

BT::NodeStatus CheckGripperEffort::onRunning()
{
  callback_group_executor_.spin_some();

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (have_sample_ && std::abs(last_effort_) >= min_effort_) {
      RCLCPP_INFO(
        node_->get_logger(),
        "Grip detected | joint='%s' effort=%.1f >= %.1f",
        joint_.c_str(), last_effort_, min_effort_);
      publishAttachCommand();
      return BT::NodeStatus::SUCCESS;
    }
  }

  const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - start_time_).count();
  if (elapsed_ms > timeout_ms_) {
    std::lock_guard<std::mutex> lock(mutex_);
    RCLCPP_WARN(
      node_->get_logger(),
      "Grip check timed out | joint='%s' last_effort=%.1f < %.1f",
      joint_.c_str(), last_effort_, min_effort_);
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void CheckGripperEffort::onHalted()
{
  std::lock_guard<std::mutex> lock(mutex_);
  have_sample_ = false;
}

void CheckGripperEffort::publishAttachCommand()
{
  if (!command_pub_) {return;}
  std::string arm;
  std::string object;
  getInput("arm", arm);
  getInput("object", object);
  if (object.empty()) {
    RCLCPP_WARN(
      node_->get_logger(),
      "CheckGripperEffort: 'object' input is empty; skipping attach command");
    return;
  }
  GraspEvent cmd;
  cmd.arm = arm;
  cmd.object = object;
  cmd.attached = true;
  command_pub_->publish(cmd);
  RCLCPP_INFO(
    node_->get_logger(),
    "Published grasp attach command | arm='%s' object='%s'",
    arm.c_str(), object.c_str());
}

void CheckGripperEffort::callback(const JointState::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  auto it = std::find(msg->name.begin(), msg->name.end(), joint_);
  if (it == msg->name.end()) {
    return;
  }
  const size_t idx = static_cast<size_t>(std::distance(msg->name.begin(), it));
  if (idx >= msg->effort.size()) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  last_effort_ = msg->effort[idx];
  have_sample_ = true;
}

}  // namespace concrete_block_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<concrete_block_behavior_tree::CheckGripperEffort>(
    "CheckGripperEffort");
}
