#include "concrete_block_behavior_tree/plugins/condition/check_gripper_effort.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "behaviortree_cpp_v3/bt_factory.h"

namespace concrete_block_behavior_tree
{

namespace
{

template<typename T>
T getParameterOrDeclare(
  const rclcpp::Node::SharedPtr & node,
  const std::string & name,
  const T & default_value)
{
  if (!node->has_parameter(name)) {
    try {
      node->declare_parameter<T>(name, default_value);
    } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
    }
  }

  T value = default_value;
  node->get_parameter(name, value);
  return value;
}

}  // namespace

CheckGripperEffort::CheckGripperEffort(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  getInput("param_prefix", param_prefix_);
  getInput("topic", topic_);
  topic_ = getParameterOrDeclare(node_, param_prefix_ + ".topic", topic_);
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

  RCLCPP_INFO(
    node_->get_logger(),
    "CheckGripperEffort initialized, listening to %s",
    topic_.c_str());
}

BT::NodeStatus CheckGripperEffort::onStart()
{
  getInput("joint", joint_);
  getInput("min_effort", min_effort_);
  getInput("min_duration_ms", min_duration_ms_);
  getInput("timeout_ms", timeout_ms_);
  joint_ = getParameterOrDeclare(node_, param_prefix_ + ".joint", joint_);
  min_effort_ = getParameterOrDeclare(node_, param_prefix_ + ".min_effort", min_effort_);
  min_duration_ms_ = getParameterOrDeclare(
    node_, param_prefix_ + ".min_duration_ms", min_duration_ms_);
  timeout_ms_ = getParameterOrDeclare(node_, param_prefix_ + ".timeout_ms", timeout_ms_);

  {
    std::lock_guard<std::mutex> lock(mutex_);
    have_sample_ = false;
    above_threshold_ = false;
    last_effort_ = 0.0;
  }

  start_time_ = std::chrono::steady_clock::now();
  RCLCPP_INFO(
    node_->get_logger(),
    "Checking gripper grip | joint='%s' min_effort=%.1f min_duration_ms=%d timeout_ms=%d",
    joint_.c_str(), min_effort_, min_duration_ms_, timeout_ms_);

  return onRunning();
}

BT::NodeStatus CheckGripperEffort::onRunning()
{
  callback_group_executor_.spin_some();

  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = std::chrono::steady_clock::now();
    const bool currently_above = have_sample_ && std::abs(last_effort_) >= min_effort_;
    if (currently_above) {
      if (!above_threshold_) {
        above_threshold_ = true;
        above_threshold_since_ = now;
      }
      const auto held_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - above_threshold_since_).count();
      if (held_ms >= min_duration_ms_) {
        RCLCPP_INFO(
          node_->get_logger(),
          "Grip detected | joint='%s' effort=%.1f >= %.1f for %ld ms",
          joint_.c_str(), last_effort_, min_effort_, held_ms);
        return BT::NodeStatus::SUCCESS;
      }
    } else {
      above_threshold_ = false;
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
  above_threshold_ = false;
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
