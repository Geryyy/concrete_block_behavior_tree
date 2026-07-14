#include "concrete_block_behavior_tree/plugins/condition/wait_for_grasp_signal.hpp"

#include <stdexcept>

#include "behaviortree_cpp_v3/bt_factory.h"

namespace concrete_block_behavior_tree
{

WaitForGraspSignal::WaitForGraspSignal(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  getInput("topic", topic_);
  if (topic_.empty()) {
    throw std::invalid_argument("WaitForGraspSignal input 'topic' must not be empty");
  }

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(
    callback_group_,
    node_->get_node_base_interface());

  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  sub_ = node_->create_subscription<Bool>(
    topic_,
    rclcpp::QoS(10).transient_local(),
    std::bind(&WaitForGraspSignal::callback, this, std::placeholders::_1),
    options);

  RCLCPP_INFO(
    node_->get_logger(),
    "WaitForGraspSignal initialized, listening to %s",
    topic_.c_str());
}

BT::NodeStatus WaitForGraspSignal::onStart()
{
  getInput("timeout_ms", timeout_ms_);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    grasp_detected_ = false;
  }

  start_time_ = std::chrono::steady_clock::now();
  RCLCPP_INFO(
    node_->get_logger(),
    "Waiting for grasp signal | topic='%s' timeout_ms=%d",
    topic_.c_str(), timeout_ms_);

  return onRunning();
}

BT::NodeStatus WaitForGraspSignal::onRunning()
{
  callback_group_executor_.spin_some();

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (grasp_detected_) {
      RCLCPP_INFO(node_->get_logger(), "Grasp signal received");
      return BT::NodeStatus::SUCCESS;
    }
  }

  const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - start_time_).count();
  if (elapsed_ms > timeout_ms_) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Timed out waiting for grasp signal | topic='%s'",
      topic_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void WaitForGraspSignal::onHalted()
{
  std::lock_guard<std::mutex> lock(mutex_);
  grasp_detected_ = false;
}

void WaitForGraspSignal::callback(const Bool::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  grasp_detected_ = msg->data;
}

}  // namespace concrete_block_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<concrete_block_behavior_tree::WaitForGraspSignal>(
    "WaitForGraspSignal");
}
