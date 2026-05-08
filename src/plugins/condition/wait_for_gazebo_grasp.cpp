#include "concrete_block_behavior_tree/plugins/condition/wait_for_gazebo_grasp.hpp"

#include <functional>
#include <stdexcept>
#include <utility>

#include "behaviortree_cpp_v3/bt_factory.h"

namespace concrete_block_behavior_tree
{

WaitForGazeboGrasp::WaitForGazeboGrasp(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  getInput("topic", topic_);
  if (topic_.empty()) {
    throw std::invalid_argument("WaitForGazeboGrasp input 'topic' must not be empty");
  }

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(
    callback_group_,
    node_->get_node_base_interface());

  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  sub_ = node_->create_subscription<GraspEvent>(
    topic_,
    rclcpp::QoS(10),
    std::bind(&WaitForGazeboGrasp::callback, this, std::placeholders::_1),
    options);

  RCLCPP_INFO(
    node_->get_logger(),
    "WaitForGazeboGrasp initialized, listening to %s",
    topic_.c_str());
}

BT::NodeStatus WaitForGazeboGrasp::onStart()
{
  getInput("arm", arm_);
  getInput("object_contains", object_contains_);
  getInput("attached", expected_attached_);
  getInput("timeout_ms", timeout_ms_);

  {
    std::lock_guard<std::mutex> lock(mutex_);
    matched_ = false;
    last_match_ = GraspEvent();
  }

  start_time_ = std::chrono::steady_clock::now();
  RCLCPP_INFO(
    node_->get_logger(),
    "Waiting for Gazebo grasp event | arm='%s' object_contains='%s' attached=%s timeout_ms=%d",
    arm_.c_str(),
    object_contains_.c_str(),
    expected_attached_ ? "true" : "false",
    timeout_ms_);

  return onRunning();
}

BT::NodeStatus WaitForGazeboGrasp::onRunning()
{
  callback_group_executor_.spin_some();

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (matched_) {
      RCLCPP_INFO(
        node_->get_logger(),
        "Matched Gazebo grasp event | arm='%s' object='%s' attached=%s",
        last_match_.arm.c_str(),
        last_match_.object.c_str(),
        last_match_.attached ? "true" : "false");
      return BT::NodeStatus::SUCCESS;
    }
  }

  const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - start_time_).count();
  if (elapsed_ms > timeout_ms_) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Timed out waiting for Gazebo grasp event | arm='%s' object_contains='%s' attached=%s",
      arm_.c_str(),
      object_contains_.c_str(),
      expected_attached_ ? "true" : "false");
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void WaitForGazeboGrasp::onHalted()
{
  std::lock_guard<std::mutex> lock(mutex_);
  matched_ = false;
}

void WaitForGazeboGrasp::callback(const GraspEvent::SharedPtr msg)
{
  if (!msg || !matches(*msg)) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  matched_ = true;
  last_match_ = *msg;
}

bool WaitForGazeboGrasp::matches(const GraspEvent & msg) const
{
  if (!arm_.empty() && msg.arm != arm_) {
    return false;
  }
  if (!object_contains_.empty() && msg.object.find(object_contains_) == std::string::npos) {
    return false;
  }
  return msg.attached == expected_attached_;
}

}  // namespace concrete_block_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<concrete_block_behavior_tree::WaitForGazeboGrasp>(
    "WaitForGazeboGrasp");
}
