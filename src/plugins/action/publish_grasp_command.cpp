#include "concrete_block_behavior_tree/plugins/action/publish_grasp_command.hpp"

#include <stdexcept>

#include "behaviortree_cpp_v3/bt_factory.h"

namespace concrete_block_behavior_tree
{

PublishGraspCommand::PublishGraspCommand(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  getInput("topic", topic_);
  if (topic_.empty()) {
    throw std::invalid_argument("PublishGraspCommand input 'topic' must not be empty");
  }
  publisher_ = node_->create_publisher<GraspEvent>(topic_, 10);
}

BT::NodeStatus PublishGraspCommand::tick()
{
  std::string arm;
  std::string object;
  bool attached = true;
  getInput("arm", arm);
  getInput("object", object);
  getInput("attached", attached);

  if (object.empty()) {
    RCLCPP_WARN(
      node_->get_logger(),
      "PublishGraspCommand: 'object' input is empty; not publishing");
    return BT::NodeStatus::FAILURE;
  }

  GraspEvent cmd;
  cmd.arm = arm;
  cmd.object = object;
  cmd.attached = attached;
  publisher_->publish(cmd);
  RCLCPP_INFO(
    node_->get_logger(),
    "Published grasp command | arm='%s' object='%s' attached=%s",
    arm.c_str(), object.c_str(), attached ? "true" : "false");
  return BT::NodeStatus::SUCCESS;
}

}  // namespace concrete_block_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<concrete_block_behavior_tree::PublishGraspCommand>(
    "PublishGraspCommand");
}
