#include "concrete_block_behavior_tree/plugins/action/gripper_action.hpp"

#include <sstream>

#include "behaviortree_cpp_v3/bt_factory.h"

namespace concrete_block_behavior_tree
{

GripperActionStub::GripperActionStub(const std::string & name, const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  logger_ = node_->get_logger().get_child("GripperAction");
  command_pub_ = node_->create_publisher<std_msgs::msg::String>(
    "/concrete_block/gripper_command",
    10);
}

BT::NodeStatus GripperActionStub::tick()
{
  std::string command = "OPEN";
  std::string block_id;
  double payload_mass_kg = 800.0;
  double payload_length_m = 0.9;
  double payload_radius_m = 0.3;
  double grip_point_x = 0.0;
  double grip_point_y = 0.0;
  double grip_point_z = 0.0;

  getInput("command", command);
  getInput("block_id", block_id);
  getInput("payload_mass_kg", payload_mass_kg);
  getInput("payload_length_m", payload_length_m);
  getInput("payload_radius_m", payload_radius_m);
  getInput("grip_point_x", grip_point_x);
  getInput("grip_point_y", grip_point_y);
  getInput("grip_point_z", grip_point_z);

  std::ostringstream encoded;
  encoded << command;
  if (command == "CLOSE") {
    encoded << " block_id=" << block_id;
    encoded << " mass_kg=" << payload_mass_kg;
    encoded << " length_m=" << payload_length_m;
    encoded << " radius_m=" << payload_radius_m;
    encoded << " grip_point_x=" << grip_point_x;
    encoded << " grip_point_y=" << grip_point_y;
    encoded << " grip_point_z=" << grip_point_z;
  }

  std_msgs::msg::String msg;
  msg.data = encoded.str();
  command_pub_->publish(msg);

  RCLCPP_INFO(
    logger_,
    "Published gripper command | command=%s block_id=%s mass=%.1fkg length=%.3fm radius=%.3fm",
    command.c_str(),
    block_id.empty() ? "<unspecified>" : block_id.c_str(),
    payload_mass_kg,
    payload_length_m,
    payload_radius_m);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace concrete_block_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<concrete_block_behavior_tree::GripperActionStub>("GripperAction");
}
