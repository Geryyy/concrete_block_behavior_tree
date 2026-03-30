#pragma once

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace concrete_block_behavior_tree
{

/**
 * @brief Smoke-only stub for gripper open/close commands.
 *
 * Returns SUCCESS immediately. Replace with a real service or action call
 * once the PZS100 gripper controller interface is confirmed.
 *
 * XML usage:
 *   <GripperAction command="CLOSE"/>
 *   <GripperAction command="OPEN"/>
 */
class GripperActionStub : public BT::SyncActionNode
{
public:
  GripperActionStub(const std::string & name, const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("command", "OPEN", "Gripper command: OPEN or CLOSE"),
      BT::InputPort<std::string>("block_id", "", "Attached block identifier"),
      BT::InputPort<double>("payload_mass_kg", 800.0, "Payload mass proxy"),
      BT::InputPort<double>("payload_length_m", 0.9, "Payload length proxy"),
      BT::InputPort<double>("payload_radius_m", 0.3, "Payload radius proxy"),
      BT::InputPort<double>("grip_point_x", 0.0, "Payload grip point x"),
      BT::InputPort<double>("grip_point_y", 0.0, "Payload grip point y"),
      BT::InputPort<double>("grip_point_z", 0.0, "Payload grip point z")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("GripperAction")};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
};

}  // namespace concrete_block_behavior_tree
