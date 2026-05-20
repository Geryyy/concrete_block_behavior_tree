#pragma once

#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "gazebo_grasp_plugin_ros/msg/gazebo_grasp_event.hpp"
#include "rclcpp/rclcpp.hpp"

namespace concrete_block_behavior_tree
{

// One-shot BT action that publishes a GazeboGraspEvent on /grasp_command,
// forwarded by grasp_event_republisher to the GazeboGraspFix plugin so it
// performs a force-attach or force-detach. Bypasses the force-angle heuristic.
class PublishGraspCommand : public BT::SyncActionNode
{
public:
  PublishGraspCommand(const std::string & name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic", "/grasp_command", "Grasp command topic"),
      BT::InputPort<std::string>("arm", "gripper", "Gazebo grasp arm name"),
      BT::InputPort<std::string>("object", "", "Exact gazebo entity name to attach/detach"),
      BT::InputPort<bool>("attached", true, "true to attach, false to detach"),
    };
  }

  BT::NodeStatus tick() override;

private:
  using GraspEvent = gazebo_grasp_plugin_ros::msg::GazeboGraspEvent;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<GraspEvent>::SharedPtr publisher_;
  std::string topic_;
};

}  // namespace concrete_block_behavior_tree
