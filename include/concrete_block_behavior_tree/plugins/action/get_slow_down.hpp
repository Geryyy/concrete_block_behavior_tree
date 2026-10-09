#pragma once

#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"

namespace concrete_block_behavior_tree
{

// Put the run's global `slow_down` on the blackboard, read from the BT server's own
// `slow_down` parameter so a lab day picks its pace with a launch argument instead of an
// edited tree. The subtrees that want a slower leg multiply this value; they do not
// overwrite it, so one number paces the whole run.
class GetSlowDown : public BT::SyncActionNode
{
public:
  GetSlowDown(const std::string & name, const BT::NodeConfig & conf);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<double>("slow_down", "Time stretch every planned move is paced by"),
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace concrete_block_behavior_tree
