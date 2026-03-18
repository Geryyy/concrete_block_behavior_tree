#pragma once

#include <string>

#include "behaviortree_cpp_v3/action_node.h"

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
  GripperActionStub(const std::string & name, const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(name, conf) {}

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("command", "OPEN", "Gripper command: OPEN or CLOSE")
    };
  }
};

}  // namespace concrete_block_behavior_tree
