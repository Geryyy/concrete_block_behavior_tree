#include "concrete_block_behavior_tree/plugins/action/gripper_action.hpp"

#include <cstdio>

#include "behaviortree_cpp_v3/bt_factory.h"

namespace concrete_block_behavior_tree
{

BT::NodeStatus GripperActionStub::tick()
{
  std::string command = "OPEN";
  getInput("command", command);

  // Smoke-only stub: log and succeed until a real PZS100 controller interface is wired.
  std::printf("[GripperAction] command=%s (smoke-only stub)\n", command.c_str());

  return BT::NodeStatus::SUCCESS;
}

}  // namespace concrete_block_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<concrete_block_behavior_tree::GripperActionStub>("GripperAction");
}
