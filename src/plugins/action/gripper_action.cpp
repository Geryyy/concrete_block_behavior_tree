#include "concrete_block_behavior_tree/plugins/action/gripper_action.hpp"

#include <cstdio>

#include "behaviortree_cpp_v3/bt_factory.h"

namespace concrete_block_behavior_tree
{

BT::NodeStatus GripperActionStub::tick()
{
  std::string command = "OPEN";
  getInput("command", command);

  // Stub: log and succeed. Wire real PZS100 controller service/action here.
  std::printf("[GripperAction] command=%s (simulation stub)\n", command.c_str());

  return BT::NodeStatus::SUCCESS;
}

}  // namespace concrete_block_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<concrete_block_behavior_tree::GripperActionStub>("GripperAction");
}
