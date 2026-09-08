#include "concrete_block_behavior_tree/plugins/condition/plan_complete.hpp"

#include "behaviortree_cpp/bt_factory.h"

namespace concrete_block_behavior_tree
{

PlanComplete::PlanComplete(const std::string & name, const BT::NodeConfig & conf)
: BT::ConditionNode(name, conf)
{
}

BT::NodeStatus PlanComplete::tick()
{
  bool plan_has_task = true;
  if (!getInput("plan_has_task", plan_has_task)) {
    // No flag on the blackboard yet: cannot claim the plan is complete.
    return BT::NodeStatus::FAILURE;
  }
  return plan_has_task ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
}

}  // namespace concrete_block_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<concrete_block_behavior_tree::PlanComplete>("PlanComplete");
}
