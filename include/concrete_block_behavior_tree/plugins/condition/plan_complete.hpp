#pragma once

#include <string>

#include "behaviortree_cpp/condition_node.h"

namespace concrete_block_behavior_tree
{

// Returns SUCCESS when the assembly plan is finished (no more tasks left),
// FAILURE otherwise. Reads the `plan_has_task` flag that GetNextAssemblyTask
// writes to the blackboard before returning: has_task == false means the plan
// is complete.
//
// Used as the recovery branch of a Fallback wrapping the assembly loop so that
// the loop's terminal failure becomes an overall SUCCESS only when the wall is
// actually done. A genuine pick/place failure leaves plan_has_task == true (the
// last GetNextAssemblyTask had succeeded), so this node fails and the overall
// goal still reports failure.
class PlanComplete : public BT::ConditionNode
{
public:
  PlanComplete(const std::string & name, const BT::NodeConfig & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>(
        "plan_has_task",
        "Did the last GetNextAssemblyTask return a task? false => plan complete"),
    };
  }

  BT::NodeStatus tick() override;
};

}  // namespace concrete_block_behavior_tree
