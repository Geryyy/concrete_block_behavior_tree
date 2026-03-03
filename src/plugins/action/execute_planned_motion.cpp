#include "concrete_block_behavior_tree/plugins/action/execute_planned_motion.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"

namespace concrete_block_behavior_tree
{

void ExecutePlannedMotionService::on_tick()
{
  getInput("plan_id", request_->plan_id);
  getInput("dry_run", request_->dry_run);
}

BT::NodeStatus ExecutePlannedMotionService::on_completion(std::shared_ptr<ResponseT> response)
{
  setOutput("execution_ok", response->success);
  setOutput("execution_message", response->message);
  return response->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace concrete_block_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<concrete_block_behavior_tree::ExecutePlannedMotionService>(
    "ExecutePlannedMotion");
}
