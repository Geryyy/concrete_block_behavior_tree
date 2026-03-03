#include "concrete_block_behavior_tree/plugins/action/plan_block_motion.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"

namespace concrete_block_behavior_tree
{

void PlanBlockMotionService::on_tick()
{
  getInput("start_pose", request_->start_pose);
  getInput("goal_pose", request_->goal_pose);
  getInput("target_block_id", request_->target_block_id);
  getInput("reference_block_id", request_->reference_block_id);
  getInput("planner_method", request_->planner_method);
  getInput("use_world_model", request_->use_world_model);

  double timeout_s = 5.0;
  getInput("timeout_s", timeout_s);
  request_->timeout_s = static_cast<float>(timeout_s);
}

BT::NodeStatus PlanBlockMotionService::on_completion(std::shared_ptr<ResponseT> response)
{
  setOutput("plan_id", response->plan_id);
  setOutput("planned_path", response->cartesian_path);
  setOutput("planning_ok", response->success);
  setOutput("planning_message", response->message);
  return response->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace concrete_block_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<concrete_block_behavior_tree::PlanBlockMotionService>(
    "PlanBlockMotion");
}
