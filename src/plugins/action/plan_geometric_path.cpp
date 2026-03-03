#include "concrete_block_behavior_tree/plugins/action/plan_geometric_path.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"

namespace concrete_block_behavior_tree
{

void PlanGeometricPathService::on_tick()
{
  getInput("start_pose", request_->start_pose);
  getInput("goal_pose", request_->goal_pose);
  getInput("target_block_id", request_->target_block_id);
  getInput("reference_block_id", request_->reference_block_id);
  getInput("method", request_->method);
  getInput("use_world_model", request_->use_world_model);

  double timeout_s = 5.0;
  getInput("timeout_s", timeout_s);
  request_->timeout_s = static_cast<float>(timeout_s);
}

BT::NodeStatus PlanGeometricPathService::on_completion(std::shared_ptr<ResponseT> response)
{
  setOutput("geometric_plan_id", response->geometric_plan_id);
  setOutput("cartesian_path", response->cartesian_path);
  setOutput("geometric_ok", response->success);
  setOutput("geometric_message", response->message);
  return response->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace concrete_block_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<concrete_block_behavior_tree::PlanGeometricPathService>(
    "PlanGeometricPath");
}
