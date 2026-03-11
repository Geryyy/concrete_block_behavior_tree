#include "concrete_block_behavior_tree/plugins/action/plan_and_compute_trajectory.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"

namespace concrete_block_behavior_tree
{

void PlanAndComputeTrajectoryService::on_tick()
{
  getInput("start_pose", request_->start_pose);
  getInput("goal_pose", request_->goal_pose);
  getInput("target_block_id", request_->target_block_id);
  getInput("reference_block_id", request_->reference_block_id);
  getInput("use_world_model", request_->use_world_model);
  getInput("geometric_method", request_->geometric_method);
  getInput("trajectory_method", request_->trajectory_method);
  getInput("validate_dynamics", request_->validate_dynamics);

  double geometric_timeout_s = 5.0;
  getInput("geometric_timeout_s", geometric_timeout_s);
  request_->geometric_timeout_s = static_cast<float>(geometric_timeout_s);

  double trajectory_timeout_s = 10.0;
  getInput("trajectory_timeout_s", trajectory_timeout_s);
  request_->trajectory_timeout_s = static_cast<float>(trajectory_timeout_s);
}

BT::NodeStatus PlanAndComputeTrajectoryService::on_completion(std::shared_ptr<ResponseT> response)
{
  setOutput("geometric_plan_id", response->geometric_plan_id);
  setOutput("cartesian_path", response->cartesian_path);
  setOutput("trajectory_id", response->trajectory_id);
  setOutput("joint_trajectory", response->trajectory);
  setOutput("trajectory_ok", response->success);
  setOutput("trajectory_message", response->message);
  return response->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace concrete_block_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<concrete_block_behavior_tree::PlanAndComputeTrajectoryService>(
    "PlanAndComputeTrajectory");
}
