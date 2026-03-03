#include "concrete_block_behavior_tree/plugins/action/compute_trajectory.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"

namespace concrete_block_behavior_tree
{

void ComputeTrajectoryService::on_tick()
{
  getInput("geometric_plan_id", request_->geometric_plan_id);
  getInput("cartesian_path", request_->direct_path);
  getInput("use_direct_path", request_->use_direct_path);
  getInput("method", request_->method);
  getInput("validate_dynamics", request_->validate_dynamics);

  double timeout_s = 10.0;
  getInput("timeout_s", timeout_s);
  request_->timeout_s = static_cast<float>(timeout_s);
}

BT::NodeStatus ComputeTrajectoryService::on_completion(std::shared_ptr<ResponseT> response)
{
  setOutput("trajectory_id", response->trajectory_id);
  setOutput("joint_trajectory", response->trajectory);
  setOutput("trajectory_ok", response->success);
  setOutput("trajectory_message", response->message);
  return response->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace concrete_block_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<concrete_block_behavior_tree::ComputeTrajectoryService>(
    "ComputeTrajectory");
}
