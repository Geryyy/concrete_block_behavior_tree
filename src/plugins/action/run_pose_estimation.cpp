#include "concrete_block_behavior_tree/plugins/action/run_pose_estimation.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"

namespace concrete_block_behavior_tree
{

void RunPoseEstimationService::on_tick()
{
  getInput("mode", request_->mode);
  getInput("target_block_id", request_->target_block_id);
  getInput("enable_debug", request_->enable_debug);

  double timeout_s = 8.0;
  getInput("timeout_s", timeout_s);
  request_->timeout_s = static_cast<float>(timeout_s);
}

BT::NodeStatus RunPoseEstimationService::on_completion(std::shared_ptr<ResponseT> response)
{
  setOutput("perception_ok", response->success);
  setOutput("perception_message", response->message);
  return response->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace concrete_block_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<concrete_block_behavior_tree::RunPoseEstimationService>(
    "RunPoseEstimation");
}
