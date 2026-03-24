#include "concrete_block_behavior_tree/plugins/action/get_next_assembly_task.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

namespace concrete_block_behavior_tree
{

void GetNextAssemblyTaskService::on_tick()
{
  getInput("wall_plan_name", request_->wall_plan_name);
  getInput("reset_plan", request_->reset_plan);
  RCLCPP_INFO(
    node_->get_logger(),
    "Requesting next assembly task | wall_plan=%s reset_plan=%s",
    request_->wall_plan_name.c_str(),
    request_->reset_plan ? "true" : "false");
}

BT::NodeStatus GetNextAssemblyTaskService::on_completion(std::shared_ptr<ResponseT> response)
{
  setOutput("task_id", response->task_id);
  setOutput("target_block_id", response->target_block_id);
  setOutput("reference_block_id", response->reference_block_id);
  setOutput("target_block_pose_coarse", response->target_pose);
  setOutput("target_block_pose_precise", response->target_pose);
  setOutput("reference_block_pose_precise", response->reference_pose);
  setOutput("plan_has_task", response->has_task);
  setOutput("plan_message", response->message);

  RCLCPP_INFO(
    node_->get_logger(),
    "GetNextAssemblyTask response | success=%s has_task=%s task_id=%s target=%s reference=%s message=%s",
    response->success ? "true" : "false",
    response->has_task ? "true" : "false",
    response->task_id.c_str(),
    response->target_block_id.c_str(),
    response->reference_block_id.c_str(),
    response->message.c_str());

  if (!response->success || !response->has_task) {
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace concrete_block_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<concrete_block_behavior_tree::GetNextAssemblyTaskService>(
    "GetNextAssemblyTask");
}
