#include "concrete_block_behavior_tree/plugins/action/set_block_task_status.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"

namespace concrete_block_behavior_tree
{

void SetBlockTaskStatusService::on_tick()
{
  getInput("block_id", request_->block_id);

  int task_status = 3;  // TASK_PLACED
  getInput("task_status", task_status);
  request_->task_status = task_status;
}

BT::NodeStatus SetBlockTaskStatusService::on_completion(std::shared_ptr<ResponseT> response)
{
  setOutput("status_ok", response->success);
  setOutput("status_message", response->message);
  return response->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace concrete_block_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<concrete_block_behavior_tree::SetBlockTaskStatusService>(
    "SetBlockTaskStatus");
}
