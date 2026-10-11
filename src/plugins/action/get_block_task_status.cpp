#include "concrete_block_behavior_tree/plugins/action/get_block_task_status.hpp"

#include <algorithm>
#include <stdexcept>

#include "behaviortree_cpp/bt_factory.h"
#include "concrete_block_world_model_interfaces/msg/block.hpp"

namespace concrete_block_behavior_tree
{

GetBlockTaskStatus::GetBlockTaskStatus(
  const std::string & service_node_name,
  const BT::NodeConfig & conf)
: nav2_behavior_tree::BtServiceNode<ServiceT>(service_node_name, conf)
{
}

void GetBlockTaskStatus::on_tick()
{
  getInput("block_id", block_id_);
  if (block_id_.empty()) {
    throw std::invalid_argument("GetBlockTaskStatus: 'block_id' input is empty");
  }

  request_->force_refresh = false;
  request_->timeout_s = 1.0f;
}

BT::NodeStatus GetBlockTaskStatus::on_completion(std::shared_ptr<ResponseT> response)
{
  using Block = concrete_block_world_model_interfaces::msg::Block;

  if (!response->success) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "GetBlockTaskStatus: GetCoarseBlocks failed: %s",
      response->message.c_str());
    return BT::NodeStatus::FAILURE;
  }

  const auto & blocks = response->blocks.blocks;
  const auto it = std::find_if(
    blocks.begin(), blocks.end(),
    [&](const auto & b) {return b.id == block_id_;});
  if (it == blocks.end()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "GetBlockTaskStatus: block_id='%s' not in world model (got %zu blocks)",
      block_id_.c_str(), blocks.size());
    return BT::NodeStatus::FAILURE;
  }

  setOutput("task_status", static_cast<int>(it->task_status));

  // The operator-facing half of issue 274: this is the one status that changes what the tree
  // does next, and the recovery is not something the tree can do for them.
  if (it->task_status == Block::TASK_MOVE) {
    RCLCPP_WARN(
      node_->get_logger(),
      "GetBlockTaskStatus: '%s' is still TASK_MOVE - a cancelled run left it mid-cycle, so the "
      "pick is skipped and the carry resumed. If the grasp signal then fails, check the hook "
      "rather than assume it is empty: the signal expires, so the block may still be clamped. "
      "Reset it to TASK_FREE (/world_model_node/set_block_task_status) only once you have "
      "found it.",
      block_id_.c_str());
  } else {
    RCLCPP_INFO(
      node_->get_logger(),
      "GetBlockTaskStatus: block_id='%s' task_status=%d",
      block_id_.c_str(), it->task_status);
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace concrete_block_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<concrete_block_behavior_tree::GetBlockTaskStatus>(
    "GetBlockTaskStatus");
}
