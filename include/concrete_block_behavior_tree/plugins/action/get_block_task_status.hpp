#pragma once

#include <memory>
#include <string>

#include "concrete_block_world_model_interfaces/srv/get_coarse_blocks.hpp"
#include "nav2_behavior_tree/bt_service_node.hpp"

namespace concrete_block_behavior_tree
{

// Read one block's task_status (concrete_block_world_model_interfaces/Block) out of the world
// model. The counterpart of SetBlockTaskStatus, and the only thing the tree needs to tell a
// fresh task from one the plan server served again after a cancel (issue 268): that one comes
// back with its block still TASK_MOVE.
class GetBlockTaskStatus
  : public nav2_behavior_tree::BtServiceNode<
    concrete_block_world_model_interfaces::srv::GetCoarseBlocks>
{
public:
  using ServiceT = concrete_block_world_model_interfaces::srv::GetCoarseBlocks;
  using ResponseT = ServiceT::Response;

  GetBlockTaskStatus(
    const std::string & service_node_name,
    const BT::NodeConfig & conf);

  void on_tick() override;
  BT::NodeStatus on_completion(std::shared_ptr<ResponseT> response) override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<std::string>("block_id", "Block ID to look up"),
        BT::OutputPort<int>("task_status", "Block task_status (TASK_MOVE = 2, TASK_PLACED = 3)"),
      });
  }

private:
  std::string block_id_;
};

}  // namespace concrete_block_behavior_tree
