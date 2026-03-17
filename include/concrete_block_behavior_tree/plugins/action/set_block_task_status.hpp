#pragma once

#include <memory>
#include <string>

#include "concrete_block_perception/srv/set_block_task_status.hpp"
#include "nav2_behavior_tree/bt_service_node.hpp"

namespace concrete_block_behavior_tree
{

class SetBlockTaskStatusService
  : public nav2_behavior_tree::BtServiceNode<concrete_block_perception::srv::SetBlockTaskStatus>
{
public:
  using ServiceT = concrete_block_perception::srv::SetBlockTaskStatus;
  using ResponseT = concrete_block_perception::srv::SetBlockTaskStatus_Response;

  SetBlockTaskStatusService(const std::string & service_name, const BT::NodeConfiguration & conf)
  : nav2_behavior_tree::BtServiceNode<ServiceT>(service_name, conf) {}

  void on_tick() override;
  BT::NodeStatus on_completion(std::shared_ptr<ResponseT> response) override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<std::string>("block_id", "", "Block ID to update"),
        BT::InputPort<int>("task_status", 3, "Task status int (TASK_PLACED=3)"),
        BT::OutputPort<bool>("status_ok"),
        BT::OutputPort<std::string>("status_message")
      });
  }
};

}  // namespace concrete_block_behavior_tree
