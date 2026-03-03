#pragma once

#include <memory>
#include <string>

#include "concrete_block_motion_planning/srv/execute_trajectory.hpp"
#include "nav2_behavior_tree/bt_service_node.hpp"

namespace concrete_block_behavior_tree
{

class ExecuteTrajectoryService
  : public nav2_behavior_tree::BtServiceNode<concrete_block_motion_planning::srv::ExecuteTrajectory>
{
public:
  using ServiceT = concrete_block_motion_planning::srv::ExecuteTrajectory;
  using ResponseT = concrete_block_motion_planning::srv::ExecuteTrajectory_Response;

  ExecuteTrajectoryService(const std::string & service_name, const BT::NodeConfiguration & conf)
  : nav2_behavior_tree::BtServiceNode<ServiceT>(service_name, conf) {}

  void on_tick() override;
  BT::NodeStatus on_completion(std::shared_ptr<ResponseT> response) override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<std::string>("trajectory_id"),
        BT::InputPort<bool>("dry_run", true, "Execute as dry run"),
        BT::OutputPort<bool>("execution_ok"),
        BT::OutputPort<std::string>("execution_message")
      });
  }
};

}  // namespace concrete_block_behavior_tree
