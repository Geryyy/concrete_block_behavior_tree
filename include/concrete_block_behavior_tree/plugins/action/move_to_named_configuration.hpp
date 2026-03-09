#pragma once

#include <memory>
#include <string>

#include "concrete_block_motion_planning/srv/execute_named_configuration.hpp"
#include "nav2_behavior_tree/bt_service_node.hpp"

namespace concrete_block_behavior_tree
{

class MoveToNamedConfigurationService
  : public nav2_behavior_tree::BtServiceNode<concrete_block_motion_planning::srv::ExecuteNamedConfiguration>
{
public:
  using ServiceT = concrete_block_motion_planning::srv::ExecuteNamedConfiguration;
  using ResponseT = concrete_block_motion_planning::srv::ExecuteNamedConfiguration_Response;

  MoveToNamedConfigurationService(const std::string & service_name, const BT::NodeConfiguration & conf)
  : nav2_behavior_tree::BtServiceNode<ServiceT>(service_name, conf) {}

  void on_tick() override;
  BT::NodeStatus on_completion(std::shared_ptr<ResponseT> response) override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<std::string>("configuration_name"),
        BT::InputPort<double>("timeout_s", 20.0, "Execution timeout in seconds"),
        BT::InputPort<bool>("dry_run", false, "Execute as dry run"),
        BT::OutputPort<std::string>("trajectory_id"),
        BT::OutputPort<bool>("move_ok"),
        BT::OutputPort<std::string>("move_message")
      });
  }
};

}  // namespace concrete_block_behavior_tree

