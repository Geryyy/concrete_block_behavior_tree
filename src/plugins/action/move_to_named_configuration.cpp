#include "concrete_block_behavior_tree/plugins/action/move_to_named_configuration.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

namespace concrete_block_behavior_tree
{

void MoveToNamedConfigurationService::on_tick()
{
  getInput("configuration_name", request_->configuration_name);
  getInput("dry_run", request_->dry_run);

  double timeout_s = 20.0;
  getInput("timeout_s", timeout_s);
  request_->timeout_s = static_cast<float>(timeout_s);
}

BT::NodeStatus MoveToNamedConfigurationService::on_completion(std::shared_ptr<ResponseT> response)
{
  setOutput("trajectory_id", response->trajectory_id);
  setOutput("move_ok", response->success);
  setOutput("move_message", response->message);
  if (!response->success) {
    RCLCPP_WARN(
      node_->get_logger(),
      "MoveToNamedConfiguration failed for '%s': %s",
      request_->configuration_name.c_str(),
      response->message.c_str());
  }
  return response->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace concrete_block_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<concrete_block_behavior_tree::MoveToNamedConfigurationService>(
    "MoveToNamedConfiguration");
}
