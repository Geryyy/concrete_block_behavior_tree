#include "concrete_block_behavior_tree/plugins/action/set_mpc_payload.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>

#include "behaviortree_cpp/bt_factory.h"

namespace concrete_block_behavior_tree
{

namespace
{
// Discovery is minutes old by the time a pick reaches this node, so this only
// separates "no MPC in this launch profile" from "the MPC is there".
constexpr std::chrono::milliseconds kDiscovery{200};
}  // namespace

SetMpcPayload::SetMpcPayload(const std::string & name, const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  getInput("service_name", service_name_);
  if (service_name_.empty()) {
    throw std::invalid_argument("SetMpcPayload input 'service_name' must not be empty");
  }

  mpc_drives_ = node_->has_parameter("a2b_mpc_drives")
    ? node_->get_parameter("a2b_mpc_drives").as_bool()
    : node_->declare_parameter<bool>("a2b_mpc_drives", false);

  // Our own callback group, kept out of the node's executor, so waiting for the
  // answer inside tick() cannot re-enter the executor that drives the tree.
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
  client_ = node_->create_client<SetPayload>(
    service_name_, rclcpp::SystemDefaultsQoS(), callback_group_);
}

double SetMpcPayload::sanitize(double value, const char * field_name)
{
  if (!std::isfinite(value)) {
    RCLCPP_WARN(
      node_->get_logger(),
      "SetMpcPayload: '%s' is not finite, substituting 0.0", field_name);
    return 0.0;
  }
  return value;
}

BT::NodeStatus SetMpcPayload::not_taken(const std::string & why)
{
  if (mpc_drives_) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "SetMpcPayload: %s. crane_mpc drives this move, so it would solve it against the "
      "payload it last accepted", why.c_str());
    return BT::NodeStatus::FAILURE;
  }
  RCLCPP_WARN(
    node_->get_logger(),
    "SetMpcPayload: %s. Nothing drives the crane from crane_mpc on this profile "
    "(a2b_mpc_drives=false), so the move stands", why.c_str());
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SetMpcPayload::tick()
{
  bool carries = false;
  double mass = 0.0;
  double s_log_8_x = 0.0;
  double s_log_8_y = 0.0;
  double s_log_8_z = 0.0;
  double size_x = 0.0;
  double size_y = 0.0;
  double size_z = 0.0;
  int timeout_ms = 2000;

  getInput("carries", carries);
  getInput("mass", mass);
  getInput("s_log_8_x", s_log_8_x);
  getInput("s_log_8_y", s_log_8_y);
  getInput("s_log_8_z", s_log_8_z);
  getInput("size_x", size_x);
  getInput("size_y", size_y);
  getInput("size_z", size_z);
  getInput("timeout_ms", timeout_ms);

  auto request = std::make_shared<SetPayload::Request>();
  if (carries) {
    if (!std::isfinite(mass) || mass <= 0.0) {
      // The hazard this node exists to close: a carrying move solved for an
      // empty hook. Refuse rather than send a payload crane_mpc would reject.
      RCLCPP_ERROR(
        node_->get_logger(),
        "SetMpcPayload: carries=true with mass %.3f kg -- the MPC would keep modelling "
        "an empty gripper", mass);
      return BT::NodeStatus::FAILURE;
    }
    // crane_mpc models a uniform box of these edges; SHAPE_NONE is how the message says
    // "empty hook".
    request->payload.shape = crane_msgs::msg::Payload::SHAPE_BOX;
    request->payload.dimensions.x = size_x;
    request->payload.dimensions.y = size_y;
    request->payload.dimensions.z = size_z;
    request->payload.mass = mass;
    request->payload.com.x = sanitize(s_log_8_x, "s_log_8_x");
    request->payload.com.y = sanitize(s_log_8_y, "s_log_8_y");
    request->payload.com.z = sanitize(s_log_8_z, "s_log_8_z");
  } else {
    request->payload.shape = crane_msgs::msg::Payload::SHAPE_NONE;
  }

  if (!client_->wait_for_service(kDiscovery)) {
    return not_taken("nothing answers " + service_name_);
  }

  auto pending = client_->async_send_request(request);
  const int64_t request_id = pending.request_id;
  auto future = pending.future.share();
  if (executor_.spin_until_future_complete(future, std::chrono::milliseconds(timeout_ms)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    client_->remove_pending_request(request_id);
    return not_taken(
      service_name_ + " did not answer within " + std::to_string(timeout_ms) + " ms");
  }

  const auto response = future.get();
  if (!response->success) {
    return not_taken(service_name_ + " refused the payload: " + response->message);
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "Set MPC payload | carries=%s mass=%.1fkg s_log_8=(%.3f, %.3f, %.3f) | %s",
    carries ? "true" : "false", request->payload.mass, request->payload.com.x,
    request->payload.com.y, request->payload.com.z, response->message.c_str());

  return BT::NodeStatus::SUCCESS;
}

}  // namespace concrete_block_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<concrete_block_behavior_tree::SetMpcPayload>("SetMpcPayload");
}
