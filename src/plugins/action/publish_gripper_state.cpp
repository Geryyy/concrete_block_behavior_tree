#include "concrete_block_behavior_tree/plugins/action/publish_gripper_state.hpp"

#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>

#include "behaviortree_cpp/bt_factory.h"

namespace concrete_block_behavior_tree
{

namespace
{

std::string trim(const std::string & text)
{
  const auto first = text.find_first_not_of(" \t\n\r");
  if (first == std::string::npos) {
    return {};
  }
  const auto last = text.find_last_not_of(" \t\n\r");
  return text.substr(first, last - first + 1);
}

// Parse "radius_top: 0.30, radius_bottom: 0.30, length: 0.90". Same format the
// A2B planner already consumes via logCarryingText, so both models are fed from
// the single log_shape_text blackboard entry.
bool parse_log_shape(
  const std::string & text, double & radius_top, double & radius_bottom, double & length)
{
  bool has_radius_top = false;
  bool has_radius_bottom = false;
  bool has_length = false;

  std::stringstream stream(text);
  std::string token;
  while (std::getline(stream, token, ',')) {
    const auto colon = token.find(':');
    if (colon == std::string::npos) {
      continue;
    }
    const auto key = trim(token.substr(0, colon));
    double value = 0.0;
    try {
      value = std::stod(trim(token.substr(colon + 1)));
    } catch (const std::exception &) {
      continue;
    }
    if (key == "radius_top") {
      radius_top = value;
      has_radius_top = true;
    } else if (key == "radius_bottom") {
      radius_bottom = value;
      has_radius_bottom = true;
    } else if (key == "length") {
      length = value;
      has_length = true;
    }
  }

  return has_radius_top && has_radius_bottom && has_length;
}

}  // namespace

PublishGripperState::PublishGripperState(
  const std::string & name,
  const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  getInput("topic", topic_);
  if (topic_.empty()) {
    throw std::invalid_argument("PublishGripperState input 'topic' must not be empty");
  }

  // transient_local is compatible with the MPC's volatile subscription and
  // additionally latches for any transient_local reader (grip_estimator, the
  // upstream publisher of this topic, uses transient_local too).
  pub_ = node_->create_publisher<GripperState>(topic_, rclcpp::QoS(1).transient_local());

  RCLCPP_INFO(
    node_->get_logger(),
    "PublishGripperState initialized, publishing payload state to %s",
    topic_.c_str());
}

double PublishGripperState::sanitize(double value, const char * field_name)
{
  if (!std::isfinite(value)) {
    RCLCPP_WARN(
      node_->get_logger(),
      "PublishGripperState: '%s' is not finite, substituting 0.0", field_name);
    return 0.0;
  }
  return value;
}

BT::NodeStatus PublishGripperState::tick()
{
  bool carries = false;
  double mass = 0.0;
  double s_log_8_x = 0.0;
  double s_log_8_y = 0.0;
  double s_log_8_z = 0.0;
  std::string shape_text;

  getInput("carries", carries);
  getInput("mass", mass);
  getInput("log_shape_text", shape_text);
  getInput("s_log_8_x", s_log_8_x);
  getInput("s_log_8_y", s_log_8_y);
  getInput("s_log_8_z", s_log_8_z);

  double radius_top = 0.0;
  double radius_bottom = 0.0;
  double length = 0.0;
  const bool shape_ok = parse_log_shape(shape_text, radius_top, radius_bottom, length);

  if (carries) {
    if (!shape_ok) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "PublishGripperState: carries=true but log_shape_text could not be parsed: '%s'",
        shape_text.c_str());
    }
    if (mass <= 0.0) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "PublishGripperState: carries=true with non-positive mass %.3f kg -- the MPC will "
        "model an unloaded gripper", mass);
    }
  }

  GripperState msg;
  msg.header.stamp = node_->get_clock()->now();
  msg.carries_log = carries;
  msg.mass = sanitize(mass, "mass");
  msg.log.radius_top = static_cast<float>(sanitize(radius_top, "radius_top"));
  msg.log.radius_bottom = static_cast<float>(sanitize(radius_bottom, "radius_bottom"));
  msg.log.length = static_cast<float>(sanitize(length, "length"));
  msg.s_log_8.x = sanitize(s_log_8_x, "s_log_8_x");
  msg.s_log_8.y = sanitize(s_log_8_y, "s_log_8_y");
  msg.s_log_8.z = sanitize(s_log_8_z, "s_log_8_z");

  pub_->publish(msg);

  RCLCPP_INFO(
    node_->get_logger(),
    "Published gripper state | carries=%s mass=%.1fkg shape=(r=%.2f/%.2f, l=%.2f) "
    "s_log_8=(%.3f, %.3f, %.3f)",
    carries ? "true" : "false", msg.mass, msg.log.radius_top, msg.log.radius_bottom,
    msg.log.length, msg.s_log_8.x, msg.s_log_8.y, msg.s_log_8.z);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace concrete_block_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<concrete_block_behavior_tree::PublishGripperState>(
    "PublishGripperState");
}
