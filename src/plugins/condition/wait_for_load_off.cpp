#include "concrete_block_behavior_tree/plugins/condition/wait_for_load_off.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/point.hpp"
#include "tf2/exceptions.h"

namespace concrete_block_behavior_tree
{

namespace
{
// Block hanging still at the start of a 5 s descent; the dip has to last this long.
constexpr std::chrono::milliseconds kReference{500};
constexpr std::chrono::milliseconds kHold{100};
// A frozen transform freezes the floor while the JTC keeps descending.
constexpr double kMaxTfAge = 0.2;
constexpr double kGravity = 9.81;
}  // namespace

WaitForLoadOff::WaitForLoadOff(const std::string & name, const BT::NodeConfig & conf)
: BT::StatefulActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  getInput("joint", joint_);
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(
    callback_group_, node_->get_node_base_interface());
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  // Keep-last 1: spin_some() per ~28 Hz tick against 100 Hz kept a deep queue full,
  // so the gate read effort 1 s old (issue 258).
  sub_ = node_->create_subscription<JointState>(
    "/joint_states", rclcpp::QoS(1),
    std::bind(&WaitForLoadOff::callback, this, std::placeholders::_1), options);
  // data = [reference - effort, drop needed, centre above target [m]]
  pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/crane/load_off", 10);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void WaitForLoadOff::callback(const JointState::SharedPtr msg)
{
  // By name: the two stacks publish different name sets in different orders.
  const auto it = std::find(msg->name.begin(), msg->name.end(), joint_);
  if (it == msg->name.end()) {
    return;
  }
  const auto index = static_cast<size_t>(std::distance(msg->name.begin(), it));
  if (index >= msg->effort.size() || !std::isfinite(msg->effort[index])) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  effort_ = msg->effort[index];
  fresh_ = true;
  if (!referenced_) {
    window_.push_back(effort_);
  }
}

BT::NodeStatus WaitForLoadOff::onStart()
{
  callback_group_executor_.spin_some();
  std::lock_guard<std::mutex> lock(mutex_);
  window_.clear();
  referenced_ = false;
  dropped_ = false;
  fresh_ = false;
  lowest_ = std::numeric_limits<double>::infinity();
  start_ = std::chrono::steady_clock::now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForLoadOff::onRunning()
{
  callback_group_executor_.spin_some();
  const auto now = std::chrono::steady_clock::now();
  const auto log = node_->get_logger();

  double mass = 0.0, target_z = 0.0, drop_fraction = 0.5, max_above = 0.05, floor = 0.01;
  int timeout_ms = 20000;
  geometry_msgs::msg::Point centre_k8;
  std::string lever_frame, k8_frame, world_frame;
  if (!getInput("mass", mass) || !getInput("target_z", target_z) ||
    !getInput("s_log_8_x", centre_k8.x) || !getInput("s_log_8_y", centre_k8.y) ||
    !getInput("s_log_8_z", centre_k8.z))
  {
    RCLCPP_ERROR(log, "WaitForLoadOff: mass, target_z and s_log_8_* are required");
    return BT::NodeStatus::FAILURE;
  }
  // NaN passes every comparison below as "landed": the trees seed s_log_8_z with nan.
  if (!(mass > 0.0) || !std::isfinite(mass) || !std::isfinite(target_z) ||
    !std::isfinite(centre_k8.x) || !std::isfinite(centre_k8.y) || !std::isfinite(centre_k8.z))
  {
    RCLCPP_ERROR(
      log, "WaitForLoadOff: mass %.1f, target_z %.3f, s_log_8 (%.3f, %.3f, %.3f) not usable; "
      "stopping, the jaws stay shut", mass, target_z, centre_k8.x, centre_k8.y, centre_k8.z);
    return BT::NodeStatus::FAILURE;
  }
  getInput("lever_frame", lever_frame);
  getInput("k8_frame", k8_frame);
  getInput("frame", world_frame);
  getInput("drop_fraction", drop_fraction);
  getInput("max_above_target", max_above);
  getInput("floor_below_target", floor);
  getInput("timeout_ms", timeout_ms);

  if (now - start_ > std::chrono::milliseconds(timeout_ms)) {
    RCLCPP_WARN(log, "WaitForLoadOff: no landing within %d ms, the jaws stay shut", timeout_ms);
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::Point centre, lever;
  try {
    const auto k8 = tf_buffer_->lookupTransform(world_frame, k8_frame, tf2::TimePointZero);
    const auto pivot = tf_buffer_->lookupTransform(world_frame, lever_frame, tf2::TimePointZero);
    const double age =
      (node_->get_clock()->now() - rclcpp::Time(k8.header.stamp)).seconds();
    if (age > kMaxTfAge) {
      RCLCPP_ERROR(
        log, "WaitForLoadOff: %s is %.2f s old, the block height is unknown; stopping, "
        "the jaws stay shut", k8_frame.c_str(), age);
      return BT::NodeStatus::FAILURE;
    }
    // Hanging: the block sits |s_log_8| straight below K8. Not rotated through K8's
    // attitude -- the tool tilts as the block lands, and a 1 m lever then moved the
    // estimate 0.4 m in a second.
    centre.x = k8.transform.translation.x;
    centre.y = k8.transform.translation.y;
    centre.z = k8.transform.translation.z - std::abs(centre_k8.z);
    lever.x = pivot.transform.translation.x;
    lever.y = pivot.transform.translation.y;
  } catch (const tf2::TransformException & failure) {
    RCLCPP_WARN_THROTTLE(log, *node_->get_clock(), 1000, "WaitForLoadOff: %s", failure.what());
    return BT::NodeStatus::RUNNING;
  }
  const double above = centre.z - target_z;
  // Gated on the lowest point reached, not on now: the boom springs up as the load
  // leaves it, 0.2 m within a second of touchdown.
  lowest_ = std::min(lowest_, above);
  if (above < -floor) {
    RCLCPP_ERROR(
      log, "WaitForLoadOff: block centre %.3f m under its target without the load coming off, "
      "past the %.3f m floor; stopping, the jaws stay shut", -above, floor);
    return BT::NodeStatus::FAILURE;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (!referenced_) {
    if (now - start_ < kReference || window_.empty()) {
      return BT::NodeStatus::RUNNING;
    }
    std::nth_element(window_.begin(), window_.begin() + window_.size() / 2, window_.end());
    reference_ = window_[window_.size() / 2];
    referenced_ = true;
  }
  if (!fresh_ || !std::isfinite(effort_)) {
    return BT::NodeStatus::RUNNING;
  }
  fresh_ = false;
  const double r = std::hypot(centre.x - lever.x, centre.y - lever.y);
  const double threshold = drop_fraction * mass * kGravity * r;
  std_msgs::msg::Float64MultiArray plot;
  plot.data = {reference_ - effort_, threshold, above};
  pub_->publish(plot);
  RCLCPP_INFO_THROTTLE(
    log, *node_->get_clock(), 1000,
    "WaitForLoadOff: effort %.0f, reference %.0f, drop needed %.0f (r %.2f m), centre %+.3f m "
    "from target", effort_, reference_, threshold, r, above);
  if (reference_ - effort_ < threshold) {
    dropped_ = false;
    return BT::NodeStatus::RUNNING;
  }
  if (!dropped_) {
    dropped_ = true;
    dropped_since_ = now;
  }
  if (now - dropped_since_ < kHold) {
    return BT::NodeStatus::RUNNING;
  }
  if (lowest_ > max_above) {
    // Resting on something higher than the plan: stop pushing, do not let go.
    RCLCPP_ERROR(
      log, "WaitForLoadOff: the load came off with the block centre %.3f m over its "
      "target; stopping, the jaws stay shut", lowest_);
    return BT::NodeStatus::FAILURE;
  }
  RCLCPP_INFO(
    log, "WaitForLoadOff: landed | effort %.0f -> %.0f against %.0f (m g r, r %.2f m), "
    "centre %+.3f m from target", reference_, effort_, threshold, r, above);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace concrete_block_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<concrete_block_behavior_tree::WaitForLoadOff>("WaitForLoadOff");
}
