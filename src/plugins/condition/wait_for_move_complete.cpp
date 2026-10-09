#include "concrete_block_behavior_tree/plugins/condition/wait_for_move_complete.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"

namespace concrete_block_behavior_tree
{
namespace
{

std::vector<std::string> split(const std::string & csv)
{
  std::vector<std::string> out;
  std::stringstream stream(csv);
  std::string item;
  while (std::getline(stream, item, ',')) {
    const auto begin = item.find_first_not_of(" \t");
    if (begin != std::string::npos) {
      out.push_back(item.substr(begin, item.find_last_not_of(" \t") - begin + 1));
    }
  }
  return out;
}

// Position of `name` in `names`, or names.size() when it is not there.
size_t index_of(const std::vector<std::string> & names, const std::string & name)
{
  const auto it = std::find(names.begin(), names.end(), name);
  return static_cast<size_t>(std::distance(names.begin(), it));
}

}  // namespace

WaitForMoveComplete::WaitForMoveComplete(
  const std::string & name,
  const BT::NodeConfig & conf)
: BT::StatefulActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  // Launch-bound, not tree-bound: true only on controller:=cbs_mpc_active, where
  // the a2b spawner remaps the JTC's ~/joint_trajectory onto /crane/mpc/horizon.
  // node_ outlives the tree and every reload reconstructs this plugin, so
  // declaring unconditionally throws ParameterAlreadyDeclared on the second load.
  mpc_drives_ = node_->has_parameter("a2b_mpc_drives")
    ? node_->get_parameter("a2b_mpc_drives").as_bool()
    : node_->declare_parameter<bool>("a2b_mpc_drives", false);
  // Tree structure, not state: which half of the move this instance is.
  getInput("settle_only", settle_only_);
  // Logged because a silent failure to read it is the two-commander state this
  // node exists to prevent, and it reads as an ordinary move until the JTC log
  // is grepped for goal tolerances.
  RCLCPP_INFO(
    node_->get_logger(), "WaitForMoveComplete: a2b_mpc_drives=%s, so the commander is %s; "
    "this instance is %s",
    mpc_drives_ ? "true" : "false", mpc_drives_ ? "crane_mpc" : "the action goal",
    settle_only_ ? "the settle wait after it" : "the arrival oracle");

  std::string joint_states_topic;
  std::string settled_topic;
  std::string horizon_topic;
  getInput("joint_states_topic", joint_states_topic);
  getInput("settled_topic", settled_topic);
  getInput("horizon_topic", horizon_topic);

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(
    callback_group_,
    node_->get_node_base_interface());

  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  joint_state_sub_ = node_->create_subscription<JointState>(
    joint_states_topic, rclcpp::QoS(10),
    [this](JointState::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      joint_state_ = *msg;
      // Same shape as the sway dwell below: any sample over the bound restarts it.
      if (!moving_joints_text(*msg).empty()) {
        dq_a_still_since_.reset();
      } else if (!dq_a_still_since_) {
        dq_a_still_since_ = std::chrono::steady_clock::now();
      }
    },
    options);
  // crane_mpc publishes the verdict RELIABLE/VOLATILE, depth 1.
  settled_sub_ = node_->create_subscription<SwaySettled>(
    settled_topic, rclcpp::QoS(10),
    [this](SwaySettled::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      settled_ = *msg;
      settled_received_ = std::chrono::steady_clock::now();
      // Any verdict that is not YES restarts the hold.
      if (!still(*msg)) {
        settled_since_.reset();
      } else if (!settled_since_) {
        settled_since_ = settled_received_;
      }
    },
    options);
  // Only where the MPC is the commander: this is the one instance whose gates the
  // MPC's silence freezes. The settle wait runs off that arm, after the goal.
  if (mpc_drives_ && !settle_only_) {
    horizon_sub_ = node_->create_subscription<JointTrajectory>(
      horizon_topic, rclcpp::QoS(1),
      [this](JointTrajectory::SharedPtr) {
        std::lock_guard<std::mutex> lock(mutex_);
        horizon_received_ = std::chrono::steady_clock::now();
      },
      options);
  }
}

BT::NodeStatus WaitForMoveComplete::onStart()
{
  setOutput("mpc_drives", mpc_drives_);
  if (!mpc_drives_ && !settle_only_) {
    // The action goal is still the commander and still the arrival test.
    return BT::NodeStatus::SKIPPED;
  }

  if (!getInput("trajectory", trajectory_) || trajectory_.points.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "WaitForMoveComplete got no trajectory to arrive at");
    return BT::NodeStatus::FAILURE;
  }
  // A narrower last point would leave the joints past its width untested, and
  // this node is the only arrival test left.
  if (trajectory_.points.back().positions.size() != trajectory_.joint_names.size()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "WaitForMoveComplete got %zu joint names but %zu goal positions",
      trajectory_.joint_names.size(), trajectory_.points.back().positions.size());
    return BT::NodeStatus::FAILURE;
  }
  getInput("goal_tolerance", goal_tolerance_);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    sway_bound_ = 0.0;
    getInput("sway_bound", sway_bound_);
    getInput("dq_a_bound", dq_a_bound_);
  }
  getInput("timeout_ms", timeout_ms_);

  std::string joints;
  getInput("joints", joints);
  joints_ = split(joints);
  if (joints_.empty()) {
    joints_ = trajectory_.joint_names;
  }
  for (const auto & joint : joints_) {
    if (index_of(trajectory_.joint_names, joint) == trajectory_.joint_names.size()) {
      RCLCPP_ERROR(
        node_->get_logger(), "WaitForMoveComplete asked for joint %s, which the plan does not "
        "carry", joint.c_str());
      return BT::NodeStatus::FAILURE;
    }
  }

  {
    // A verdict latched before the move began says nothing about this move.
    std::lock_guard<std::mutex> lock(mutex_);
    settled_.reset();
    settled_since_.reset();
    dq_a_still_since_.reset();
    joint_state_.reset();
  }
  start_time_ = std::chrono::steady_clock::now();
  horizon_received_ = start_time_;
  // settle_only runs after the goal returned, so the plan has already been
  // played; waiting its duration a second time would double every move.
  plan_end_ = settle_only_
    ? node_->now()
    : node_->now() + rclcpp::Duration(trajectory_.points.back().time_from_start);

  RCLCPP_INFO(
    node_->get_logger(),
    "Waiting for the move to %s | joints=%zu tolerance=%.3f plan=%.1fs timeout_ms=%d",
    settle_only_ ? "settle" : "arrive", joints_.size(), goal_tolerance_,
    rclcpp::Duration(trajectory_.points.back().time_from_start).seconds(), timeout_ms_);

  return onRunning();
}

BT::NodeStatus WaitForMoveComplete::onRunning()
{
  callback_group_executor_.spin_some();

  const std::string reason = not_arrived_reason();
  if (reason.empty()) {
    std::lock_guard<std::mutex> lock(mutex_);
    RCLCPP_INFO(
      node_->get_logger(), "Move arrived and the load is settled | %s",
      joint_errors_text().c_str());
    return BT::NodeStatus::SUCCESS;
  }

  const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - start_time_).count();
  std::chrono::milliseconds horizon_quiet;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    horizon_quiet = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - horizon_received_);
  }
  // The commander stopping is the end of this move, not a reason to keep waiting:
  // past `max_reference_age` the JTC has run the last horizon out and holds its last
  // point, so the arrival error freezes there. The gates are read first and the bound
  // outlasts both the horizon and the dwell, so a move that can still end on its own
  // has already succeeded by here. Reported with the reason it had not, because that
  // pair is what says which half the move was short of when the driving stopped.
  if (horizon_sub_ && horizon_quiet > kHorizonMaxAge && elapsed_ms > kProducerGrace.count()) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Nothing has arrived on %s for %d ms, so crane_mpc has stopped driving this move and "
      "the JTC is holding its last horizon point: %s, and nothing can change that now. Its "
      "own log says which gate it fell silent on -- max_reference_age past the end of the "
      "plan on /crane/reference, most often",
      horizon_sub_->get_topic_name(), static_cast<int>(horizon_quiet.count()), reason.c_str());
    return BT::NodeStatus::FAILURE;
  }
  // No publisher means the question cannot be answered, and no amount of waiting
  // makes one: spending the whole timeout in front of a motionless machine, with
  // the controller active and the operator holding the deadman, is the worst way
  // to learn that. Not gated on having heard nothing yet (issue 241): a crane_mpc
  // that dies *mid-move* is the same dead end, and the settle wait is where it
  // bites -- 60 s of it. kProducerGrace covers the start, where "not discovered
  // yet" and "never will be" read the same.
  if (elapsed_ms > kProducerGrace.count() && settled_sub_->get_publisher_count() == 0) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Nothing publishes %s, so whether the load is still is unknowable: start crane_mpc "
      "(shadow is enough) or do not run this tree",
      settled_sub_->get_topic_name());
    return BT::NodeStatus::FAILURE;
  }
  if (elapsed_ms > timeout_ms_) {
    RCLCPP_WARN(node_->get_logger(), "Move did not complete: %s", reason.c_str());
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void WaitForMoveComplete::onHalted()
{
  std::lock_guard<std::mutex> lock(mutex_);
  settled_.reset();
  settled_since_.reset();
  dq_a_still_since_.reset();
  joint_state_.reset();
}

std::string WaitForMoveComplete::not_arrived_reason() const
{
  const auto now = std::chrono::steady_clock::now();

  // The plan has to have had time to run before "where the crane is" says
  // anything about where it went.
  const rclcpp::Time clock_now = node_->now();
  if (clock_now < plan_end_) {
    return "the plan still has " + std::to_string((plan_end_ - clock_now).seconds()) + " s to run";
  }

  std::lock_guard<std::mutex> lock(mutex_);

  if (!joint_state_) {
    return "no joint state received";
  }
  bool missed = false;
  for (const auto & joint : joints_) {
    // By name on both sides: the two stacks publish different /joint_states name
    // sets in different orders, so the i-th state is not the i-th planned joint.
    const size_t j = index_of(joint_state_->name, joint);
    if (j == joint_state_->name.size()) {
      return "joint " + joint + " is not in /joint_states";
    }
    if (j >= joint_state_->position.size()) {
      return "joint " + joint + " carries no position";
    }
    // Negated <=, not >: a NaN reading must not read as arrived.
    missed = missed || !(joint_error(joint) <= goal_tolerance_);
  }
  // Every joint, not the first that misses: which axes stand off and by how much
  // is the whole diagnosis, and one name per failed move is what left issue 241
  // four samples from four runs to size a tolerance on.
  if (missed) {
    return "joints off their goal, tolerance " + std::to_string(goal_tolerance_) + ": " +
           joint_errors_text();
  }

  // Inside the tolerance is not stopped (issue 249): a quiet plan reaches its last
  // point with the joints still running, and whatever runs next starts from zero
  // velocity and brakes the machine into a swing nothing is left to damp.
  const std::string moving = moving_joints_text(*joint_state_);
  if (!moving.empty()) {
    return "actuated joints still moving, bound " + std::to_string(dq_a_bound_) + ": " + moving;
  }
  if (!dq_a_still_since_ || now - *dq_a_still_since_ < kSettledMinHold) {
    return "the actuated joints have not held still for long enough to have stopped";
  }

  if (!settled_ || now - settled_received_ > kSettledMaxAge) {
    return "no fresh sway verdict";
  }
  if (!still(*settled_)) {
    return "the load is not settled: " + settled_->message;
  }
  if (!settled_since_ || now - *settled_since_ < kSettledMinHold) {
    // A turning point of a full-amplitude swing reads SETTLED_YES for an
    // instant, because the verdict is on the rate.
    return "the load has not read settled for long enough to be still";
  }
  return {};
}

double WaitForMoveComplete::joint_error(const std::string & joint) const
{
  const size_t i = index_of(trajectory_.joint_names, joint);
  const size_t j = index_of(joint_state_->name, joint);
  // NaN, not an out-of-range read: this is also called on the arrival path, one
  // lock later than the caller that checked, so a /joint_states message that
  // changed its name set in between must not index past the array. NaN fails
  // the `<=` below as a miss, which is the safe reading of an unresolvable axis.
  if (j >= joint_state_->position.size()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::abs(joint_state_->position[j] - trajectory_.points.back().positions[i]);
}

std::string WaitForMoveComplete::joint_errors_text() const
{
  if (!joint_state_) {
    return "no joint state";
  }
  std::string text;
  for (const auto & joint : joints_) {
    text += (text.empty() ? "" : ", ") + joint + " " + std::to_string(joint_error(joint));
  }
  return text;
}

std::string WaitForMoveComplete::moving_joints_text(const JointState & state) const
{
  std::string text;
  for (const auto & joint : joints_) {
    // By name, same as the positions: the i-th state is not the i-th planned joint.
    const size_t j = index_of(state.name, joint);
    if (j >= state.velocity.size()) {
      text += (text.empty() ? "" : ", ") + joint + " no rate";
      continue;
    }
    // Negated <=: a NaN rate must not read as stopped.
    if (!(std::abs(state.velocity[j]) <= dq_a_bound_)) {
      text += (text.empty() ? "" : ", ") + joint + " " + std::to_string(state.velocity[j]);
    }
  }
  return text;
}

bool WaitForMoveComplete::still(const SwaySettled & verdict) const
{
  if (sway_bound_ <= 0.0 || verdict.settled == SwaySettled::SETTLED_UNKNOWN) {
    return verdict.settled == SwaySettled::SETTLED_YES;
  }
  for (const double rate : verdict.velocity) {
    // Negated <=: a NaN rate must not read as still.
    if (!(std::abs(rate) <= sway_bound_)) {
      return false;
    }
  }
  return true;
}

}  // namespace concrete_block_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<concrete_block_behavior_tree::WaitForMoveComplete>(
    "WaitForMoveComplete");
}
