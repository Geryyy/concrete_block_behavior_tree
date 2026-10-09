#pragma once

#include <chrono>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "crane_msgs/msg/sway_settled.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace concrete_block_behavior_tree
{

// Arrival oracle for an A2B move nobody holds an action goal for (issue 148).
//
// With crane_mpc in active mode the JTC is re-goaled by topic every 40 ms, so the
// tree must not also hold a FollowJointTrajectory goal. The goal was the only
// arrival test in the loop -- goal_time plus constraints.<joint>.goal -- and a
// topic-driven trajectory carries none. This node is that test, plus the one the
// action never had: the load has to have stopped swinging before the move counts
// as over.
//
// Three things must hold at the same tick, and each replaces something the goal
// gave away:
//  - the plan's own duration has elapsed. `time_from_start` of the last point,
//    on the node's clock (sim time under Gazebo). Without it a plan whose
//    endpoint is already within tolerance of the start -- a placement
//    correction, a rotator trim, a re-plan that barely moves -- reads as
//    arrived on the second tick while the MPC is still executing it, and the
//    next node descends on a moving crane. The action path got this free: the
//    JTC had to actually run the trajectory.
//  - every joint in `joints` is within `goal_tolerance` of /joint_states,
//    matched BY NAME (the two stacks publish different name sets in different
//    orders, so an index match reads the wrong axis). Not every joint of the
//    trajectory: `{trajectory}` is the canonical eight and carries the passive
//    pendulum pair, whose planned values are "the sway the plan claims, not
//    where the tool would settle" and are read out through an uncalibrated
//    offset. The MPC does not track them either -- /crane/reference is the six
//    actuated columns -- and the sway half below is what covers them.
//  - /crane/sway_settled has read SETTLED_YES continuously for kSettledMinHold,
//    and the newest verdict is no older than kSettledMaxAge.
// SETTLED_UNKNOWN is not success: a bracketing IMU that stopped answering makes
// "settled" unknowable, and unknowable must not read as settled.
//
// Why the sway verdict needs a hold and not just a sample: the verdict is
// `all(|dq_u| <= bound)` on the passive *rate*, and a pendulum at maximum
// deflection has zero rate. A swinging load therefore publishes SETTLED_YES
// twice per period, at the turning points. One sample of it is a coin flip on
// exactly the load this node exists to refuse.
//
// The caches are cleared in onStart(), so nothing latched before the move can
// satisfy it; kSettledMaxAge then covers the producer that dies mid-move, where
// the JTC still runs its last trajectory to the end and would otherwise arrive
// under the last verdict of a dead estimator.
//
// AND THE COMMANDER HAS TO STILL BE COMMANDING (issue 239). On the MPC arm this
// node also watches `horizon_topic`, and fails the move when it goes quiet for
// kHorizonMaxAge. crane_mpc stops publishing `max_reference_age` (20 s) after the
// plan on /crane/reference ends, which on a 5-15 s plan is ~25-35 s into this
// node's window; the JTC runs that last horizon out and then holds *its own last
// point*, not the plan's end, so the arrival error freezes there and no later tick
// can change it. Without this the move spent the rest of `timeout_ms` -- measured
// 90 s -- and then reported a tolerance miss from 90 s earlier. So
// `timeout_ms` is not the bound on the MPC arm: the MPC's own hold is, and this
// is what makes the two one decision instead of two constants in two repos.
//
// SETTLE-ONLY, THE ACTION ARM (issue 220). On `pid` / `mpc` / `cbs_mpc` the
// FollowJointTrajectory goal is the commander and the arrival test, but it
// tests positions and a goal_time -- never the load. Nothing on that arm damps
// sway either, so the tree used to descend into a swinging block. A second
// instance of this node runs *after* the goal with `settle_only`, which drops
// the profile gate and the plan-duration wait (the goal already played the
// plan) and leaves the half the goal never had. It needs a live
// /crane/sway_settled, and crane_mpc is its only publisher: on bare
// `controller:=pid` nothing answers and this instance times out. See
// wiki/commissioning.md on which switch the lab-day fallback actually is.
class WaitForMoveComplete : public BT::StatefulActionNode
{
public:
  WaitForMoveComplete(const std::string & name, const BT::NodeConfig & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<trajectory_msgs::msg::JointTrajectory>(
        "trajectory", "Planned trajectory; its last point is the goal"),
      // Default is constraints.<joint>.goal of the a2b JTC
      // (config/ros2_control/crane_controller_hydraulic_a2b_jtc_pid_pzs100.ros2_control.yaml),
      // which is the controller the cbs_mpc* profiles run. One number and not a
      // per-joint map, because that file sets the same 0.05 for every joint it
      // actually constrains -- see `joints`.
      BT::InputPort<double>("goal_tolerance", 0.05, "Per-joint position tolerance [rad|m]"),
      // Which joints that tolerance is asked of. Empty means every joint the
      // trajectory names, which for a plan off /a2b_movement is wrong; the
      // subtree passes the set the a2b JTC gives a non-zero
      // constraints.<joint>.goal.
      BT::InputPort<std::string>("joints", "", "Comma-separated joints to check"),
      // Not the action's `goal_time`, which was leeway past the trajectory end:
      // this one bounds the whole move plus the settling of the load after it,
      // so it is a "never going to arrive" bound, not a tracking tolerance.
      BT::InputPort<int>("timeout_ms", 120000, "Give up and fail after this long"),
      BT::InputPort<std::string>("joint_states_topic", "/joint_states", "Measured joint state"),
      BT::InputPort<std::string>("settled_topic", "/crane/sway_settled", "Sway verdict"),
      // Watched, not consumed: what the MPC drives the JTC with, so this node can
      // see its own commander stop. Subscribed on the MPC arm only.
      BT::InputPort<std::string>("horizon_topic", "/crane/mpc/horizon", "The MPC's command"),
      // 0: the verdict's own dq_u_settled (0.02). A lift or a pickup descent waited ~5 s on
      // a decaying sway that nothing after it needs that still.
      BT::InputPort<double>("sway_bound", 0.0, "Sway rate bound [rad/s], 0: the verdict's"),
      // True only where this node is the commander, from the `a2b_mpc_drives`
      // ROS parameter. The sibling FollowJointTrajectory action is _skipIf'd on
      // it, so exactly one of the two commands any given move.
      BT::OutputPort<bool>("mpc_drives", "Whether crane_mpc, not the action, drives"),
      // Which half of the move this instance is, fixed by the XML. False: the
      // whole oracle, on the MPC arm only. True: the settle half, after the
      // action goal on every other arm.
      BT::InputPort<bool>("settle_only", false, "True: the goal arrived; wait for the load"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using JointState = sensor_msgs::msg::JointState;
  using JointTrajectory = trajectory_msgs::msg::JointTrajectory;
  using SwaySettled = crane_msgs::msg::SwaySettled;

  // Why the move is not over yet, or empty when it is. The sentence is what the
  // timeout reports, so the failure says which half was missing.
  std::string not_arrived_reason() const;
  // The verdict's YES, or, with a sway_bound, its rates against that. UNKNOWN never is.
  bool still(const SwaySettled & verdict) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<SwaySettled>::SharedPtr settled_sub_;
  // Null off the MPC arm, where the action goal is the commander and the horizon
  // is nobody's business.
  rclcpp::Subscription<JointTrajectory>::SharedPtr horizon_sub_;

  // crane_mpc decimates this stream by 2 and publishes every change, so a live
  // producer is quiet for two of its cycles at most: 120 ms on the 0.06 s grid
  // it ships today, and a few hundred on a box loaded enough to overrun it.
  // Eight times that is still inside this bound, which is why the bound is not
  // written as a multiple of a cycle time that moved once already.
  static constexpr std::chrono::milliseconds kSettledMaxAge{1000};
  // Before this, "no verdict yet" is indistinguishable from "the producer has not
  // been discovered". After it, zero publishers means there is no producer: fail
  // instead of spending the whole timeout on a question nothing will answer. The
  // subscription is built at tree load, long before any move starts, so this is
  // generous by a wide margin.
  static constexpr std::chrono::milliseconds kProducerGrace{3000};
  // The MPC re-goals the JTC every Ts (0.06 s), so this is not a late cycle: it is
  // `max_reference_age` spent past the plan's end, or any other gate crane_mpc falls
  // silent on. Not a multiple of Ts, though -- it is how long the *machine* outlives
  // the stream, because the move can still end on its own inside that:
  //  - the last horizon is N Ts = 3.6 s long (horizon_length 60, Ts 0.06), which
  //    crane_mpc_parameters.yaml calls "the fault-tolerance budget: that many missed
  //    solves before the receiver's buffer runs dry". Until it is spent the JTC is
  //    still driving the plan and the joints can still reach tolerance.
  //  - the sway gates never needed a commander at all: a pendulum swings whoever
  //    holds the joints, so a kSettledMinHold dwell that began before the silence
  //    finishes after it.
  // At 1000 ms both were cut off: two wall_assembly moves failed on "has not read
  // settled for long enough" with every other gate met, 1.7 s short of the dwell.
  // kProducerGrace covers the other end, where "not driving yet" and "never will"
  // are the same reading.
  static constexpr std::chrono::milliseconds kHorizonMaxAge{3700};
  // One period of the carried pendulum (2.7 s, sim): a whole cycle under the rate bound
  // caps the swing at bound / omega. 1 s let a +-1.8 cm swing through near its turning
  // points, and the block slid 5 cm on touchdown.
  static constexpr std::chrono::milliseconds kSettledMinHold{2700};

  mutable std::mutex mutex_;
  std::optional<JointState> joint_state_;
  std::optional<SwaySettled> settled_;
  std::chrono::steady_clock::time_point settled_received_;
  // When the current unbroken run of SETTLED_YES began.
  std::optional<std::chrono::steady_clock::time_point> settled_since_;
  // When the newest horizon arrived, seeded at onStart so that "never published"
  // and "stopped publishing" are one bound.
  std::chrono::steady_clock::time_point horizon_received_;

  bool mpc_drives_{false};
  bool settle_only_{false};
  double sway_bound_{0.0};
  JointTrajectory trajectory_;
  std::vector<std::string> joints_;
  double goal_tolerance_{0.05};
  int timeout_ms_{120000};
  std::chrono::steady_clock::time_point start_time_;
  // End of the plan on the node's clock, which is sim time under Gazebo. The
  // give-up watchdog deliberately stays on steady_clock: a stalled /clock must
  // not be able to hang the tree.
  rclcpp::Time plan_end_;
};

}  // namespace concrete_block_behavior_tree
