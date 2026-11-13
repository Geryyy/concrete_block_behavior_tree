#pragma once

#include <chrono>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace concrete_block_behavior_tree
{

// Arrival oracle for an A2B move nobody holds an action goal for (issue 148).
//
// With crane_mpc in active mode the JTC is re-goaled by topic every T_s (0.12 s), so the
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
//  - the passive pair theta6_tip_joint / theta7_tilt_joint has |dq_u| <=
//    `sway_bound` continuously for `hold_s`, off the same /joint_states. A joint
//    the stream does not carry, or a NaN rate, is not still.
//  - every joint in `joints` has |dq_a| <= `dq_a_bound` continuously for
//    `hold_s` (issue 249). Position alone said arrived while the crane was
//    still running: a quiet-planned A2B approach ended 0.032 rad inside the 0.05
//    tolerance with slew at -0.04 rad/s and the rotator at -0.09, and the grip
//    trajectory that followed referenced -0.058 -> 0 in one step. That stopped the
//    machine in ~0.5 s and threw 0.16 rad/s of empty tilt, with the MPC already
//    off the move and nothing left to damp it. The dwell is the same one the sway
//    half needs and for a weaker reason -- the rate is read off an encoder
//    difference, so a single sample under the bound is a sample, not a stop.
//
// THE SWAY GATE IS THE PENDULUM'S OWN RATE, NOT A VERDICT (framework review item
// 1). It used to be crane_mpc's /crane/sway_settled, which needs that node alive
// in at least shadow mode and its clock within a second of this one: on the
// machine the BT and crane_mpc run on different boxes, so IPC-vs-PC skew over
// 0.2 s failed every settle wait, and a crane_mpc that was never started failed it
// 3 s in. The verdict was `all(|dq_u| <= dq_u_settled)` on the two passive joints,
// which /joint_states carries directly, so this node reads them itself and
// crane_mpc is no longer in the loop. Nothing else changes: the same bound, the
// same dwell.
//
// Why the sway half needs a hold and not just a sample: the gate is on the passive
// *rate*, and a pendulum at maximum deflection has zero rate. A swinging load is
// therefore under the bound twice per period, at the turning points. One sample of
// it is a coin flip on exactly the load this node exists to refuse.
//
// The caches are cleared in onStart(), so nothing latched before the move can
// satisfy it; kJointStateMaxAge then covers the stream that stops mid-move, where
// the JTC still runs its last trajectory to the end and would otherwise arrive
// off a frozen sample.
//
// AND THE COMMANDER HAS TO STILL BE COMMANDING (issue 239). On the MPC arm this
// node also watches `horizon_topic`, and fails the move when it goes quiet for
// kHorizonMaxAge. crane_mpc stops publishing `max_reference_age` (crane_mpc.yaml)
// after the plan on /crane/reference ends, which on a 5-15 s plan is well into this
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
// plan) and leaves the half the goal never had. /joint_states is all it needs,
// so it passes on bare `controller:=pid` too.
//
// DECAY-ONLY, THE PI RECOVERY HOLD (issue 306). A loaded waypoint descent leaves
// the boom 0.007-0.011 rad low -- it keeps lowering ~0.8 s after the command
// reaches zero (lab day 2026-11-09) -- and the arrival check above passes that at
// its 0.05 rad goal_tolerance. The hold that gives the JTC's PI time to take it
// back used to be a bare 5 s Sleep, which does not say whether the error is gone.
// A third instance with `decay_only` is that hold: the position and rate half at a
// tight per-joint tolerance, no sway gate (nothing descends onto the sway between
// here and the next plan, and on the action arm the verdict is the *next* node's
// business), no plan-duration wait, and no profile gate -- the JTC's PI holds the
// joints on either arm, so this is the one instance that runs on both. Its timeout
// is short on purpose: the hold is active, and a loaded active hold pumps the tip
// mode (issue 234).
//
// There the dwell is on the positions as well as the rates, and it has to be: the
// band is 0.005 rad where the arrival gate's is 0.05, and at `dq_a_bound` 0.01 rad/s
// a joint crosses the whole of it in 0.5 s. "Inside it on this tick and slow for
// `hold_s`" is then satisfied by a joint creeping through, and measured: on a
// descent that left the load swinging 4.7 deg the boom crossed the band twice before
// settling 1 mrad outside it.
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
      // The decay hold ignores this port: its band is a lab-day decision, so it takes the
      // server's `hold_tolerance` parameter instead (issue 332).
      BT::InputPort<double>("goal_tolerance", 0.05, "Per-joint position tolerance [rad|m]"),
      // The exception to that one number, for the joint whose unit is not the
      // others': the decay hold asks 0.005 rad of boom and arm and 0.02 m of the
      // telescope, which is ~3 and ~2 cm at the block. Overrides only -- an
      // unlisted joint takes `goal_tolerance`.
      BT::InputPort<std::string>(
        "goal_tolerances", "", "Per-joint overrides, `<joint>:<tolerance>` comma-separated"),
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
      // Watched, not consumed: what the MPC drives the JTC with, so this node can
      // see its own commander stop. Subscribed on the MPC arm only.
      BT::InputPort<std::string>(
        "horizon_topic", "", "The MPC's command; empty: ROS param mpc_horizon_topic"),
      // 0: kSwayBound. A lift or a pickup descent waited ~5 s on a decaying sway that
      // nothing after it needs that still.
      BT::InputPort<double>("sway_bound", 0.0, "Passive rate bound [rad/s], 0: kSwayBound"),
      // Read off the holds of bag ~/Documents/sim_captures/2026-10-09-wall-assembly.
      // Over the 1285 s of its /joint_states where the actuated positions move less
      // than 1e-3 rad per 2 s -- a stillness criterion that does not use the rate
      // channel it is sizing -- max|dq_a| across the actuated five is p99 0.0008 and
      // p99.9 0.0034 rad/s. 0.01 is three times that floor and a quarter of the
      // 0.04 rad/s slew this gate exists to refuse, so it needs no per-joint map and
      // no campaign to size. Replaying the gate over that bag costs 2.72-2.74 s per
      // hold -- the dwell and nothing more; no standing sample there restarted it.
      // Sim is noisier: over the seven arrivals of a headless basic_pick_and_place the
      // worst dwell peaked at 0.0079, so in Gazebo this is 20 % of headroom, not 3x.
      BT::InputPort<double>("dq_a_bound", 0.01, "Actuated joint rate bound [rad/s|m/s]"),
      // How long both dwells have to hold (issue 255). Default kSettledMinHold, which is
      // what a *carried* block costs: one period of the loaded pendulum. Every move paid
      // it, and on an empty or pickup move the residual sway is nobody's gate -- nothing
      // descends onto it and the grasp tolerates it -- so ~1.2 s per move was spent in
      // front of a crane already standing still. The XML sets it short there and leaves
      // the loaded A2B and the placement descents at the default.
      // Short is still a dwell on both halves now that the sway gate reads the passive
      // rates off the 100 Hz stream: 0.3 s is 30 samples, where the old verdict stayed
      // fresh for a whole second and a 0.3 s hold could be carried by one of them.
      BT::InputPort<double>(
        "hold_s", kSettledMinHold.count() / 1000.0, "Dwell under both bounds [s]"),
      // True only where this node is the commander, from the `a2b_mpc_drives`
      // ROS parameter. The sibling FollowJointTrajectory action is _skipIf'd on
      // it, so exactly one of the two commands any given move.
      BT::OutputPort<bool>("mpc_drives", "Whether crane_mpc, not the action, drives"),
      // Which half of the move this instance is, fixed by the XML. False: the
      // whole oracle, on the MPC arm only. True: the settle half, after the
      // action goal on every other arm.
      BT::InputPort<bool>("settle_only", false, "True: the goal arrived; wait for the load"),
      // The third: the PI recovery hold, on either arm. The joints and their rates
      // against `goal_tolerances`, and nothing of the load.
      BT::InputPort<bool>("decay_only", false, "True: wait for the control error to decay"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using JointState = sensor_msgs::msg::JointState;
  using JointTrajectory = trajectory_msgs::msg::JointTrajectory;

  // Why the move is not over yet, or empty when it is. The sentence is what the
  // timeout reports, so the failure says which half was missing.
  std::string not_arrived_reason() const;
  // |measured - planned| for one joint. Caller holds mutex_ and has established
  // that the joint is in both the plan and /joint_states.
  double joint_error(const std::string & joint) const;
  // What that error is asked to be under: the joint's override, or goal_tolerance.
  double tolerance_for(const std::string & joint) const;
  // Every checked joint's error, logged on arrival as well as on a miss: the
  // arrival residuals per axis are what a tolerance is sized on, and until
  // issue 241 only the first offending joint of a failed move was ever reported.
  // Caller holds mutex_.
  std::string joint_errors_text() const;
  // max |dq_u| over the passive pair, NaN when the state does not carry both or a rate
  // is NaN -- an unreadable pendulum is not a still one. Caller holds mutex_.
  double sway_rate(const JointState & state) const;
  // The checked joints over dq_a_bound, with their rates; empty when every one of
  // them is under it. A joint /joint_states carries no rate for is listed too --
  // "no rate" is not "not moving". Caller holds mutex_.
  std::string moving_joints_text(const JointState & state) const;
  // Every checked joint inside its tolerance on `state`. False before onStart has a
  // plan, and false for a joint the state does not carry. Caller holds mutex_.
  bool in_band(const JointState & state) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<JointState>::SharedPtr joint_state_sub_;
  // /cbs/status/settle: the sway gate as this node applies it, per tick. Latched, shared.
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  // Null off the MPC arm, where the action goal is the commander and the horizon
  // is nobody's business.
  rclcpp::Subscription<JointTrajectory>::SharedPtr horizon_sub_;

  // The passive pair's bound when `sway_bound` is unset or <= 0, which is what two of
  // the trees pass: crane_mpc's own `dq_u_settled`, the verdict this gate replaced.
  static constexpr double kSwayBound{0.02};
  // The two passive joints of the pendulum, by name: the stacks publish different
  // /joint_states name sets in different orders.
  static constexpr const char * kSwayJoints[]{"theta6_tip_joint", "theta7_tilt_joint"};
  // Before this, the MPC's horizon not having arrived yet and it never arriving read
  // the same. The subscription is built at tree load, long before any move starts, so
  // this is generous by a wide margin.
  static constexpr std::chrono::milliseconds kProducerGrace{3000};
  // A stream that stopped must not read as arrived off its last sample: every gate here
  // is now on /joint_states, so this is the one freshness bound left. 100 Hz, so it is
  // 100 missed samples, not a late one.
  static constexpr std::chrono::milliseconds kJointStateMaxAge{1000};
  // The MPC re-goals the JTC every Ts, so this is not a late cycle: it is
  // `max_reference_age` spent past the plan's end, or any other gate crane_mpc falls
  // silent on. Not a multiple of Ts, though -- it is how long the *machine* outlives
  // the stream, because the move can still end on its own inside that. The sway gates
  // never needed a commander at all: a pendulum swings whoever holds the joints, so a
  // kSettledMinHold dwell that began before the silence finishes after it. At 1000 ms
  // that was cut off: two wall_assembly moves failed on "has not read settled for long
  // enough" with every other gate met, 1.7 s short of the dwell. kProducerGrace covers
  // the other end, where "not driving yet" and "never will" are the same reading.
  //
  // Derived from the horizon and not carried as a total (issue 247): it was
  // 3700 ms for 239's 60 x 0.06 s and sat a whole second inside 240's 80 x 0.06,
  // so the tree ended MPC-driven moves before their last horizon was spent. These
  // two must track crane_mpc.yaml's `horizon_length` and `Ts`; there is no runtime
  // path from that file to this one, so a knot change is an edit here as well.
  //
  // **The reach is the generous bound here, not the binding one, and 239's reason for
  // it was wrong.** 239 and crane_mpc_parameters.yaml both say the JTC keeps driving
  // the plan until the last horizon is spent; it does not. `topic_timeout` on the a2b
  // JTC (0.24 s) converts the stale trajectory to a zero-velocity hold long before
  // that, so the joints stop moving ~0.24 s into the silence and no later tick can
  // change the arrival error. What actually has to fit inside this bound is the sway
  // dwell plus one state age -- kSettledMinHold + kJointStateMaxAge = 3700 ms, which is
  // exactly the number 239 landed on by the other argument. The grid derivation below
  // is a superset of that and is what issue 247 asked for; its cost is that an
  // MPC-death-mid-move holds the operator on the deadman for the full bound rather
  // than for the ~1.2 s the machine is actually still moving. Whether this should
  // instead read `kSettledMinHold + kJointStateMaxAge` and stop tracking the grid at all
  // is a live question -- see 247's notes.
  static constexpr int kMpcHorizonKnots{40};
  static constexpr std::chrono::milliseconds kMpcKnot{120};
  // One knot of slack past the reach: the stream can fall silent just after a
  // horizon lands, so the bound covers the reach plus one re-goal.
  static constexpr std::chrono::milliseconds kHorizonMaxAge{
    (kMpcHorizonKnots + 1) * kMpcKnot};
  // One period of the carried pendulum (2.7 s, sim): a whole cycle under the rate bound
  // caps the swing at bound / omega. 1 s let a +-1.8 cm swing through near its turning
  // points, and the block slid 5 cm on touchdown. Now only the `hold_s` default, i.e.
  // the longest dwell the XML asks for; a caller that asks for more than this outlives
  // the horizon bound below and the static_assert no longer covers it.
  static constexpr std::chrono::milliseconds kSettledMinHold{2700};
  // The requirement above, now that the bound no longer states it: whatever the grid
  // does, a dwell begun before the silence has to be able to finish inside it.
  static_assert(
    kHorizonMaxAge >= kSettledMinHold + kJointStateMaxAge,
    "kHorizonMaxAge must outlast a sway dwell that began before the silence");

  mutable std::mutex mutex_;
  std::optional<JointState> joint_state_;
  std::chrono::steady_clock::time_point joint_state_received_;
  // When the current unbroken run of passive rates under the bound began.
  std::optional<std::chrono::steady_clock::time_point> sway_still_since_;
  // Same, for the actuated rates. Kept on the /joint_states stream and not on the
  // tick, so the dwell runs through the plan instead of starting at its end: a move
  // that was already standing still arrives without paying 2.7 s for it.
  std::optional<std::chrono::steady_clock::time_point> dq_a_still_since_;
  // Same again for the positions, which only the decay hold dwells on.
  std::optional<std::chrono::steady_clock::time_point> in_band_since_;
  // When the newest horizon arrived, seeded at onStart so that "never published"
  // and "stopped publishing" are one bound.
  std::chrono::steady_clock::time_point horizon_received_;

  bool mpc_drives_{false};
  bool settle_only_{false};
  bool decay_only_{false};
  double sway_bound_{kSwayBound};
  double dq_a_bound_{0.01};
  std::chrono::milliseconds hold_{kSettledMinHold};
  JointTrajectory trajectory_;
  std::vector<std::string> joints_;
  double goal_tolerance_{0.05};
  std::map<std::string, double> tolerance_;
  int timeout_ms_{120000};
  std::chrono::steady_clock::time_point start_time_;
  // End of the plan on the node's clock, which is sim time under Gazebo. The
  // give-up watchdog deliberately stays on steady_clock: a stalled /clock must
  // not be able to hang the tree.
  rclcpp::Time plan_end_;
};

}  // namespace concrete_block_behavior_tree
