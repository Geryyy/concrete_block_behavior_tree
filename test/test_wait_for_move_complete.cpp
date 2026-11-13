// The predicate that decides an MPC-driven A2B move is over (issue 148).
//
// No sim run tells you this one is wrong until a block is placed on a swinging
// load, so the cases below are driven against the real node over real topics:
// arrived and settled, arrived and still running, arrived and swinging, arrived
// with the pendulum unreadable, never arrived
// -- plus the three ways it could say "arrived" about a move that did not
// happen, and the profile gate that keeps the action path for `pid`. On that
// path the same node runs again after the goal as the settle wait (issue 220),
// and on a loaded waypoint descent a third time as the PI recovery hold
// (issue 306: the joints alone, at a tenth of the arrival tolerance), so the
// tree below is the subtree's shape, all three instances included.
//
// Both halves now come off /joint_states: the sway gate used to be crane_mpc's
// /crane/sway_settled verdict, which needed that node alive and its clock inside
// a second of this one (framework review item 1).

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "concrete_block_behavior_tree/plugins/condition/wait_for_move_complete.hpp"
#include "crane_msgs/msg/sway_settled.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

using concrete_block_behavior_tree::WaitForMoveComplete;
using sensor_msgs::msg::JointState;
using trajectory_msgs::msg::JointTrajectory;

namespace
{

constexpr char kCheckedJoints[] = "theta1_slewing_joint,theta2_boom_joint";
// Over the node's 0.02 rad/s bound by 15x: a load still swinging at the arrival.
constexpr double kSwinging = 0.3;

// Two checked joints plus a passive one, which is what a plan off /a2b_movement
// looks like: the canonical eight carry theta6/theta7, which nothing aims.
JointTrajectory goal_trajectory(double plan_seconds)
{
  JointTrajectory trajectory;
  trajectory.joint_names = {"theta1_slewing_joint", "theta2_boom_joint", "theta6_tip_joint"};
  trajectory.points.resize(2);
  trajectory.points[0].positions = {0.0, 0.0, 0.0};
  trajectory.points[1].positions = {1.0, 2.0, 0.0};
  trajectory.points[1].time_from_start = rclcpp::Duration::from_seconds(plan_seconds);
  return trajectory;
}

// Deliberately neither the order nor the width of the trajectory: the two stacks
// publish different /joint_states name sets in different orders, so a node that
// matches by index reads the wrong axis here. theta6 is parked far from its
// planned value, where an uncalibrated pendulum reading would sit: the passive
// *positions* are nobody's gate, only their rates are.
JointState measured(double theta1, double theta2, double dq_theta1 = 0.0, double dq_u = 0.0)
{
  JointState state;
  state.name = {"q9_left_rail_joint", "theta6_tip_joint", "theta2_boom_joint",
    "theta7_tilt_joint", "theta1_slewing_joint"};
  state.position = {0.7, 0.4, theta2, 0.1, theta1};
  state.velocity = {0.0, dq_u, 0.0, dq_u, dq_theta1};
  return state;
}

// The same state with the tilt axis gone from the stream, which is a dead sensor
// or a renamed joint. Half a pendulum is not a readable one.
JointState without_tilt(JointState state)
{
  const auto at = std::find(state.name.begin(), state.name.end(), "theta7_tilt_joint");
  const auto i = std::distance(state.name.begin(), at);
  state.name.erase(at);
  state.position.erase(state.position.begin() + i);
  state.velocity.erase(state.velocity.begin() + i);
  return state;
}

struct Outcome
{
  BT::NodeStatus status;
  // 1 if the sibling that stands in for the FollowJointTrajectory goal ran.
  int commanded;
  // 1 if the PI recovery hold passed; 0 if it failed or was skipped.
  int held;
  std::chrono::milliseconds elapsed;
};

// The subtree's shape: the arrival oracle, the action goal that runs instead of
// it off the MPC arm, the PI recovery hold on the loaded waypoint descents
// (issue 306, on either arm) and the settle wait that follows the goal (220).
// The hold takes its own tolerance from the server's `hold_tolerance` parameter
// (0.005 by default, issue 332), not from the port; its one override stands for
// the subtree's telescope, whose tolerance is metres.
constexpr char kTree[] =
  R"(<root BTCPP_format="4">
       <BehaviorTree ID="T">
         <Sequence>
           <WaitForMoveComplete trajectory="{trajectory}" goal_tolerance="0.05"
             joints="JOINTS" timeout_ms="TIMEOUT" hold_s="HOLD" mpc_drives="{mpc_drives}"/>
           <Script _skipIf="mpc_drives" code="commanded := 1"/>
           <WaitForMoveComplete trajectory="{trajectory}"
             goal_tolerances="theta1_slewing_joint:0.05" decay_only="true"
             joints="JOINTS" timeout_ms="TIMEOUT" hold_s="HOLD"
             _skipIf="ex_until_landed || !ex_watch_landed"/>
           <Script _skipIf="ex_until_landed || !ex_watch_landed" code="held := 1"/>
           <Sequence _skipIf="ex_until_landed">
             <WaitForMoveComplete trajectory="{trajectory}" goal_tolerance="0.05"
               joints="JOINTS" timeout_ms="TIMEOUT" hold_s="HOLD" mpc_drives="{mpc_drives}"
               settle_only="true" _skipIf="mpc_drives"/>
           </Sequence>
         </Sequence>
       </BehaviorTree>
     </root>)";

// Every placeholder stands twice: the two instances share every port but
// settle_only.
std::string tree_text(int timeout_ms, double hold_s)
{
  std::string text = kTree;
  const auto substitute = [&text](const std::string & token, const std::string & value) {
      for (size_t at = text.find(token); at != std::string::npos; at = text.find(token, at)) {
        text.replace(at, token.size(), value);
        at += value.size();
      }
    };
  substitute("JOINTS", kCheckedJoints);
  substitute("TIMEOUT", std::to_string(timeout_ms));
  substitute("HOLD", std::to_string(hold_s));
  return text;
}

// `sway_ticks` >= 0 starts the pendulum swinging that many ticks in, which is a
// load that reads still for an instant -- a turning point -- and then does not.
// `state` only starts being published after `arrive_after_ticks`.
// `horizon_ticks` is the same for the MPC's command: < 0 drives the whole run,
// and a count is the MPC falling silent that many ticks in.
// `hold_s` is the dwell both arrival gates have to hold, defaulted to the loaded
// 2.7 s so that every case above it reads as the move it was written for.
// `watch_landed` is a loaded descent to a waypoint, the one move the PI recovery
// hold runs on. `state_ticks` >= 0 stops /joint_states that many ticks in, which is
// the broadcaster dying mid-move -- the one input either half has.
Outcome run(
  bool mpc_drives, const JointState & state, int timeout_ms,
  int sway_ticks = -1, int arrive_after_ticks = 0, double plan_seconds = 0.2,
  bool until_landed = false, int horizon_ticks = -1, double hold_s = 2.7,
  bool watch_landed = false, int state_ticks = -1)
{
  auto node = std::make_shared<rclcpp::Node>("test_wait_for_move_complete");
  node->declare_parameter<bool>("a2b_mpc_drives", mpc_drives);

  auto state_pub = node->create_publisher<JointState>("/joint_states", rclcpp::QoS(10));
  // The move is only driven while this flows; nothing reads the content.
  auto horizon_pub = node->create_publisher<JointTrajectory>(
    "/crane/mpc/horizon", rclcpp::QoS(1));

  auto blackboard = BT::Blackboard::create();
  blackboard->set<rclcpp::Node::SharedPtr>("node", node);
  blackboard->set<JointTrajectory>("trajectory", goal_trajectory(plan_seconds));
  blackboard->set<int>("commanded", 0);
  blackboard->set<int>("held", 0);
  // Every caller of the subtree sets these before the SubTree call.
  blackboard->set<bool>("ex_until_landed", until_landed);
  blackboard->set<bool>("ex_watch_landed", watch_landed);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<WaitForMoveComplete>("WaitForMoveComplete");
  auto tree = factory.createTreeFromText(tree_text(timeout_ms, hold_s), blackboard);

  const auto started = std::chrono::steady_clock::now();
  const auto deadline = started + std::chrono::milliseconds(timeout_ms + 3000);
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  for (int tick = 0; std::chrono::steady_clock::now() < deadline; ++tick) {
    if (state_ticks < 0 || tick < state_ticks) {
      JointState sample = tick >= arrive_after_ticks ? state : measured(0.0, 0.0);
      if (sway_ticks >= 0 && tick >= sway_ticks) {
        for (const std::string joint : {"theta6_tip_joint", "theta7_tilt_joint"}) {
          const auto at = std::find(sample.name.begin(), sample.name.end(), joint);
          sample.velocity[std::distance(sample.name.begin(), at)] = kSwinging;
        }
      }
      state_pub->publish(sample);
    }
    if (horizon_ticks < 0 || tick < horizon_ticks) {
      horizon_pub->publish(JointTrajectory());
    }
    status = tree.tickExactlyOnce();
    if (status != BT::NodeStatus::RUNNING) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return {
    status, blackboard->get<int>("commanded"), blackboard->get<int>("held"),
    std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - started)};
}

}  // namespace

TEST(WaitForMoveComplete, succeeds_when_the_joints_arrived_and_the_load_is_settled)
{
  const auto outcome = run(true, measured(1.0, 2.0), 6000);
  EXPECT_EQ(outcome.status, BT::NodeStatus::SUCCESS);
  // The failure this issue closes: never both commanders on one move. The
  // passive theta6_tip_joint sits 0.4 rad from its planned value throughout,
  // so this also pins that the plan's uncalibrated sway columns are not gated on.
  EXPECT_EQ(outcome.commanded, 0);
}

TEST(WaitForMoveComplete, settles_with_nothing_publishing_the_sway_verdict)
{
  // Framework review item 1, and the whole point of it: `controller:=pid` starts no
  // crane_mpc, so /crane/sway_settled has no publisher at all -- and on the machine the
  // two run on different boxes, where a clock skew over 0.2 s aged every verdict out.
  // This used to fail after kProducerGrace with the controller active and the operator
  // on the deadman. Nothing subscribes the verdict any more, which is the other half of
  // the claim: crane_mpc is out of this loop, not merely tolerated in it.
  auto probe = std::make_shared<rclcpp::Node>("sway_verdict_probe");
  auto verdict = probe->create_publisher<crane_msgs::msg::SwaySettled>(
    "/crane/sway_settled", rclcpp::QoS(10));
  const auto outcome = run(false, measured(1.0, 2.0), 6000);
  EXPECT_EQ(outcome.status, BT::NodeStatus::SUCCESS);
  EXPECT_EQ(outcome.commanded, 1);
  EXPECT_EQ(verdict->get_subscription_count(), 0u);
}

TEST(WaitForMoveComplete, does_not_call_a_move_over_before_the_plan_has_run)
{
  // Arrived and settled from the first tick, on a two-second plan: a placement
  // correction whose endpoint is already inside tolerance of the start.
  const auto outcome = run(true, measured(1.0, 2.0), 8000, -1, 0, 2.0);
  EXPECT_EQ(outcome.status, BT::NodeStatus::SUCCESS);
  EXPECT_GE(outcome.elapsed, std::chrono::milliseconds(2000));
}

TEST(WaitForMoveComplete, a_short_hold_does_not_pay_the_carried_pendulum_on_an_empty_move)
{
  // Issue 255. The 2.7 s dwell is one period of the *loaded* pendulum, and every
  // move paid it: measured ~1.2 s of pure pause after each of the eight moves of
  // a wall-assembly run, in front of a crane that had already stopped. An empty
  // or pickup move has no gate on its residual sway, so the XML asks for 0.3 s
  // there -- enough to tell a stop from one sample under the rate bound, which is
  // all the dwell is for without a block on the hook.
  const auto loaded = run(true, measured(1.0, 2.0), 8000);
  const auto empty = run(true, measured(1.0, 2.0), 8000, -1, 0, 0.2, false, -1, 0.3);
  EXPECT_EQ(loaded.status, BT::NodeStatus::SUCCESS);
  EXPECT_EQ(empty.status, BT::NodeStatus::SUCCESS);
  // 2.4 s is the difference of the two dwells; both runs pay the same 0.2 s plan
  // and the same tick period on top, so the gap is the dwell and nothing else.
  EXPECT_GE(loaded.elapsed - empty.elapsed, std::chrono::milliseconds(2000));
}

TEST(WaitForMoveComplete, does_not_accept_a_single_still_sample_as_settled)
{
  // The gate is on the passive *rate*, so a load at full deflection reads still
  // for an instant. Ten ticks of it and then a swing is that instant; it must not
  // carry the move. The same run without the swing succeeds at 0.3 s of dwell
  // (the empty-move case above), so this is the swing and not the budget.
  const auto outcome = run(true, measured(1.0, 2.0), 2000, 10, 0, 0.2, false, -1, 0.3);
  EXPECT_EQ(outcome.status, BT::NodeStatus::FAILURE);
}

TEST(WaitForMoveComplete, does_not_call_a_move_over_while_the_actuated_joints_still_run)
{
  // Issue 249. Every position inside tolerance and the load settled, but the slew
  // is still turning at the -0.04 rad/s the live run measured 0.032 rad off its
  // goal. Arriving there hands a moving crane to a trajectory that starts from
  // zero velocity, which brakes it in ~0.5 s and throws the sway the MPC is no
  // longer on the move to damp.
  // The same 6 s window the success case above gets, not the 900 ms the other
  // failures get: inside 900 ms the sway dwell has not completed either, so a
  // short window fails with or without this gate and would pin nothing.
  const auto outcome = run(true, measured(1.0, 2.0, -0.04), 6000);
  EXPECT_EQ(outcome.status, BT::NodeStatus::FAILURE);
  EXPECT_EQ(outcome.commanded, 0);
}

TEST(WaitForMoveComplete, fails_when_the_joints_arrived_but_the_load_is_swinging)
{
  const auto outcome = run(true, measured(1.0, 2.0, 0.0, kSwinging), 900);
  EXPECT_EQ(outcome.status, BT::NodeStatus::FAILURE);
  EXPECT_EQ(outcome.commanded, 0);
}

TEST(WaitForMoveComplete, fails_when_the_pendulum_is_not_in_the_stream)
{
  // Unreadable must not read as settled: the verdict had SETTLED_UNKNOWN for this,
  // and off /joint_states it is a joint the stream does not carry, or a NaN rate.
  const auto outcome = run(true, without_tilt(measured(1.0, 2.0)), 900);
  EXPECT_EQ(outcome.status, BT::NodeStatus::FAILURE);
}

TEST(WaitForMoveComplete, fails_instead_of_hanging_when_the_move_never_arrives)
{
  const auto outcome = run(true, measured(0.2, 2.0), 900);
  EXPECT_EQ(outcome.status, BT::NodeStatus::FAILURE);
  EXPECT_EQ(outcome.commanded, 0);
}

TEST(WaitForMoveComplete, stands_aside_where_the_action_goal_is_still_the_commander)
{
  // pid / mpc / cbs_mpc: the arrival oracle skips itself and the action runs
  // unchanged. Commanding is all it gives up there -- the settle wait after the
  // goal is the same node.
  const auto outcome = run(false, measured(1.0, 2.0), 6000);
  EXPECT_EQ(outcome.status, BT::NodeStatus::SUCCESS);
  EXPECT_EQ(outcome.commanded, 1);
}

TEST(WaitForMoveComplete, does_not_let_the_action_arm_move_on_from_a_swinging_load)
{
  // Issue 220, and the whole reason the settle wait exists: the goal returns on
  // positions and a goal_time, and nothing on this arm damps sway, so the tree
  // used to descend into a swinging block the instant the goal came back.
  const auto outcome = run(false, measured(1.0, 2.0, 0.0, kSwinging), 900);
  EXPECT_EQ(outcome.status, BT::NodeStatus::FAILURE);
  EXPECT_EQ(outcome.commanded, 1);
}

TEST(WaitForMoveComplete, the_recovery_hold_ends_a_loaded_descent_once_the_error_has_decayed)
{
  // Issue 306, and on both arms: the JTC's PI holds the joints whoever commanded
  // the move, so the hold is the one instance the action arm does not skip.
  // theta1 stands 0.02 off, which only its override allows -- in the subtree that
  // override is the telescope, 0.02 m being the same ~2 cm at the block as the
  // 0.003 rad asked of boom and arm, and a millimetre gate on a prismatic axis
  // would time out on every descent.
  for (const bool mpc_drives : {true, false}) {
    const auto outcome = run(
      mpc_drives, measured(0.98, 2.0), 8000, -1, 0, 0.2, false, -1, 0.3, true);
    EXPECT_EQ(outcome.status, BT::NodeStatus::SUCCESS) << "mpc_drives=" << mpc_drives;
    EXPECT_EQ(outcome.held, 1) << "mpc_drives=" << mpc_drives;
  }
}

TEST(WaitForMoveComplete, the_recovery_hold_fails_on_a_coast_the_arrival_gate_passed)
{
  // The lab day's 0.007-0.011 rad of boom coast, as 0.02 here: inside the move's
  // 0.05 rad arrival tolerance -- which is why the arrival check never caught it --
  // and well over the hold's 0.003. The fixed 5 s Sleep that used to stand here
  // ended whatever the residual was; the timeout now fails the goal instead of
  // measuring the block off a boom that never came back.
  const auto outcome = run(
    false, measured(1.0, 1.98), 3000, -1, 0, 0.2, false, -1, 0.3, true);
  EXPECT_EQ(outcome.status, BT::NodeStatus::FAILURE);
  EXPECT_EQ(outcome.commanded, 1);  // the move itself arrived
  EXPECT_EQ(outcome.held, 0);
  // At its own timeout and not hanging: the hold is active, and a loaded active hold
  // pumps the tip mode (issue 234), so the timeout is the bound on it.
  EXPECT_LT(outcome.elapsed, std::chrono::milliseconds(5000));
}

TEST(WaitForMoveComplete, the_recovery_hold_dwells_on_the_position_and_not_only_on_the_rate)
{
  // The band is 0.003 rad and dq_a_bound 0.01 rad/s, so a joint creeping under the
  // bound crosses the whole band in 0.3 s: inside it on one tick is not "decayed".
  // Here the joints are 1 rad out for the first second and their reported rate is
  // zero throughout, so the rate dwell completes while the move is nowhere near its
  // goal -- a hold that dwelled on the rate alone would pass at 1.0 s.
  const auto outcome = run(
    false, measured(1.0, 2.0), 20000, -1, 50, 0.2, false, -1, 1.0, true);
  EXPECT_EQ(outcome.status, BT::NodeStatus::SUCCESS);
  EXPECT_EQ(outcome.held, 1);
  // 1 s out of band, then the 1 s dwell, then the settle wait's own 1 s: 3 s. Without
  // the dwell on the positions the hold would end at 1 s and the tree at 2.
  EXPECT_GE(outcome.elapsed, std::chrono::milliseconds(2800));
}

TEST(WaitForMoveComplete, the_recovery_hold_refuses_a_joint_state_stream_that_stopped)
{
  // With /joint_states gone the dwells would run out on a frozen sample and the hold
  // would report the error decayed, with the controller still active over the slot and
  // the measurement that follows taken off the same dead state. Since item 1 the sway
  // half has no other input either, so this is the one freshness bound left. The stream
  // stops 0.4 s in, which is inside the dwell.
  const auto outcome = run(
    false, measured(1.0, 2.0), 6000, -1, 0, 0.2, false, -1, 2.7, true, 20);
  EXPECT_EQ(outcome.status, BT::NodeStatus::FAILURE);
  EXPECT_EQ(outcome.held, 0);
}

TEST(WaitForMoveComplete, the_recovery_hold_runs_on_no_other_move)
{
  // The same 0.02 coast, which the hold would fail. On a landing descent the block
  // is on the ground and WaitForLoadOff ended the move; on every other move the
  // 0.05 arrival gate is the whole test and nothing descends onto the residual.
  // Skipped, not waited out: a hold on each would cost a run its empty moves.
  const auto landing = run(
    false, measured(1.0, 1.98), 8000, -1, 0, 0.2, true, -1, 0.3, true);
  const auto empty = run(
    false, measured(1.0, 1.98), 8000, -1, 0, 0.2, false, -1, 0.3, false);
  EXPECT_EQ(landing.status, BT::NodeStatus::SUCCESS);
  EXPECT_EQ(landing.held, 0);
  EXPECT_EQ(empty.status, BT::NodeStatus::SUCCESS);
  EXPECT_EQ(empty.held, 0);
}

TEST(WaitForMoveComplete, the_recovery_hold_asks_nothing_of_the_load)
{
  // It gates the joints and their rates and no sway, so it passes with the pendulum
  // still swinging. The settle wait after it is what fails that run, which is the
  // decision issue 220 took; the hold must not fail it a second time, and for a
  // question it never asks.
  const auto outcome = run(
    false, measured(1.0, 2.0, 0.0, kSwinging), 20000, -1, 0, 0.2, false, -1, 0.3, true);
  EXPECT_EQ(outcome.held, 1);
  EXPECT_EQ(outcome.status, BT::NodeStatus::FAILURE);
}

TEST(WaitForMoveComplete, leaves_a_landing_descent_to_wait_for_load_off)
{
  // A landing descent is ended by WaitForLoadOff with the block on the ground,
  // and the oracle before the goal is skipped there too -- so `mpc_drives` is
  // whatever the move before left on the blackboard. Gating a touchdown on sway
  // would fail the landing the Parallel's sibling is still deciding, and would
  // read an unset key on any tree that reached a descent first.
  const auto outcome = run(false, measured(1.0, 2.0, 0.0, kSwinging), 900, -1, 0, 0.2, true);
  EXPECT_EQ(outcome.status, BT::NodeStatus::SUCCESS);
  EXPECT_EQ(outcome.commanded, 1);
}

TEST(WaitForMoveComplete, fails_at_once_when_the_mpc_stops_driving_the_move)
{
  // Issue 239. crane_mpc goes silent `max_reference_age` after the plan on
  // /crane/reference ends and the JTC then holds its own last horizon point, so
  // the arrival error freezes: this move is 0.8 rad short of its goal and will
  // stay there. It used to spend the rest of the window -- 90 s live -- and then
  // report the tolerance miss from 90 s earlier.
  const auto outcome = run(true, measured(0.2, 2.0), 60000, -1, 0, 0.2, false, 20);
  EXPECT_EQ(outcome.status, BT::NodeStatus::FAILURE);
  EXPECT_LT(outcome.elapsed, std::chrono::milliseconds(8000));
}

TEST(WaitForMoveComplete, lets_a_move_the_last_horizon_still_finishes_finish)
{
  // The other half of 239: the stream stopping is not the machine stopping. The
  // JTC keeps its last trajectory for `topic_timeout` and the load swings whoever
  // holds the joints, so a move that reaches tolerance -- or completes its settled
  // dwell -- inside that is over, not failed. Horizon stops at ~1.8 s and the joints
  // arrive at ~4.0 s: at a 1 s bound this failed at 3.0 s, a second before the
  // arrival it was already driving towards.
  const auto outcome = run(true, measured(1.0, 2.0), 20000, -1, 200, 0.2, false, 90);
  EXPECT_EQ(outcome.status, BT::NodeStatus::SUCCESS);
}

TEST(WaitForMoveComplete, does_not_replay_the_plan_duration_after_the_goal_arrived)
{
  // The goal already played the plan. Anchoring the settle wait's plan end on
  // its own onStart would add the whole plan to every move on this arm, which
  // is a 20 s plan inside an 8 s budget here.
  const auto outcome = run(false, measured(1.0, 2.0), 8000, -1, 0, 20.0);
  EXPECT_EQ(outcome.status, BT::NodeStatus::SUCCESS);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
