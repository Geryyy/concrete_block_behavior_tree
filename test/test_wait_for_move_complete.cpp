// The predicate that decides an MPC-driven A2B move is over (issue 148).
//
// No sim run tells you this one is wrong until a block is placed on a swinging
// load, so the cases below are driven against the real node over real topics:
// arrived and settled, arrived and swinging, arrived and unknown, never arrived
// -- plus the three ways it could say "arrived" about a move that did not
// happen, and the profile gate that keeps the action path for `pid`. On that
// path the same node runs again after the goal as the settle wait (issue 220),
// so the tree below is the subtree's shape, both instances included.

#include <gtest/gtest.h>

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
using crane_msgs::msg::SwaySettled;
using sensor_msgs::msg::JointState;
using trajectory_msgs::msg::JointTrajectory;

namespace
{

constexpr char kCheckedJoints[] = "theta1_slewing_joint,theta2_boom_joint";

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
// planned value, where an uncalibrated pendulum reading would sit.
JointState measured(double theta1, double theta2)
{
  JointState state;
  state.name = {"q9_left_rail_joint", "theta6_tip_joint", "theta2_boom_joint",
    "theta1_slewing_joint"};
  state.position = {0.7, 0.4, theta2, theta1};
  return state;
}

struct Outcome
{
  BT::NodeStatus status;
  // 1 if the sibling that stands in for the FollowJointTrajectory goal ran.
  int commanded;
  std::chrono::milliseconds elapsed;
};

// The subtree's shape: the arrival oracle, the action goal that runs instead of
// it off the MPC arm, and the settle wait that follows that goal (issue 220).
constexpr char kTree[] =
  R"(<root BTCPP_format="4">
       <BehaviorTree ID="T">
         <Sequence>
           <WaitForMoveComplete trajectory="{trajectory}" goal_tolerance="0.05"
             joints="JOINTS" timeout_ms="TIMEOUT" mpc_drives="{mpc_drives}"/>
           <Script _skipIf="mpc_drives" code="commanded := 1"/>
           <Sequence _skipIf="ex_until_landed">
             <WaitForMoveComplete trajectory="{trajectory}" goal_tolerance="0.05"
               joints="JOINTS" timeout_ms="TIMEOUT" mpc_drives="{mpc_drives}"
               settle_only="true" _skipIf="mpc_drives"/>
           </Sequence>
         </Sequence>
       </BehaviorTree>
     </root>)";

// Both placeholders stand twice: the two instances share every port but
// settle_only.
std::string tree_text(int timeout_ms)
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
  return text;
}

// `settled_ticks` < 0 publishes the verdict for the whole run; otherwise the
// verdict stream stops after that many ticks, and `state` only starts being
// published after `arrive_after_ticks` -- a producer that died mid-move.
// `horizon_ticks` is the same for the MPC's command: < 0 drives the whole run,
// and a count is the MPC falling silent that many ticks in.
// `producer_dies_after_ticks` >= 0 destroys the verdict publisher that many ticks
// in, which is a crane_mpc that exits mid-move -- not the same as a producer that
// goes quiet, because the publisher count drops to zero.
Outcome run(
  bool mpc_drives, const JointState & state, uint8_t settled_value, int timeout_ms,
  int settled_ticks = -1, int arrive_after_ticks = 0, double plan_seconds = 0.2,
  bool until_landed = false, bool with_producer = true, int horizon_ticks = -1,
  int producer_dies_after_ticks = -1)
{
  auto node = std::make_shared<rclcpp::Node>("test_wait_for_move_complete");
  node->declare_parameter<bool>("a2b_mpc_drives", mpc_drives);

  auto state_pub = node->create_publisher<JointState>("/joint_states", rclcpp::QoS(10));
  // The move is only driven while this flows; nothing reads the content.
  auto horizon_pub = node->create_publisher<JointTrajectory>(
    "/crane/mpc/horizon", rclcpp::QoS(1));
  // No publisher at all is a different case from one that says nothing: it is
  // every profile that starts no crane_mpc.
  auto settled_pub = with_producer
    ? node->create_publisher<SwaySettled>("/crane/sway_settled", rclcpp::QoS(10))
    : nullptr;

  auto blackboard = BT::Blackboard::create();
  blackboard->set<rclcpp::Node::SharedPtr>("node", node);
  blackboard->set<JointTrajectory>("trajectory", goal_trajectory(plan_seconds));
  blackboard->set<int>("commanded", 0);
  // Every caller of the subtree sets this before the SubTree call.
  blackboard->set<bool>("ex_until_landed", until_landed);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<WaitForMoveComplete>("WaitForMoveComplete");
  auto tree = factory.createTreeFromText(tree_text(timeout_ms), blackboard);

  SwaySettled verdict;
  verdict.settled = settled_value;

  const auto started = std::chrono::steady_clock::now();
  const auto deadline = started + std::chrono::milliseconds(timeout_ms + 3000);
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  for (int tick = 0; std::chrono::steady_clock::now() < deadline; ++tick) {
    state_pub->publish(tick >= arrive_after_ticks ? state : measured(0.0, 0.0));
    if (horizon_ticks < 0 || tick < horizon_ticks) {
      horizon_pub->publish(JointTrajectory());
    }
    if (producer_dies_after_ticks >= 0 && tick >= producer_dies_after_ticks) {
      settled_pub.reset();
    }
    if (settled_pub && (settled_ticks < 0 || tick < settled_ticks)) {
      settled_pub->publish(verdict);
    }
    status = tree.tickExactlyOnce();
    if (status != BT::NodeStatus::RUNNING) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return {
    status, blackboard->get<int>("commanded"),
    std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - started)};
}

// Arrived, still, and nothing on the topic to say so.
Outcome run_without_a_verdict_producer(int timeout_ms)
{
  return run(
    false, measured(1.0, 2.0), SwaySettled::SETTLED_YES, timeout_ms, -1, 0, 0.2, false, false);
}

}  // namespace

TEST(WaitForMoveComplete, succeeds_when_the_joints_arrived_and_the_load_is_settled)
{
  const auto outcome = run(true, measured(1.0, 2.0), SwaySettled::SETTLED_YES, 6000);
  EXPECT_EQ(outcome.status, BT::NodeStatus::SUCCESS);
  // The failure this issue closes: never both commanders on one move. The
  // passive theta6_tip_joint sits 0.4 rad from its planned value throughout,
  // so this also pins that the plan's uncalibrated sway columns are not gated on.
  EXPECT_EQ(outcome.commanded, 0);
}

TEST(WaitForMoveComplete, does_not_call_a_move_over_before_the_plan_has_run)
{
  // Arrived and settled from the first tick, on a two-second plan: a placement
  // correction whose endpoint is already inside tolerance of the start.
  const auto outcome =
    run(true, measured(1.0, 2.0), SwaySettled::SETTLED_YES, 8000, -1, 0, 2.0);
  EXPECT_EQ(outcome.status, BT::NodeStatus::SUCCESS);
  EXPECT_GE(outcome.elapsed, std::chrono::milliseconds(2000));
}

TEST(WaitForMoveComplete, does_not_accept_a_single_turning_point_reading_as_settled)
{
  // The verdict is on the passive *rate*, so a load at full deflection reads
  // SETTLED_YES for an instant. Ten ticks of YES and then silence is that
  // instant; it must not carry the move.
  const auto outcome = run(true, measured(1.0, 2.0), SwaySettled::SETTLED_YES, 900, 10);
  EXPECT_EQ(outcome.status, BT::NodeStatus::FAILURE);
}

TEST(WaitForMoveComplete, fails_when_the_joints_arrived_but_the_load_is_swinging)
{
  const auto outcome = run(true, measured(1.0, 2.0), SwaySettled::SETTLED_NO, 900);
  EXPECT_EQ(outcome.status, BT::NodeStatus::FAILURE);
  EXPECT_EQ(outcome.commanded, 0);
}

TEST(WaitForMoveComplete, fails_when_the_sway_is_unknown)
{
  // Unknowable must not read as settled -- that is what the third value is for.
  const auto outcome = run(true, measured(1.0, 2.0), SwaySettled::SETTLED_UNKNOWN, 900);
  EXPECT_EQ(outcome.status, BT::NodeStatus::FAILURE);
}

TEST(WaitForMoveComplete, does_not_let_a_dead_producer_certify_an_arrival_it_never_saw)
{
  // The verdict stream stops 0.2 s in; the joints reach the goal 1.4 s in, by
  // which time the last SETTLED_YES is older than kSettledMaxAge.
  const auto outcome =
    run(true, measured(1.0, 2.0), SwaySettled::SETTLED_YES, 2500, 10, 70);
  EXPECT_EQ(outcome.status, BT::NodeStatus::FAILURE);
}

TEST(WaitForMoveComplete, fails_instead_of_hanging_when_the_move_never_arrives)
{
  const auto outcome = run(true, measured(0.2, 2.0), SwaySettled::SETTLED_YES, 900);
  EXPECT_EQ(outcome.status, BT::NodeStatus::FAILURE);
  EXPECT_EQ(outcome.commanded, 0);
}

TEST(WaitForMoveComplete, stands_aside_where_the_action_goal_is_still_the_commander)
{
  // pid / mpc / cbs_mpc: the arrival oracle skips itself and the action runs
  // unchanged. Commanding is all it gives up there -- the settle wait after the
  // goal is the same node.
  const auto outcome = run(false, measured(1.0, 2.0), SwaySettled::SETTLED_YES, 6000);
  EXPECT_EQ(outcome.status, BT::NodeStatus::SUCCESS);
  EXPECT_EQ(outcome.commanded, 1);
}

TEST(WaitForMoveComplete, does_not_let_the_action_arm_move_on_from_a_swinging_load)
{
  // Issue 220, and the whole reason the settle wait exists: the goal returns on
  // positions and a goal_time, and nothing on this arm damps sway, so the tree
  // used to descend into a swinging block the instant the goal came back.
  const auto outcome = run(false, measured(1.0, 2.0), SwaySettled::SETTLED_NO, 900);
  EXPECT_EQ(outcome.status, BT::NodeStatus::FAILURE);
  EXPECT_EQ(outcome.commanded, 1);
}

TEST(WaitForMoveComplete, leaves_a_landing_descent_to_wait_for_load_off)
{
  // A landing descent is ended by WaitForLoadOff with the block on the ground,
  // and the oracle before the goal is skipped there too -- so `mpc_drives` is
  // whatever the move before left on the blackboard. Gating a touchdown on sway
  // would fail the landing the Parallel's sibling is still deciding, and would
  // read an unset key on any tree that reached a descent first.
  const auto outcome =
    run(false, measured(1.0, 2.0), SwaySettled::SETTLED_NO, 900, -1, 0, 0.2, true);
  EXPECT_EQ(outcome.status, BT::NodeStatus::SUCCESS);
  EXPECT_EQ(outcome.commanded, 1);
}

TEST(WaitForMoveComplete, fails_at_once_when_nothing_publishes_the_verdict)
{
  // controller:=pid starts no crane_mpc, so the topic has no publisher at all.
  // Spending the whole budget on that -- controller active, deadman held, machine
  // motionless -- is the worst way to find out, so it goes after kProducerGrace.
  const auto outcome = run_without_a_verdict_producer(20000);
  EXPECT_EQ(outcome.status, BT::NodeStatus::FAILURE);
  EXPECT_EQ(outcome.commanded, 1);
  EXPECT_LT(outcome.elapsed, std::chrono::milliseconds(12000));
}

TEST(WaitForMoveComplete, fails_at_once_when_the_verdict_producer_dies_mid_move)
{
  // Issue 241, the settle arm's sibling of 239. The joints arrived and the load
  // reads still, but crane_mpc exits 0.6 s in -- before the 2.7 s dwell can
  // complete -- so the publisher count drops to zero and nothing will ever
  // finish the dwell. The check used to be gated on having heard no verdict at
  // all, which a producer that dies *mid*-move has already defeated, so this
  // burned the settle wait's whole 60 s in front of a motionless machine with
  // the deadman held.
  // 20 ticks, ~0.4 s: it has to be long enough that a verdict was heard (or the
  // old code fails here too and this proves nothing) and short enough that the
  // 2.7 s dwell cannot complete first (or the node legitimately succeeds). Those
  // bound the tick period to 135 ms, against ~20 ms measured.
  const auto outcome = run(
    false, measured(1.0, 2.0), SwaySettled::SETTLED_YES, 20000, -1, 0, 0.2, false, true, -1, 20);
  EXPECT_EQ(outcome.status, BT::NodeStatus::FAILURE);
  // It is the settle wait that fails, so the goal did run.
  EXPECT_EQ(outcome.commanded, 1);
  EXPECT_LT(outcome.elapsed, std::chrono::milliseconds(10000));
}

TEST(WaitForMoveComplete, fails_at_once_when_the_mpc_stops_driving_the_move)
{
  // Issue 239. crane_mpc goes silent `max_reference_age` after the plan on
  // /crane/reference ends and the JTC then holds its own last horizon point, so
  // the arrival error freezes: this move is 0.8 rad short of its goal and will
  // stay there. It used to spend the rest of the window -- 90 s live -- and then
  // report the tolerance miss from 90 s earlier.
  const auto outcome = run(
    true, measured(0.2, 2.0), SwaySettled::SETTLED_YES, 60000, -1, 0, 0.2, false, true, 20);
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
  const auto outcome = run(
    true, measured(1.0, 2.0), SwaySettled::SETTLED_YES, 20000, -1, 200, 0.2, false, true, 90);
  EXPECT_EQ(outcome.status, BT::NodeStatus::SUCCESS);
}

TEST(WaitForMoveComplete, does_not_replay_the_plan_duration_after_the_goal_arrived)
{
  // The goal already played the plan. Anchoring the settle wait's plan end on
  // its own onStart would add the whole plan to every move on this arm, which
  // is a 20 s plan inside an 8 s budget here.
  const auto outcome = run(false, measured(1.0, 2.0), SwaySettled::SETTLED_YES, 8000, -1, 0, 20.0);
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
