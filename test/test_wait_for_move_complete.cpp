// The predicate that decides an MPC-driven A2B move is over (issue 148).
//
// No sim run tells you this one is wrong until a block is placed on a swinging
// load, so the cases below are driven against the real node over real topics:
// arrived and settled, arrived and swinging, arrived and unknown, never arrived
// -- plus the three ways it could say "arrived" about a move that did not
// happen, and the profile gate that keeps the action path for `pid`.

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

// `settled_ticks` < 0 publishes the verdict for the whole run; otherwise the
// verdict stream stops after that many ticks, and `state` only starts being
// published after `arrive_after_ticks` -- a producer that died mid-move.
Outcome run(
  bool mpc_drives, const JointState & state, uint8_t settled_value, int timeout_ms,
  int settled_ticks = -1, int arrive_after_ticks = 0, double plan_seconds = 0.2)
{
  auto node = std::make_shared<rclcpp::Node>("test_wait_for_move_complete");
  node->declare_parameter<bool>("a2b_mpc_drives", mpc_drives);

  auto state_pub = node->create_publisher<JointState>("/joint_states", rclcpp::QoS(10));
  auto settled_pub = node->create_publisher<SwaySettled>("/crane/sway_settled", rclcpp::QoS(10));

  auto blackboard = BT::Blackboard::create();
  blackboard->set<rclcpp::Node::SharedPtr>("node", node);
  blackboard->set<JointTrajectory>("trajectory", goal_trajectory(plan_seconds));
  blackboard->set<int>("commanded", 0);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<WaitForMoveComplete>("WaitForMoveComplete");
  auto tree = factory.createTreeFromText(
    R"(<root BTCPP_format="4">
         <BehaviorTree ID="T">
           <Sequence>
             <WaitForMoveComplete trajectory="{trajectory}" goal_tolerance="0.05"
               joints=")" + std::string(kCheckedJoints) + R"("
               timeout_ms=")" + std::to_string(timeout_ms) + R"("
               mpc_drives="{mpc_drives}"/>
             <Script _skipIf="mpc_drives" code="commanded := 1"/>
           </Sequence>
         </BehaviorTree>
       </root>)",
    blackboard);

  SwaySettled verdict;
  verdict.settled = settled_value;

  const auto started = std::chrono::steady_clock::now();
  const auto deadline = started + std::chrono::milliseconds(timeout_ms + 3000);
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  for (int tick = 0; std::chrono::steady_clock::now() < deadline; ++tick) {
    state_pub->publish(tick >= arrive_after_ticks ? state : measured(0.0, 0.0));
    if (settled_ticks < 0 || tick < settled_ticks) {
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
  // pid / mpc / cbs_mpc: this node skips itself and the action runs unchanged,
  // even though nothing here is arrived or settled.
  const auto outcome = run(false, measured(0.0, 0.0), SwaySettled::SETTLED_UNKNOWN, 900);
  EXPECT_EQ(outcome.status, BT::NodeStatus::SUCCESS);
  EXPECT_EQ(outcome.commanded, 1);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
