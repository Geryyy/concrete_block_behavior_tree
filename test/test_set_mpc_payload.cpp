// What the MPC is told the crane carries (issue 175).
//
// Driven against a real SetPayload server over a real service, because the two
// things that can go wrong are both on the wire: the request the grasp produces,
// and what happens on a profile where no crane_mpc answers at all.

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "concrete_block_behavior_tree/plugins/action/set_mpc_payload.hpp"
#include "crane_msgs/msg/payload.hpp"
#include "crane_msgs/srv/set_payload.hpp"
#include "rclcpp/rclcpp.hpp"

using concrete_block_behavior_tree::SetMpcPayload;
using crane_msgs::msg::Payload;
using crane_msgs::srv::SetPayload;

namespace
{

struct Outcome
{
  BT::NodeStatus status;
  std::vector<Payload> requests;
};

// Ticks `body` against a server on `service_name` that answers `accept`, or
// against nothing when `serve` is false. `mpc_drives` is the launch profile:
// true only on cbs_mpc_active, where crane_mpc commands the crane. The server
// node is spun on its own thread: the BT node waits for the answer inside tick().
Outcome run(
  const std::string & service_name, bool serve, const std::string & body,
  bool mpc_drives = false, bool accept = true)
{
  auto server_node = std::make_shared<rclcpp::Node>("test_set_mpc_payload_server");
  auto requests = std::make_shared<std::vector<Payload>>();
  rclcpp::Service<SetPayload>::SharedPtr service;
  if (serve) {
    service = server_node->create_service<SetPayload>(
      service_name,
      [requests, accept](
        const std::shared_ptr<SetPayload::Request> request,
        std::shared_ptr<SetPayload::Response> response) {
        requests->push_back(request->payload);
        response->success = accept;
        response->active_mass = accept ? request->payload.mass : 0.0;
        response->message = accept ? "accepted" : "the MPC is not configured";
      });
  }
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node);
  std::thread spinner([&executor]() {executor.spin();});

  auto node = std::make_shared<rclcpp::Node>("test_set_mpc_payload");
  node->declare_parameter<bool>("a2b_mpc_drives", mpc_drives);
  auto blackboard = BT::Blackboard::create();
  blackboard->set<rclcpp::Node::SharedPtr>("node", node);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<SetMpcPayload>("SetMpcPayload");
  auto tree = factory.createTreeFromText(
    R"(<root BTCPP_format="4"><BehaviorTree ID="T"><Sequence>)" + body +
    R"(</Sequence></BehaviorTree></root>)",
    blackboard);

  const BT::NodeStatus status = tree.tickWhileRunning();

  executor.cancel();
  spinner.join();
  return {status, *requests};
}

}  // namespace

TEST(SetMpcPayload, carries_the_block_mass_on_grip_and_clears_it_on_release)
{
  // The grasp offset CaptureBlockGraspOffset wrote, and {payload_mass}: the same
  // pair PublishGripperState already hands the in-JTC plugin. K8's z runs *down*
  // the chain, so a block hanging under the gripper has r_z > 0 (the envelope's
  // nominal is +1 m; grip_estimator writes d_contact + r_log).
  const auto outcome = run(
    "/crane/mpc/set_payload", true,
    R"(<SetMpcPayload service_name="/crane/mpc/set_payload" carries="true"
         mass="880.0" s_log_8_x="0.021" s_log_8_y="-0.071" s_log_8_z="1.04"/>
       <SetMpcPayload service_name="/crane/mpc/set_payload" carries="false"/>)");

  ASSERT_EQ(outcome.status, BT::NodeStatus::SUCCESS);
  ASSERT_EQ(outcome.requests.size(), 2u);

  EXPECT_EQ(outcome.requests[0].shape, Payload::SHAPE_BOX);
  EXPECT_DOUBLE_EQ(outcome.requests[0].mass, 880.0);
  EXPECT_DOUBLE_EQ(outcome.requests[0].com.x, 0.021);
  EXPECT_DOUBLE_EQ(outcome.requests[0].com.y, -0.071);
  EXPECT_DOUBLE_EQ(outcome.requests[0].com.z, 1.04);

  // SHAPE_NONE is what crane_mpc reads as the empty hook; a zero mass alone
  // would be refused as "not a light load".
  EXPECT_EQ(outcome.requests[1].shape, Payload::SHAPE_NONE);
  EXPECT_DOUBLE_EQ(outcome.requests[1].mass, 0.0);
}

TEST(SetMpcPayload, substitutes_a_centre_of_mass_the_grasp_never_wrote)
{
  // s_log_8_z sits at NaN on the blackboard until CaptureBlockGraspOffset runs.
  // crane_mpc refuses a CoM that is not finite on all three axes, and a refusal
  // here would abort the pick with the block already in the jaws.
  const auto outcome = run(
    "/crane/mpc/set_payload_nan", true,
    R"(<SetMpcPayload service_name="/crane/mpc/set_payload_nan" carries="true"
         mass="880.0" s_log_8_z="nan"/>)");

  ASSERT_EQ(outcome.status, BT::NodeStatus::SUCCESS);
  ASSERT_EQ(outcome.requests.size(), 1u);
  EXPECT_DOUBLE_EQ(outcome.requests[0].com.z, 0.0);
}

TEST(SetMpcPayload, refuses_a_carrying_move_the_mpc_would_solve_for_an_empty_hook)
{
  const auto outcome = run(
    "/crane/mpc/set_payload_zero", true,
    R"(<SetMpcPayload service_name="/crane/mpc/set_payload_zero" carries="true"
         mass="0.0"/>)");

  EXPECT_EQ(outcome.status, BT::NodeStatus::FAILURE);
  EXPECT_TRUE(outcome.requests.empty());
}

TEST(SetMpcPayload, does_not_fail_a_pick_on_a_profile_that_runs_no_mpc)
{
  // controller:=pid is the default and starts no crane_mpc, yet it runs these
  // same subtrees. A nav2 BtServiceNode would have thrown during construction.
  const auto outcome = run(
    "/crane/mpc/set_payload_absent", false,
    R"(<SetMpcPayload service_name="/crane/mpc/set_payload_absent" carries="true"
         mass="880.0"/>)");

  EXPECT_EQ(outcome.status, BT::NodeStatus::SUCCESS);
}

TEST(SetMpcPayload, stops_the_pick_when_the_mpc_that_drives_the_crane_is_not_there)
{
  // Same absent server, cbs_mpc_active. Here the MPC would command the lift
  // against the payload it last accepted -- an empty hook -- which is 174's one
  // failing corner. Carrying on would be the silent version of it.
  const auto outcome = run(
    "/crane/mpc/set_payload_absent", false,
    R"(<SetMpcPayload service_name="/crane/mpc/set_payload_absent" carries="true"
         mass="880.0"/>)", true);

  EXPECT_EQ(outcome.status, BT::NodeStatus::FAILURE);
}

TEST(SetMpcPayload, treats_a_refusal_the_same_way_as_an_absent_server)
{
  // crane_mpc refuses until a robot description has reached it. Which of the two
  // outages a cold start produces is a race, so they must not decide opposite
  // things about the same lift.
  const auto body =
    R"(<SetMpcPayload service_name="/crane/mpc/set_payload_refused" carries="true"
         mass="880.0"/>)";

  EXPECT_EQ(
    run("/crane/mpc/set_payload_refused", true, body, false, false).status,
    BT::NodeStatus::SUCCESS);
  EXPECT_EQ(
    run("/crane/mpc/set_payload_refused", true, body, true, false).status,
    BT::NodeStatus::FAILURE);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
