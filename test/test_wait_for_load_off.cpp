// A /tf stall is not a landing, and it is not a reason to end the goal either.
//
// Framework review item 12: the gate tripped on one tick with the K8 transform older than
// 0.2 s and failed the descent, which leaves the block landed, clamped and TASK_MOVE with
// the operator to recover it by hand. The 11-09 LAN stalled /tf for 1.5 s. Up to 2 s it now
// waits -- the descent is the move's own, and the height checks it skips are exactly the
// ones a frozen transform answers wrongly -- and past 2 s it is frozen, not late.

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "behaviortree_cpp/bt_factory.h"
#include "concrete_block_behavior_tree/plugins/condition/wait_for_load_off.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"

using concrete_block_behavior_tree::WaitForLoadOff;
using sensor_msgs::msg::JointState;

namespace
{

// Run 11 of the lab day: 880 kg hanging at r = 5.12 m reads 111 kN m of boom effort, and the
// touchdown takes it to 55, well past the 0.8 m g r the tree asks for.
constexpr double kMass = 880.0;
constexpr double kHanging = 111000.0;
constexpr double kLanded = 20000.0;

constexpr char kTree[] =
  R"(<root BTCPP_format="4">
       <BehaviorTree ID="T">
         <WaitForLoadOff mass="880.0" target_z="1.0" drop_fraction="0.8"
           max_above_target="0.2" floor_below_target="0.25" timeout_ms="20000"
           frame="world" k8_frame="K8_tool_center_point" lever_frame="moved_with_q2"
           s_log_8_x="0.0" s_log_8_y="0.0" s_log_8_z="1.05" load_off="{load_off}"/>
       </BehaviorTree>
     </root>)";

// Drives the node with the block resting on its target and the hook unloaded, with every
// transform stamped `tf_age` seconds in the past. Returns the status it settles on, or
// RUNNING if it is still waiting after `ticks`.
BT::NodeStatus run(double tf_age, int ticks)
{
  auto node = std::make_shared<rclcpp::Node>("test_wait_for_load_off");
  auto state_pub = node->create_publisher<JointState>("/joint_states", rclcpp::QoS(1));
  tf2_ros::TransformBroadcaster tf(node);

  auto blackboard = BT::Blackboard::create();
  blackboard->set<rclcpp::Node::SharedPtr>("node", node);
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<WaitForLoadOff>("WaitForLoadOff");
  auto tree = factory.createTreeFromText(kTree, blackboard);

  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  for (int tick = 0; tick < ticks && status == BT::NodeStatus::RUNNING; ++tick) {
    const auto stamp = node->now() - rclcpp::Duration::from_seconds(tf_age);
    for (const std::string child : {"K8_tool_center_point", "moved_with_q2"}) {
      geometry_msgs::msg::TransformStamped frame;
      frame.header.stamp = stamp;
      frame.header.frame_id = "world";
      frame.child_frame_id = child;
      // K8 2.05 m up, so the block centre sits at its target; the pivot 5.12 m away.
      frame.transform.translation.x = child == "moved_with_q2" ? 5.12 : 0.0;
      frame.transform.translation.z = child == "moved_with_q2" ? 0.0 : 2.05;
      frame.transform.rotation.w = 1.0;
      tf.sendTransform(frame);
    }
    JointState state;
    state.name = {"theta2_boom_joint"};
    // The first 0.5 s is the hanging reference the drop is measured against.
    state.effort = {tick < 15 ? kHanging : kLanded};
    state_pub->publish(state);
    rclcpp::spin_some(node);
    status = tree.tickExactlyOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
  }
  return status;
}

}  // namespace

TEST(WaitForLoadOff, lands_through_a_fresh_transform)
{
  EXPECT_EQ(run(0.0, 60), BT::NodeStatus::SUCCESS);
}

TEST(WaitForLoadOff, waits_out_a_one_and_a_half_second_tf_stall)
{
  // The landing is there to be seen -- the fresh-transform case above lands on the same
  // numbers -- so a FAILURE here is the age check and nothing else. RUNNING is the pass:
  // the gate holds the descent open until /tf catches up.
  EXPECT_EQ(run(1.5, 30), BT::NodeStatus::RUNNING);
}

TEST(WaitForLoadOff, fails_on_a_transform_that_is_frozen_and_not_late)
{
  EXPECT_EQ(run(2.5, 30), BT::NodeStatus::FAILURE);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
