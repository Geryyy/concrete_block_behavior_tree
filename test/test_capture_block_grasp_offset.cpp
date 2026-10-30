// The block centre the planner and the MPC read as s_log_8 is in K8, not the TCP.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

#include "behaviortree_cpp/bt_factory.h"
#include "concrete_block_behavior_tree/plugins/action/capture_block_grasp_offset.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

using concrete_block_behavior_tree::CaptureBlockGraspOffset;
using concrete_block_world_model_interfaces::srv::GetCoarseBlocks;

namespace
{

geometry_msgs::msg::TransformStamped fixed(
  const std::string & parent, const std::string & child, double z, double qx, double qw)
{
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = parent;
  tf.child_frame_id = child;
  tf.transform.translation.z = z;
  tf.transform.rotation.x = qx;
  tf.transform.rotation.w = qw;
  return tf;
}

}  // namespace

TEST(CaptureBlockGraspOffset, writes_s_log_8_in_k8_and_the_grasp_offset_in_the_tcp)
{
  // K8 hangs z-down at the world origin; the block centre sits 1 m below it.
  auto server_node = std::make_shared<rclcpp::Node>("test_capture_grasp_server");
  auto service = server_node->create_service<GetCoarseBlocks>(
    "get_coarse_blocks",
    [](const std::shared_ptr<GetCoarseBlocks::Request>,
    std::shared_ptr<GetCoarseBlocks::Response> response) {
      response->success = true;
      response->blocks.blocks.resize(1);
      response->blocks.blocks[0].id = "b1";
      response->blocks.blocks[0].pose.position.z = -1.0;
      response->blocks.blocks[0].pose.orientation.x = 1.0;
      response->blocks.blocks[0].pose.orientation.w = 0.0;
    });
  tf2_ros::StaticTransformBroadcaster broadcaster(server_node);
  broadcaster.sendTransform(
    {fixed("world", "K8_rotator_lower_part", 0.0, 1.0, 0.0),
      fixed("K8_rotator_lower_part", "K8_tool_center_point", 0.0487, 0.0, 1.0)});  // pzs100.urdf
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node);
  std::thread spinner([&executor]() {executor.spin();});

  auto node = std::make_shared<rclcpp::Node>("test_capture_grasp");
  auto blackboard = BT::Blackboard::create();
  blackboard->set<rclcpp::Node::SharedPtr>("node", node);
  blackboard->set<std::chrono::milliseconds>("bt_loop_duration", std::chrono::milliseconds(100));
  blackboard->set<std::chrono::milliseconds>("server_timeout", std::chrono::milliseconds(2000));
  blackboard->set<std::chrono::milliseconds>(
    "wait_for_service_timeout", std::chrono::milliseconds(2000));

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<CaptureBlockGraspOffset>("CaptureBlockGraspOffset");
  auto tree = factory.createTreeFromText(
    R"(<root BTCPP_format="4"><BehaviorTree ID="T">
         <CaptureBlockGraspOffset service_name="get_coarse_blocks" block_id="b1"
           world_frame="world" grasp_offset="{grasp_offset}"
           s_log_8_x="{x}" s_log_8_y="{y}" s_log_8_z="{z}"/>
       </BehaviorTree></root>)",
    blackboard);
  const BT::NodeStatus status = tree.tickWhileRunning();

  executor.cancel();
  spinner.join();

  ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
  EXPECT_NEAR(blackboard->get<double>("x"), 0.0, 1e-9);
  EXPECT_NEAR(blackboard->get<double>("y"), 0.0, 1e-9);
  EXPECT_NEAR(blackboard->get<double>("z"), 1.0, 1e-9);
  EXPECT_NEAR(blackboard->get<geometry_msgs::msg::Pose>("grasp_offset").position.z, 0.9513, 1e-9);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
