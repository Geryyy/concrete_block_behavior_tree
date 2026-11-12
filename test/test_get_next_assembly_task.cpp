// Issue 303: on 11-09 /tf froze, the pose transform failed, and the tree's discovery branch
// read a pickup this node never wrote. plan_ok is the promise that it did write them, so it
// may only go true once every pose is on the blackboard.

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "behaviortree_cpp/bt_factory.h"
#include "concrete_block_behavior_tree/plugins/action/get_next_assembly_task.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

using concrete_block_assembly_interfaces::srv::GetNextAssemblyTask;
using concrete_block_behavior_tree::GetNextAssemblyTaskService;

namespace
{

// One task, all poses in `world`, so every one of them has to be transformed.
void answer(
  const std::shared_ptr<GetNextAssemblyTask::Request>,
  std::shared_ptr<GetNextAssemblyTask::Response> response)
{
  response->success = true;
  response->has_task = true;
  response->task_id = "task_0";
  response->target_block_id = "block_0";
  for (auto * pose : {&response->pickup_pose, &response->pickup_approach_pose,
      &response->target_pose, &response->reference_pose, &response->approach_pose})
  {
    pose->header.frame_id = "world";
    pose->pose.position.x = 2.0;
    pose->pose.orientation.w = 1.0;
  }
  response->message = "ok";
}

BT::Blackboard::Ptr blackboard_for(const rclcpp::Node::SharedPtr & node)
{
  auto blackboard = BT::Blackboard::create();
  blackboard->set<rclcpp::Node::SharedPtr>("node", node);
  blackboard->set<std::chrono::milliseconds>("bt_loop_duration", std::chrono::milliseconds(100));
  blackboard->set<std::chrono::milliseconds>("server_timeout", std::chrono::milliseconds(2000));
  blackboard->set<std::chrono::milliseconds>(
    "wait_for_service_timeout", std::chrono::milliseconds(2000));
  return blackboard;
}

std::string tree_text(const std::string & planning_frame)
{
  return R"(<root BTCPP_format="4"><BehaviorTree ID="T">
       <GetNextAssemblyTask service_name="get_next_assembly_task" wall_plan_name="w"
         planning_frame=")" + planning_frame + R"(" plan_ok="{plan_ok}"
         plan_has_task="{plan_has_task}" plan_message="{plan_message}"
         pickup_x="{pickup_x}"/>
     </BehaviorTree></root>)";
}

}  // namespace

TEST(GetNextAssemblyTask, a_pose_it_could_not_transform_leaves_plan_ok_false)
{
  auto server_node = std::make_shared<rclcpp::Node>("test_next_task_server");
  auto service = server_node->create_service<GetNextAssemblyTask>("get_next_assembly_task", answer);
  tf2_ros::StaticTransformBroadcaster broadcaster(server_node);
  geometry_msgs::msg::TransformStamped world_k0;
  world_k0.header.frame_id = "world";
  world_k0.child_frame_id = "K0_mounting_base";
  world_k0.transform.translation.z = 1.1407;   // pzs100 on the truck bed
  world_k0.transform.rotation.w = 1.0;
  broadcaster.sendTransform(world_k0);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node);
  std::thread spinner([&executor]() {executor.spin();});

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<GetNextAssemblyTaskService>("GetNextAssemblyTask");

  auto node = std::make_shared<rclcpp::Node>("test_next_task");
  auto reachable = blackboard_for(node);
  auto ok_tree = factory.createTreeFromText(tree_text("K0_mounting_base"), reachable);
  const BT::NodeStatus answered = ok_tree.tickWhileRunning();

  // Same answer, a frame nothing publishes: what a frozen /tf does to the lookup.
  auto frozen = blackboard_for(node);
  auto broken_tree = factory.createTreeFromText(tree_text("no_such_frame"), frozen);
  const BT::NodeStatus not_transformed = broken_tree.tickWhileRunning();

  executor.cancel();
  spinner.join();

  EXPECT_EQ(answered, BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(reachable->get<bool>("plan_ok"));
  EXPECT_NEAR(reachable->get<double>("pickup_x"), 2.0, 1e-9);

  EXPECT_EQ(not_transformed, BT::NodeStatus::FAILURE);
  // False, so the tree's discovery branch stays out of the error path, and true so that
  // PlanComplete reads this as a failed task and not as a finished wall.
  EXPECT_FALSE(frozen->get<bool>("plan_ok"));
  EXPECT_TRUE(frozen->get<bool>("plan_has_task"));
}

// Issue 327: a server with no plan loaded (restarted, or never activated) answered
// success=false has_task=false, and PlanComplete turned that into a SUCCEEDED wall.
TEST(GetNextAssemblyTask, no_active_plan_is_a_failed_task_not_a_finished_wall)
{
  auto server_node = std::make_shared<rclcpp::Node>("test_no_plan_server");
  auto service = server_node->create_service<GetNextAssemblyTask>(
    "get_next_assembly_task",
    [](const std::shared_ptr<GetNextAssemblyTask::Request>,
    std::shared_ptr<GetNextAssemblyTask::Response> response) {
      response->success = false;
      response->has_task = false;
      response->message = "No active wall plan.";
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node);
  std::thread spinner([&executor]() {executor.spin();});

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<GetNextAssemblyTaskService>("GetNextAssemblyTask");
  auto node = std::make_shared<rclcpp::Node>("test_no_plan");
  auto blackboard = blackboard_for(node);
  auto tree = factory.createTreeFromText(tree_text("K0_mounting_base"), blackboard);
  const BT::NodeStatus status = tree.tickWhileRunning();

  executor.cancel();
  spinner.join();

  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
  EXPECT_TRUE(blackboard->get<bool>("plan_has_task"));
  EXPECT_FALSE(blackboard->get<bool>("plan_ok"));  // no discovery move on it either
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
