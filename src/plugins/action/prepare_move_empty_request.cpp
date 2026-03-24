#include "concrete_block_behavior_tree/plugins/action/prepare_move_empty_request.hpp"

#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace concrete_block_behavior_tree
{

PrepareMoveEmptyRequest::PrepareMoveEmptyRequest(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  logger_ = node_->get_logger().get_child("PrepareMoveEmptyRequest");
  tf_buffer_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
}

BT::NodeStatus PrepareMoveEmptyRequest::tick()
{
  geometry_msgs::msg::Point goal_point;
  double phi = 0.0;
  std::string source_frame = "K8_tool_center_point";
  std::string target_frame = "K0_mounting_base";

  if (!getInput("goal", goal_point)) {
    RCLCPP_ERROR(logger_, "Missing required input port 'goal'");
    return BT::NodeStatus::FAILURE;
  }
  if (!getInput("phi", phi)) {
    RCLCPP_ERROR(logger_, "Missing required input port 'phi'");
    return BT::NodeStatus::FAILURE;
  }
  getInput("source_frame", source_frame);
  getInput("target_frame", target_frame);

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(
      target_frame,
      source_frame,
      tf2::TimePointZero,
      tf2::durationFromSec(1.0));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      logger_,
      "Could not transform %s to %s: %s",
      target_frame.c_str(),
      source_frame.c_str(),
      ex.what());
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::PoseStamped start_pose;
  start_pose.header = transform.header;
  start_pose.pose.position.x = transform.transform.translation.x;
  start_pose.pose.position.y = transform.transform.translation.y;
  start_pose.pose.position.z = transform.transform.translation.z;
  start_pose.pose.orientation = transform.transform.rotation;

  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header = start_pose.header;
  goal_pose.pose.position = goal_point;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, phi);
  goal_pose.pose.orientation = tf2::toMsg(q);

  setOutput("start_pose", start_pose);
  setOutput("goal_pose", goal_pose);

  RCLCPP_INFO(
    logger_,
    "Prepared move-empty request in %s | start=(%.2f,%.2f,%.2f) goal=(%.2f,%.2f,%.2f) phi=%.1fdeg",
    target_frame.c_str(),
    start_pose.pose.position.x,
    start_pose.pose.position.y,
    start_pose.pose.position.z,
    goal_pose.pose.position.x,
    goal_pose.pose.position.y,
    goal_pose.pose.position.z,
    phi * 180.0 / 3.14159265358979323846);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace concrete_block_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<concrete_block_behavior_tree::PrepareMoveEmptyRequest>(
    "PrepareMoveEmptyRequest");
}
