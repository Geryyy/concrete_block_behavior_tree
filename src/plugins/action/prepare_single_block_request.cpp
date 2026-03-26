#include "concrete_block_behavior_tree/plugins/action/prepare_single_block_request.hpp"

#include <cmath>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace concrete_block_behavior_tree
{

PrepareSingleBlockRequest::PrepareSingleBlockRequest(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  logger_ = node_->get_logger().get_child("PrepareSingleBlockRequest");
  tf_buffer_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
}

geometry_msgs::msg::PoseStamped PrepareSingleBlockRequest::transform_pose(
  const geometry_msgs::msg::PoseStamped & pose,
  const std::string & target_frame) const
{
  if (pose.header.frame_id.empty() || pose.header.frame_id == target_frame) {
    auto transformed = pose;
    transformed.header.frame_id = target_frame;
    return transformed;
  }

  geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
    target_frame,
    pose.header.frame_id,
    tf2::TimePointZero,
    tf2::durationFromSec(1.0));
  geometry_msgs::msg::PoseStamped transformed;
  tf2::doTransform(pose, transformed, transform);
  transformed.header.frame_id = target_frame;
  return transformed;
}

BT::NodeStatus PrepareSingleBlockRequest::tick()
{
  geometry_msgs::msg::PoseStamped target_pose;
  std::string source_frame = "K8_tool_center_point";
  std::string target_frame;
  double approach_offset_m = 1.0;

  if (!getInput("target_pose", target_pose)) {
    RCLCPP_ERROR(logger_, "Missing required input port 'target_pose'");
    return BT::NodeStatus::FAILURE;
  }
  getInput("source_frame", source_frame);
  getInput("target_frame", target_frame);
  getInput("approach_offset_m", approach_offset_m);

  if (target_frame.empty()) {
    target_frame = target_pose.header.frame_id.empty() ? "world" : target_pose.header.frame_id;
  }

  geometry_msgs::msg::TransformStamped tool_tf;
  try {
    tool_tf = tf_buffer_->lookupTransform(
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
  start_pose.header = tool_tf.header;
  start_pose.pose.position.x = tool_tf.transform.translation.x;
  start_pose.pose.position.y = tool_tf.transform.translation.y;
  start_pose.pose.position.z = tool_tf.transform.translation.z;
  start_pose.pose.orientation = tool_tf.transform.rotation;

  geometry_msgs::msg::PoseStamped goal_pose;
  try {
    goal_pose = transform_pose(target_pose, target_frame);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      logger_,
      "Could not transform target pose from %s to %s: %s",
      target_pose.header.frame_id.c_str(),
      target_frame.c_str(),
      ex.what());
    return BT::NodeStatus::FAILURE;
  }

  const double q_norm = std::sqrt(
    goal_pose.pose.orientation.x * goal_pose.pose.orientation.x +
    goal_pose.pose.orientation.y * goal_pose.pose.orientation.y +
    goal_pose.pose.orientation.z * goal_pose.pose.orientation.z +
    goal_pose.pose.orientation.w * goal_pose.pose.orientation.w);
  if (q_norm < 1e-6) {
    goal_pose.pose.orientation.w = 1.0;
  }

  if (approach_offset_m > 1e-6) {
    const double dx = goal_pose.pose.position.x - start_pose.pose.position.x;
    const double dy = goal_pose.pose.position.y - start_pose.pose.position.y;
    const double dz = goal_pose.pose.position.z - start_pose.pose.position.z;
    const double norm = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (norm > 1e-6) {
      goal_pose.pose.position.x -= approach_offset_m * (dx / norm);
      goal_pose.pose.position.y -= approach_offset_m * (dy / norm);
      goal_pose.pose.position.z -= approach_offset_m * (dz / norm);
    }
  }

  setOutput("start_pose", start_pose);
  setOutput("goal_pose", goal_pose);

  RCLCPP_INFO(
    logger_,
    "Prepared single-block request in %s | start=(%.2f,%.2f,%.2f) goal=(%.2f,%.2f,%.2f) offset=%.2fm",
    target_frame.c_str(),
    start_pose.pose.position.x,
    start_pose.pose.position.y,
    start_pose.pose.position.z,
    goal_pose.pose.position.x,
    goal_pose.pose.position.y,
    goal_pose.pose.position.z,
    approach_offset_m);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace concrete_block_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<concrete_block_behavior_tree::PrepareSingleBlockRequest>(
    "PrepareSingleBlockRequest");
}
