#include "concrete_block_behavior_tree/plugins/action/capture_block_grasp_offset.hpp"

#include <algorithm>
#include <stdexcept>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "tf2/exceptions.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace concrete_block_behavior_tree
{

CaptureBlockGraspOffset::CaptureBlockGraspOffset(
  const std::string & service_node_name,
  const BT::NodeConfiguration & conf)
: nav2_behavior_tree::BtServiceNode<ServiceT>(service_node_name, conf)
{
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void CaptureBlockGraspOffset::on_tick()
{
  getInput("block_id", block_id_);
  getInput("gripper_frame", gripper_frame_);
  getInput("world_frame", world_frame_);

  if (block_id_.empty()) {
    throw std::invalid_argument("CaptureBlockGraspOffset: 'block_id' input is empty");
  }

  request_->force_refresh = false;
  request_->timeout_s = 1.0f;
}

BT::NodeStatus CaptureBlockGraspOffset::on_completion(std::shared_ptr<ResponseT> response)
{
  if (!response->success) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "CaptureBlockGraspOffset: GetCoarseBlocks failed: %s",
      response->message.c_str());
    return BT::NodeStatus::FAILURE;
  }

  const auto & blocks = response->blocks.blocks;
  const auto it = std::find_if(
    blocks.begin(), blocks.end(),
    [&](const auto & b) {return b.id == block_id_;});
  if (it == blocks.end()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "CaptureBlockGraspOffset: block_id='%s' not in world model (got %zu blocks)",
      block_id_.c_str(), blocks.size());
    return BT::NodeStatus::FAILURE;
  }
  const geometry_msgs::msg::Pose world_block_pose = it->pose;

  geometry_msgs::msg::TransformStamped world_tcp_tf;
  try {
    world_tcp_tf = tf_buffer_->lookupTransform(
      world_frame_, gripper_frame_, tf2::TimePointZero,
      tf2::durationFromSec(1.0));
  } catch (const tf2::TransformException & e) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "CaptureBlockGraspOffset: tf lookup '%s' <- '%s' failed: %s",
      world_frame_.c_str(), gripper_frame_.c_str(), e.what());
    return BT::NodeStatus::FAILURE;
  }

  tf2::Transform world_tcp;
  tf2::fromMsg(world_tcp_tf.transform, world_tcp);

  tf2::Transform world_block(
    tf2::Quaternion(
      world_block_pose.orientation.x, world_block_pose.orientation.y,
      world_block_pose.orientation.z, world_block_pose.orientation.w),
    tf2::Vector3(
      world_block_pose.position.x, world_block_pose.position.y,
      world_block_pose.position.z));

  const tf2::Transform tcp_block = world_tcp.inverse() * world_block;

  geometry_msgs::msg::Pose offset_pose;
  offset_pose.position.x = tcp_block.getOrigin().x();
  offset_pose.position.y = tcp_block.getOrigin().y();
  offset_pose.position.z = tcp_block.getOrigin().z();
  const tf2::Quaternion q = tcp_block.getRotation();
  offset_pose.orientation.x = q.x();
  offset_pose.orientation.y = q.y();
  offset_pose.orientation.z = q.z();
  offset_pose.orientation.w = q.w();

  setOutput("grasp_offset", offset_pose);
  setOutput("s_log_8_x", offset_pose.position.x);
  setOutput("s_log_8_y", offset_pose.position.y);
  setOutput("s_log_8_z", offset_pose.position.z);

  RCLCPP_INFO(
    node_->get_logger(),
    "CaptureBlockGraspOffset: block_id='%s' offset xyz=(%.3f, %.3f, %.3f)",
    block_id_.c_str(),
    offset_pose.position.x, offset_pose.position.y, offset_pose.position.z);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace concrete_block_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<concrete_block_behavior_tree::CaptureBlockGraspOffset>(
    "CaptureBlockGraspOffset");
}
