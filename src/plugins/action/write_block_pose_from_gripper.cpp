#include "concrete_block_behavior_tree/plugins/action/write_block_pose_from_gripper.hpp"

#include <stdexcept>

#include "behaviortree_cpp/bt_factory.h"
#include "concrete_block_behavior_tree/plugins/shared_tf_buffer.hpp"
#include "tf2/exceptions.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace concrete_block_behavior_tree
{

namespace
{
// Same bound as MeasureBlockFromGripper's: a TCP older than this is where the hook was.
constexpr double kMaxTcpAge = 0.2;
}  // namespace

WriteBlockPoseFromGripper::WriteBlockPoseFromGripper(
  const std::string & service_node_name,
  const BT::NodeConfig & conf)
: nav2_behavior_tree::BtServiceNode<ServiceT>(service_node_name, conf)
{
  tf_buffer_ = shared_tf_buffer(config().blackboard, node_, tf_listener_);
}

void WriteBlockPoseFromGripper::on_tick()
{
  std::string block_id;
  std::string gripper_frame;
  std::string world_frame;
  geometry_msgs::msg::Pose grasp_offset;
  int task_status = 3;   // TASK_PLACED
  int pose_status = 1;   // POSE_COARSE: this pose is FK x the grasp offset, not a measurement
  double confidence = 0.5;

  getInput("block_id", block_id);
  getInput("gripper_frame", gripper_frame);
  getInput("world_frame", world_frame);
  if (!getInput("grasp_offset", grasp_offset)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "WriteBlockPoseFromGripper: missing required input 'grasp_offset'");
    should_send_request_ = false;
    return;
  }
  getInput("task_status", task_status);
  getInput("pose_status", pose_status);
  getInput("confidence", confidence);

  if (block_id.empty()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "WriteBlockPoseFromGripper: 'block_id' input is empty");
    should_send_request_ = false;
    return;
  }

  geometry_msgs::msg::TransformStamped world_tcp_tf;
  try {
    world_tcp_tf = tf_buffer_->lookupTransform(
      world_frame, gripper_frame, tf2::TimePointZero,
      tf2::durationFromSec(1.0));
  } catch (const tf2::TransformException & e) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "WriteBlockPoseFromGripper: tf lookup '%s' <- '%s' failed: %s",
      world_frame.c_str(), gripper_frame.c_str(), e.what());
    should_send_request_ = false;
    return;
  }
  // This write is the placed block's pose for the rest of the run; a stale TCP would put it
  // where the hook was before the /tf stall (1.5 s on 11-09), and nothing measures it again
  // until the post-place refine has to match it within 0.35 m.
  const double age = (node_->now() - rclcpp::Time(world_tcp_tf.header.stamp)).seconds();
  if (age > kMaxTcpAge) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "WriteBlockPoseFromGripper: '%s' is %.2f s old, so where the block was let go is "
      "unknown; not writing it", gripper_frame.c_str(), age);
    should_send_request_ = false;
    return;
  }

  tf2::Transform world_tcp;
  tf2::fromMsg(world_tcp_tf.transform, world_tcp);

  tf2::Transform tcp_block(
    tf2::Quaternion(
      grasp_offset.orientation.x, grasp_offset.orientation.y,
      grasp_offset.orientation.z, grasp_offset.orientation.w),
    tf2::Vector3(
      grasp_offset.position.x, grasp_offset.position.y,
      grasp_offset.position.z));

  const tf2::Transform world_block = world_tcp * tcp_block;

  request_->block_id = block_id;
  request_->frame_id = world_frame;
  request_->pose.position.x = world_block.getOrigin().x();
  request_->pose.position.y = world_block.getOrigin().y();
  request_->pose.position.z = world_block.getOrigin().z();
  const tf2::Quaternion q = world_block.getRotation();
  request_->pose.orientation.x = q.x();
  request_->pose.orientation.y = q.y();
  request_->pose.orientation.z = q.z();
  request_->pose.orientation.w = q.w();
  setOutput("x", request_->pose.position.x);
  setOutput("y", request_->pose.position.y);
  setOutput("z", request_->pose.position.z);
  request_->pose_status = pose_status;
  request_->task_status = task_status;
  request_->confidence = static_cast<float>(confidence);

  RCLCPP_INFO(
    node_->get_logger(),
    "WriteBlockPoseFromGripper: block_id='%s' world_pose=(%.3f, %.3f, %.3f) "
    "task_status=%d pose_status=%d",
    block_id.c_str(),
    request_->pose.position.x, request_->pose.position.y, request_->pose.position.z,
    task_status, pose_status);
}

BT::NodeStatus WriteBlockPoseFromGripper::on_completion(std::shared_ptr<ResponseT> response)
{
  setOutput("status_ok", response->success);
  setOutput("status_message", response->message);
  if (!response->success) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "WriteBlockPoseFromGripper: UpsertBlock failed: %s",
      response->message.c_str());
  }
  return response->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace concrete_block_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<concrete_block_behavior_tree::WriteBlockPoseFromGripper>(
    "WriteBlockPoseFromGripper");
}
