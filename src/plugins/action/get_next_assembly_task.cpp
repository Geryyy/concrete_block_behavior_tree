#include "concrete_block_behavior_tree/plugins/action/get_next_assembly_task.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace concrete_block_behavior_tree
{

namespace
{

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace

void GetNextAssemblyTaskService::on_tick()
{
  getInput("wall_plan_name", request_->wall_plan_name);
  getInput("reset_plan", request_->reset_plan);
  RCLCPP_INFO(
    node_->get_logger(),
    "Requesting next assembly task | wall_plan=%s reset_plan=%s",
    request_->wall_plan_name.c_str(),
    request_->reset_plan ? "true" : "false");
}

BT::NodeStatus GetNextAssemblyTaskService::on_completion(std::shared_ptr<ResponseT> response)
{
  // Resolve the assembly poses (emitted in `world`) into the crane planning
  // frame. This is the single world -> K0_mounting_base conversion boundary;
  // upstream (plan, world model, visualization) stays in `world`.
  std::string planning_frame = "K0_mounting_base";
  getInput("planning_frame", planning_frame);

  auto to_planning = [&](const geometry_msgs::msg::PoseStamped & in,
      geometry_msgs::msg::PoseStamped & out, const char * label) -> bool
    {
      if (in.header.frame_id.empty() || in.header.frame_id == planning_frame) {
        out = in;
        return true;
      }
      try {
        out = tf_buffer_->transform(in, planning_frame, tf2::durationFromSec(0.5));
        return true;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR(
          node_->get_logger(),
          "GetNextAssemblyTask: cannot transform %s from '%s' to '%s': %s",
          label, in.header.frame_id.c_str(), planning_frame.c_str(), ex.what());
        return false;
      }
    };

  geometry_msgs::msg::PoseStamped pickup, pickup_approach, target, reference, approach;
  if (response->has_task) {
    if (!to_planning(response->pickup_pose, pickup, "pickup_pose") ||
      !to_planning(response->pickup_approach_pose, pickup_approach, "pickup_approach_pose") ||
      !to_planning(response->target_pose, target, "target_pose") ||
      !to_planning(response->reference_pose, reference, "reference_pose") ||
      !to_planning(response->approach_pose, approach, "approach_pose"))
    {
      return BT::NodeStatus::FAILURE;
    }
  } else {
    pickup = response->pickup_pose;
    pickup_approach = response->pickup_approach_pose;
    target = response->target_pose;
    reference = response->reference_pose;
    approach = response->approach_pose;
  }

  setOutput("task_id", response->task_id);
  setOutput("target_block_id", response->target_block_id);
  setOutput("reference_block_id", response->reference_block_id);
  setOutput("target_block_pose_coarse", pickup);
  setOutput("target_block_pose_precise", target);
  setOutput("reference_block_pose_precise", reference);
  setOutput("placement_approach_pose", approach);
  setOutput("plan_has_task", response->has_task);
  setOutput("plan_message", response->message);

  // Decomposed scalar ports for epsilon_crane BT nodes (planning frame).
  setOutput("pickup_x", pickup.pose.position.x);
  setOutput("pickup_y", pickup.pose.position.y);
  setOutput("pickup_z", pickup.pose.position.z);
  setOutput("pickup_yaw", yawFromQuaternion(pickup.pose.orientation));
  setOutput("pickup_approach_z", pickup_approach.pose.position.z);
  setOutput("place_x", target.pose.position.x);
  setOutput("place_y", target.pose.position.y);
  setOutput("place_z", target.pose.position.z);
  setOutput("place_yaw", yawFromQuaternion(target.pose.orientation));
  setOutput("approach_x", approach.pose.position.x);
  setOutput("approach_y", approach.pose.position.y);
  setOutput("approach_z", approach.pose.position.z);

  bool disturbance_enabled = false;
  double disturbance_x = 0.0;
  double disturbance_y = 0.0;
  double disturbance_z = 0.0;
  double disturbance_yaw = 0.0;
  node_->get_parameter("simulated_placement_disturbance.enabled", disturbance_enabled);
  node_->get_parameter("simulated_placement_disturbance.x_m", disturbance_x);
  node_->get_parameter("simulated_placement_disturbance.y_m", disturbance_y);
  node_->get_parameter("simulated_placement_disturbance.z_m", disturbance_z);
  node_->get_parameter("simulated_placement_disturbance.yaw_rad", disturbance_yaw);
  setOutput("disturbed_approach_x", approach.pose.position.x + (disturbance_enabled ? disturbance_x : 0.0));
  setOutput("disturbed_approach_y", approach.pose.position.y + (disturbance_enabled ? disturbance_y : 0.0));
  setOutput("disturbed_approach_z", approach.pose.position.z + (disturbance_enabled ? disturbance_z : 0.0));
  setOutput(
    "disturbed_approach_yaw",
    yawFromQuaternion(approach.pose.orientation) + (disturbance_enabled ? disturbance_yaw : 0.0));
  if (disturbance_enabled) {
    RCLCPP_WARN(
      node_->get_logger(),
      "SIMULATION A2B goal bias: dxyz=[%.3f, %.3f, %.3f] m dyaw=%.3f rad",
      disturbance_x, disturbance_y, disturbance_z, disturbance_yaw);
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "GetNextAssemblyTask response | success=%s has_task=%s task_id=%s target=%s reference=%s pickup=(%.2f,%.2f,%.2f) place=(%.2f,%.2f,%.2f) message=%s",
    response->success ? "true" : "false",
    response->has_task ? "true" : "false",
    response->task_id.c_str(),
    response->target_block_id.c_str(),
    response->reference_block_id.c_str(),
    pickup.pose.position.x,
    pickup.pose.position.y,
    pickup.pose.position.z,
    target.pose.position.x,
    target.pose.position.y,
    target.pose.position.z,
    response->message.c_str());

  if (!response->success || !response->has_task) {
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace concrete_block_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<concrete_block_behavior_tree::GetNextAssemblyTaskService>(
    "GetNextAssemblyTask");
}
