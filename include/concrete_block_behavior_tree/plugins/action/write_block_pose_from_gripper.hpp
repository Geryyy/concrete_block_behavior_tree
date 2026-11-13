#pragma once

#include <memory>
#include <string>

#include "concrete_block_world_model_interfaces/srv/upsert_block.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav2_behavior_tree/bt_service_node.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace concrete_block_behavior_tree
{

// Write a block's world pose to the world_model by composing the current
// gripper TCP pose (tf2) with a previously captured offset
// (gripper_tcp <- block_center). Calls UpsertBlock with the resulting pose
// and the requested task/pose status. Use on release to capture the placed
// pose just before opening the gripper.
class WriteBlockPoseFromGripper
  : public nav2_behavior_tree::BtServiceNode<
    concrete_block_world_model_interfaces::srv::UpsertBlock>
{
public:
  using ServiceT = concrete_block_world_model_interfaces::srv::UpsertBlock;
  using ResponseT = ServiceT::Response;

  WriteBlockPoseFromGripper(
    const std::string & service_node_name,
    const BT::NodeConfig & conf);

  void on_tick() override;
  BT::NodeStatus on_completion(std::shared_ptr<ResponseT> response) override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<std::string>("block_id", "Block ID to update"),
        BT::InputPort<std::string>(
          "gripper_frame", "K8_tool_center_point",
          "TF frame of the gripper TCP (on /tf; perception's elastic/ prefix lives on "
          "tf_elastic_full)"),
        BT::InputPort<std::string>(
          "world_frame", "world",
          "Frame in which to write the block pose"),
        BT::InputPort<geometry_msgs::msg::Pose>(
          "grasp_offset",
          "Offset gripper_tcp <- block_center (from CaptureBlockGraspOffset)"),
        BT::InputPort<int>("task_status", 3, "Task status int (default TASK_PLACED=3)"),
        BT::InputPort<int>("pose_status", 1, "Pose status int (default POSE_COARSE=1)"),
        // 0.5, not 1: FK x the grasp offset is not a measurement (303). UpsertBlock stores it
        // as given - min_update_confidence gates incoming detections, not this.
        BT::InputPort<double>("confidence", 0.5, "Confidence value [0,1]"),
        BT::OutputPort<bool>("status_ok"),
        BT::OutputPort<std::string>("status_message"),
        // The pose that went in, for the refine after it to be compared against (332).
        BT::OutputPort<double>("x", "Written block x in world_frame"),
        BT::OutputPort<double>("y", "Written block y in world_frame"),
        BT::OutputPort<double>("z", "Written block z in world_frame"),
      });
  }

private:
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace concrete_block_behavior_tree
