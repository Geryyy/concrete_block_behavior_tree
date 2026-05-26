#pragma once

#include <memory>
#include <string>

#include "concrete_block_world_model_interfaces/srv/get_coarse_blocks.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav2_behavior_tree/bt_service_node.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace concrete_block_behavior_tree
{

// Capture the static transform (gripper_tcp <- block_center) at grasp time.
// Reads the block's current world-model pose via GetCoarseBlocks, looks up the
// gripper TCP pose via tf2, composes the offset, and writes it to the
// blackboard as a geometry_msgs::Pose for later use by
// WriteBlockPoseFromGripper on release.
class CaptureBlockGraspOffset
  : public nav2_behavior_tree::BtServiceNode<
    concrete_block_world_model_interfaces::srv::GetCoarseBlocks>
{
public:
  using ServiceT = concrete_block_world_model_interfaces::srv::GetCoarseBlocks;
  using ResponseT = ServiceT::Response;

  CaptureBlockGraspOffset(
    const std::string & service_node_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  BT::NodeStatus on_completion(std::shared_ptr<ResponseT> response) override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<std::string>("block_id", "Block ID whose offset to capture"),
        BT::InputPort<std::string>(
          "gripper_frame", "K8_tool_center_point",
          "TF frame of the gripper TCP (on /tf; perception's elastic/ prefix lives on tf_elastic_full)"),
        BT::InputPort<std::string>(
          "world_frame", "K0_mounting_base",
          "World frame in which world_model block poses are expressed"),
        BT::OutputPort<geometry_msgs::msg::Pose>(
          "grasp_offset", "Offset gripper_tcp <- block_center as a Pose"),
      });
  }

private:
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string block_id_;
  std::string gripper_frame_;
  std::string world_frame_;
};

}  // namespace concrete_block_behavior_tree
