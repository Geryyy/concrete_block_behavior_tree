#pragma once

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace concrete_block_behavior_tree
{

class PrepareSingleBlockRequest : public BT::SyncActionNode
{
public:
  PrepareSingleBlockRequest(const std::string & name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("target_pose"),
      BT::InputPort<std::string>("source_frame", "K8_tool_center_point"),
      BT::InputPort<std::string>("target_frame", ""),
      BT::InputPort<double>(
        "approach_offset_m", 1.0, "Pre-approach offset along the approach vector"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("start_pose"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal_pose"),
    };
  }

  BT::NodeStatus tick() override;

private:
  geometry_msgs::msg::PoseStamped transform_pose(
    const geometry_msgs::msg::PoseStamped & pose,
    const std::string & target_frame) const;

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Logger logger_{rclcpp::get_logger("PrepareSingleBlockRequest")};
};

}  // namespace concrete_block_behavior_tree
