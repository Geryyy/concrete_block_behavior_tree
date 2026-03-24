#pragma once

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace concrete_block_behavior_tree
{

class PrepareMoveEmptyRequest : public BT::SyncActionNode
{
public:
  PrepareMoveEmptyRequest(const std::string & name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::Point>("goal"),
      BT::InputPort<double>("phi"),
      BT::InputPort<std::string>("source_frame", "K8_tool_center_point"),
      BT::InputPort<std::string>("target_frame", "K0_mounting_base"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("start_pose"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal_pose"),
    };
  }

private:
  BT::NodeStatus tick() override;

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Logger logger_{rclcpp::get_logger("PrepareMoveEmptyRequest")};
};

}  // namespace concrete_block_behavior_tree
