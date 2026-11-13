#pragma once

#include <memory>

#include <behaviortree_cpp/blackboard.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace concrete_block_behavior_tree
{

// The server's buffer; an own listener per tree node spins a thread that decodes all of /tf.
inline std::shared_ptr<tf2_ros::Buffer> shared_tf_buffer(
  const BT::Blackboard::Ptr & blackboard, const rclcpp::Node::SharedPtr & node,
  std::shared_ptr<tf2_ros::TransformListener> & own_listener)
{
  std::shared_ptr<tf2_ros::Buffer> buffer;
  if (blackboard->rootBlackboard()->get("tf_buffer", buffer) && buffer) {return buffer;}
  buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  own_listener = std::make_shared<tf2_ros::TransformListener>(*buffer);  // tests, no server
  return buffer;
}

}  // namespace concrete_block_behavior_tree
