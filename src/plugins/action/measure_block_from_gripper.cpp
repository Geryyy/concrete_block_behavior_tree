// Where the carried block is now: gripper TF (pendulum joints included) times the grasp
// offset, the estimate WriteBlockPoseFromGripper writes, without writing it. The place
// descent corrects its final segment by it: the actuated joints reach the plan, the passive
// pendulum does not hang where the plan assumes (2-4 cm at the block, sim runs 47-49).

#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace concrete_block_behavior_tree
{

class MeasureBlockFromGripper : public BT::SyncActionNode
{
public:
  MeasureBlockFromGripper(const std::string & name, const BT::NodeConfig & conf)
  : BT::SyncActionNode(name, conf)
  {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("gripper_frame", "K8_tool_center_point", "Gripper TCP frame"),
      BT::InputPort<std::string>("frame", "K0_mounting_base", "Frame of the outputs"),
      BT::InputPort<geometry_msgs::msg::Pose>(
        "grasp_offset", "Offset gripper_tcp <- block_center (from CaptureBlockGraspOffset)"),
      BT::OutputPort<double>("x", "Block centre [m]"),
      BT::OutputPort<double>("y", "Block centre [m]"),
      BT::OutputPort<double>("z", "Block centre [m]"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::string gripper_frame, frame;
    geometry_msgs::msg::Pose offset;
    getInput("gripper_frame", gripper_frame);
    getInput("frame", frame);
    if (!getInput("grasp_offset", offset)) {
      RCLCPP_ERROR(node_->get_logger(), "MeasureBlockFromGripper: no grasp_offset");
      return BT::NodeStatus::FAILURE;
    }
    geometry_msgs::msg::TransformStamped tcp;
    try {
      tcp = tf_buffer_->lookupTransform(
        frame, gripper_frame, tf2::TimePointZero, tf2::durationFromSec(1.0));
    } catch (const tf2::TransformException & e) {
      RCLCPP_ERROR(node_->get_logger(), "MeasureBlockFromGripper: %s", e.what());
      return BT::NodeStatus::FAILURE;
    }
    tf2::Transform frame_tcp, tcp_block;
    tf2::fromMsg(tcp.transform, frame_tcp);
    tf2::fromMsg(offset, tcp_block);
    const tf2::Vector3 p = (frame_tcp * tcp_block).getOrigin();
    setOutput("x", p.x());
    setOutput("y", p.y());
    setOutput("z", p.z());
    RCLCPP_INFO(
      node_->get_logger(), "MeasureBlockFromGripper: block at (%.3f, %.3f, %.3f) in %s",
      p.x(), p.y(), p.z(), frame.c_str());
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace concrete_block_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<concrete_block_behavior_tree::MeasureBlockFromGripper>(
    "MeasureBlockFromGripper");
}
