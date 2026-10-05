// Where the carried block hangs: gripper TF (pendulum joints included) times the grasp
// offset, the estimate WriteBlockPoseFromGripper writes, without writing it. Averaged over
// `window_s`: one pendulum period puts the mean at the swing's centre, where one sample read
// 0.5 cm off a block swinging +-3 cm (sim run 54).

#include <algorithm>
#include <cmath>
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

class MeasureBlockFromGripper : public BT::StatefulActionNode
{
public:
  MeasureBlockFromGripper(const std::string & name, const BT::NodeConfig & conf)
  : BT::StatefulActionNode(name, conf)
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
      BT::InputPort<double>("window_s", 0.0, "Average over this long; 0 is one sample"),
      BT::OutputPort<double>("x", "Block centre [m]"),
      BT::OutputPort<double>("y", "Block centre [m]"),
      BT::OutputPort<double>("z", "Block centre [m]"),
      BT::OutputPort<double>("tilt", "Block's vertical axis off the frame's z [rad]"),
    };
  }

  BT::NodeStatus onStart() override
  {
    window_ = 0.0;
    getInput("window_s", window_);
    sum_ = tf2::Vector3(0.0, 0.0, 0.0);
    tilt_sum_ = 0.0;
    samples_ = 0;
    start_ = node_->now();
    return sample() ? onRunning() : BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus onRunning() override
  {
    if ((node_->now() - start_).seconds() < window_) {
      return sample() ? BT::NodeStatus::RUNNING : BT::NodeStatus::FAILURE;
    }
    const tf2::Vector3 p = sum_ / static_cast<double>(samples_);
    setOutput("x", p.x());
    setOutput("y", p.y());
    setOutput("z", p.z());
    const double tilt = tilt_sum_ / static_cast<double>(samples_);
    setOutput("tilt", tilt);
    RCLCPP_INFO(
      node_->get_logger(),
      "MeasureBlockFromGripper: block at (%.3f, %.3f, %.3f) in %s, tilt %.2f deg, %d samples",
      p.x(), p.y(), p.z(), frame_.c_str(), tilt * 180.0 / M_PI, samples_);
    return BT::NodeStatus::SUCCESS;
  }

  void onHalted() override {}

private:
  // One TF sample into the sum; false on a missing offset or transform.
  bool sample()
  {
    std::string gripper_frame;
    geometry_msgs::msg::Pose offset;
    getInput("gripper_frame", gripper_frame);
    getInput("frame", frame_);
    if (!getInput("grasp_offset", offset)) {
      RCLCPP_ERROR(node_->get_logger(), "MeasureBlockFromGripper: no grasp_offset");
      return false;
    }
    geometry_msgs::msg::TransformStamped tcp;
    try {
      tcp = tf_buffer_->lookupTransform(
        frame_, gripper_frame, tf2::TimePointZero, tf2::durationFromSec(1.0));
    } catch (const tf2::TransformException & e) {
      RCLCPP_ERROR(node_->get_logger(), "MeasureBlockFromGripper: %s", e.what());
      return false;
    }
    tf2::Transform frame_tcp, tcp_block;
    tf2::fromMsg(tcp.transform, frame_tcp);
    tf2::fromMsg(offset, tcp_block);
    const tf2::Transform block = frame_tcp * tcp_block;
    sum_ += block.getOrigin();
    // Only as good as the grasp offset: FK cannot see a block clamped crooked (the gripper
    // hung within 1.2 deg while its block hung 9), a vision-refined offset can.
    tilt_sum_ += std::acos(std::clamp(std::abs(block.getBasis().getColumn(2).z()), 0.0, 1.0));
    ++samples_;
    return true;
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string frame_;
  double window_{0.0};
  tf2::Vector3 sum_;
  double tilt_sum_{0.0};
  int samples_{0};
  rclcpp::Time start_;
};

}  // namespace concrete_block_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<concrete_block_behavior_tree::MeasureBlockFromGripper>(
    "MeasureBlockFromGripper");
}
