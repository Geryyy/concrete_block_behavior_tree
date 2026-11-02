// Where the carried block hangs: gripper TF (pendulum joints included) times the grasp
// offset, the estimate WriteBlockPoseFromGripper writes, without writing it. Averaged over
// `window_s`: one pendulum period puts the mean at the swing's centre, where one sample read
// 0.5 cm off a block swinging +-3 cm.

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "concrete_block_behavior_tree/status_text.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
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
    // Latched, so a panel attaching mid-run still sees the last measurement, and one publisher
    // shared by every instance, so it sees only that one and not each node's last in any order.
    static rclcpp::Publisher<std_msgs::msg::String>::SharedPtr shared;
    if (!shared) {
      shared = node_->create_publisher<std_msgs::msg::String>(
        "/cbs/status/inair", rclcpp::QoS(1).transient_local());
    }
    status_ = shared;
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("gripper_frame", "K8_tool_center_point", "Gripper TCP frame"),
      BT::InputPort<std::string>("frame", "K0_mounting_base", "Frame of the outputs"),
      BT::InputPort<geometry_msgs::msg::Pose>(
        "grasp_offset", "Offset gripper_tcp <- block_center (from CaptureBlockGraspOffset)"),
      BT::InputPort<double>("window_s", 0.0, "Average over this long; 0 is one sample"),
      BT::InputPort<bool>("report", true, "Publish /cbs/status/inair, an in-air pass"),
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
    axis_sum_ = tf2::Vector3(0.0, 0.0, 0.0);
    samples_ = 0;
    ++passes_;  // the correction loop re-ticks this node; nothing else counts the passes
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
    // Tilt of the mean block axis, not the mean of |tilt|: a swing cancels over the window
    // instead of reading as a crooked grip.
    const double tilt = std::acos(std::clamp(std::abs(axis_sum_.normalized().z()), 0.0, 1.0));
    setOutput("tilt", tilt);
    RCLCPP_INFO(
      node_->get_logger(),
      "MeasureBlockFromGripper: block at (%.3f, %.3f, %.3f) in %s, tilt %.2f deg, %d samples",
      p.x(), p.y(), p.z(), frame_.c_str(), tilt * 180.0 / M_PI, samples_);
    bool report = true;
    getInput("report", report);
    if (report) {
      publish_status(p, tilt);
    }
    return BT::NodeStatus::SUCCESS;
  }

  void onHalted() override {}

private:
  // Also in `world`: the task line's approach and slot are there, and `frame` is not
  // (K0_mounting_base is turned by pi and offset in the sim).
  void publish_status(const tf2::Vector3 & p, double tilt)
  {
    double world_xyz[3];
    bool in_world = false;
    try {
      tf2::Transform world_frame;
      tf2::fromMsg(
        tf_buffer_->lookupTransform("world", frame_, tf2::TimePointZero).transform, world_frame);
      const tf2::Vector3 w = world_frame * p;
      world_xyz[0] = w.x();
      world_xyz[1] = w.y();
      world_xyz[2] = w.z();
      in_world = true;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN(node_->get_logger(), "MeasureBlockFromGripper: no world pose: %s", e.what());
    }
    std_msgs::msg::String status;
    status.data = inair_status(
      passes_, frame_, p.x(), p.y(), p.z(), tilt, samples_, in_world ? world_xyz : nullptr);
    status_->publish(status);
  }

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
    // Block z-axis; its sign is arbitrary, so keep it pointing up before summing.
    const tf2::Vector3 axis = block.getBasis().getColumn(2);
    axis_sum_ += axis.z() < 0.0 ? -axis : axis;
    ++samples_;
    return true;
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  std::string frame_;
  double window_{0.0};
  tf2::Vector3 sum_;
  tf2::Vector3 axis_sum_;
  int samples_{0};
  int passes_{0};
  rclcpp::Time start_;
};

}  // namespace concrete_block_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<concrete_block_behavior_tree::MeasureBlockFromGripper>(
    "MeasureBlockFromGripper");
}
