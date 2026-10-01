// Hand a grip trajectory to crane_mpc the way crane_planning hands it a move:
// /crane/reference plus its curve on /crane/joint_path, one stamp. The JTC played lift
// and descent open loop and nothing damped the swing they excited (sim, 2026-10-01).

#include <algorithm>
#include <sstream>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "crane_msgs/msg/joint_path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace concrete_block_behavior_tree
{

class PublishMpcReference : public BT::SyncActionNode
{
public:
  PublishMpcReference(const std::string & name, const BT::NodeConfig & conf)
  : BT::SyncActionNode(name, conf)
  {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    // Latched as crane_planning's: the MPC may subscribe after the tree published.
    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    reference_pub_ = node_->create_publisher<Trajectory>("/crane/reference", qos);
    path_pub_ = node_->create_publisher<crane_msgs::msg::JointPath>("/crane/joint_path", qos);
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<Trajectory>("trajectory", "Grip trajectory to drive"),
      BT::InputPort<std::string>(
        "path_joints",
        "theta1_slewing_joint,theta2_boom_joint,theta3_arm_joint,q4_big_telescope,"
        "theta8_rotator_joint",
        "The planned columns crane_mpc reads off the path"),
    };
  }

  BT::NodeStatus tick() override
  {
    Trajectory trajectory;
    std::string joints_text;
    if (!getInput("trajectory", trajectory) || trajectory.points.size() < 2) {
      RCLCPP_ERROR(node_->get_logger(), "PublishMpcReference: no trajectory to drive");
      return BT::NodeStatus::FAILURE;
    }
    getInput("path_joints", joints_text);

    std::vector<size_t> columns;
    std::stringstream stream(joints_text);
    std::string joint;
    crane_msgs::msg::JointPath path;
    while (std::getline(stream, joint, ',')) {
      const auto & names = trajectory.joint_names;
      const auto it = std::find(names.begin(), names.end(), joint);
      if (it == trajectory.joint_names.end()) {
        RCLCPP_ERROR(node_->get_logger(), "PublishMpcReference: no %s", joint.c_str());
        return BT::NodeStatus::FAILURE;
      }
      columns.push_back(static_cast<size_t>(std::distance(names.begin(), it)));
      path.joint_names.push_back(joint);
    }

    // The curve uniform in time: a parametrisation of the same geometry, which is
    // all `path_control` fits. Linear between points, 10 ms apart.
    const double total = rclcpp::Duration(trajectory.points.back().time_from_start).seconds();
    size_t k = 0;
    for (size_t row = 0; row < kRows; ++row) {
      const double t = total * static_cast<double>(row) / (kRows - 1);
      while (k + 2 < trajectory.points.size() &&
        rclcpp::Duration(trajectory.points[k + 1].time_from_start).seconds() < t)
      {
        ++k;
      }
      const auto & a = trajectory.points[k];
      const auto & b = trajectory.points[k + 1];
      const double ta = rclcpp::Duration(a.time_from_start).seconds();
      const double tb = rclcpp::Duration(b.time_from_start).seconds();
      const double w = tb > ta ? std::clamp((t - ta) / (tb - ta), 0.0, 1.0) : 0.0;
      for (const auto column : columns) {
        const double from = a.positions[column];
        path.q_path.push_back(from + w * (b.positions[column] - from));
      }
    }
    path.duration = trajectory.points.back().time_from_start;

    const auto stamp = node_->now();
    trajectory.header.stamp = stamp;
    path.header.stamp = stamp;
    reference_pub_->publish(trajectory);
    path_pub_->publish(path);
    RCLCPP_INFO(
      node_->get_logger(), "PublishMpcReference: %.2f s, %zu points to crane_mpc", total,
      trajectory.points.size());
    return BT::NodeStatus::SUCCESS;
  }

private:
  using Trajectory = trajectory_msgs::msg::JointTrajectory;
  static constexpr size_t kRows = 100;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<Trajectory>::SharedPtr reference_pub_;
  rclcpp::Publisher<crane_msgs::msg::JointPath>::SharedPtr path_pub_;
};

}  // namespace concrete_block_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<concrete_block_behavior_tree::PublishMpcReference>(
    "PublishMpcReference");
}
