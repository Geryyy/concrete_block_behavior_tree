#pragma once

#include <chrono>
#include <mutex>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "gazebo_grasp_plugin_ros/msg/gazebo_grasp_event.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace concrete_block_behavior_tree
{

// Decides whether the gripper has clamped on a block by watching the actuator
// effort published on /joint_states. Mirrors the forestry stack's
// CalcGripState approach (see epsilon_crane_behavior_tree), which does not
// rely on gazebo_grasp_fix's force-angle heuristic.
class CheckGripperEffort : public BT::StatefulActionNode
{
public:
  CheckGripperEffort(const std::string & name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic", "/joint_states", "Joint states topic"),
      BT::InputPort<std::string>("joint", "q9_left_rail_joint", "Gripper actuator joint name"),
      BT::InputPort<double>("min_effort", 1000.0, "Minimum |effort| to consider grip successful"),
      BT::InputPort<int>("timeout_ms", 5000, "Maximum wait time in milliseconds"),
      BT::InputPort<std::string>("grasp_command_topic", "/grasp_command",
        "Topic to publish a force-attach command on grip success"),
      BT::InputPort<std::string>("arm", "gripper",
        "Arm name to publish in the grasp_command message"),
      BT::InputPort<std::string>("object", "",
        "Object name to publish in the grasp_command message (empty = skip publish)"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using JointState = sensor_msgs::msg::JointState;

  void callback(const JointState::SharedPtr msg);
  void publishAttachCommand();

  using GraspEvent = gazebo_grasp_plugin_ros::msg::GazeboGraspEvent;

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<JointState>::SharedPtr sub_;
  rclcpp::Publisher<GraspEvent>::SharedPtr command_pub_;

  std::mutex mutex_;
  bool have_sample_{false};
  double last_effort_{0.0};

  std::string topic_;
  std::string joint_;
  double min_effort_{1000.0};
  int timeout_ms_{5000};
  std::chrono::steady_clock::time_point start_time_;
};

}  // namespace concrete_block_behavior_tree
