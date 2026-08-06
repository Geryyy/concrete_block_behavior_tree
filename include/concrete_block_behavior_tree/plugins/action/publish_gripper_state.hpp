#pragma once

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "epsilon_crane_control_interfaces/msg/gripper_state.hpp"
#include "rclcpp/rclcpp.hpp"

namespace concrete_block_behavior_tree
{

// Publish the current payload state to the low-level MPC.
//
// The A2B planner already receives the payload via the CalcA2BMovement service
// (carriesLog / m_log / logCarryingText / s_log_8_*), but the MPC controller
// plugin reads it from a separate `gripper_state` topic. Nothing in this stack
// publishes that topic, so the MPC models an empty gripper on every move.
//
// This node forwards the payload that is already on the blackboard, so planner
// and controller share one source of truth. Tick it immediately before each
// motion: the MPC subscribes with volatile QoS, so republishing per move keeps
// the payload correct across a controller restart.
class PublishGripperState : public BT::SyncActionNode
{
public:
  using GripperState = epsilon_crane_control_interfaces::msg::GripperState;

  PublishGripperState(const std::string & name, const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "topic", "/gripper_state",
        "Topic the MPC controller plugin subscribes to"),
      BT::InputPort<bool>(
        "carries", false,
        "Is a payload currently held? Feed the subtree's *_carries key"),
      BT::InputPort<double>("mass", 0.0, "Payload mass in kg"),
      BT::InputPort<std::string>(
        "log_shape_text", "",
        "Payload shape as {radius_top: <m>, radius_bottom: <m>, length: <m>}"),
      BT::InputPort<double>("s_log_8_x", 0.0, "Payload CoM x in K8 [m]"),
      BT::InputPort<double>("s_log_8_y", 0.0, "Payload CoM y in K8 [m]"),
      BT::InputPort<double>("s_log_8_z", 0.0, "Payload CoM z in K8 [m]"),
    };
  }

private:
  // Replace a non-finite value with 0.0 and warn. The MPC validates every
  // numeric field of GripperState up front and throws std::invalid_argument on
  // a NaN or inf -- from a call site that is not wrapped in a try/catch -- so a
  // stray NaN would take down the controller. s_log_8_z in particular defaults
  // to NaN on the blackboard until CaptureBlockGraspOffset has run.
  double sanitize(double value, const char * field_name);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<GripperState>::SharedPtr pub_;
  std::string topic_;
};

}  // namespace concrete_block_behavior_tree
