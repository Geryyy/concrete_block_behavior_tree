#pragma once

#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "crane_msgs/srv/set_payload.hpp"
#include "rclcpp/rclcpp.hpp"

namespace concrete_block_behavior_tree
{

// Tell the standalone crane_mpc node what the crane carries.
//
// The OCP's payload only moves over crane_msgs/SetPayload and nothing called it,
// so the MPC solved every carrying move for an empty hook. Issue 174's robust
// sweep found exactly one condition that leaves the measured mismatch envelope:
// softest k on every axis, high Psi gain and an *unknown* 800 kg block, where the
// rotator runs 2.2x past its clamp. The same corner arrives once the OCP knows.
//
// Same source as the planner: {payload_mass} and the {s_log_8_*} that
// CaptureBlockGraspOffset writes at the grasp. PublishGripperState forwards those
// same keys to the in-JTC MPC plugin; this is the standalone node's channel.
//
// Deliberately not a nav2 BtServiceNode: that constructor throws when no server
// answers, and these subtrees are built for controller:=pid too, where no
// crane_mpc runs at all.
//
// Whether a payload the MPC did not take is fatal is therefore decided by
// `a2b_mpc_drives`, the same parameter WaitForMoveComplete reads: only on
// cbs_mpc_active does this node's model reach the crane, and only there does a
// missing, silent or refusing server have to stop the pick. Elsewhere it is a
// profile without an MPC, not a failed pick.
//
// Precondition: crane_mpc drops its warm start and re-seeds the force state on
// every accepted change (node.py, `forget_plan`). Both call sites sit between
// trajectories with the crane near rest; ticking this mid-move is a different
// question and needs its own issue.
class SetMpcPayload : public BT::SyncActionNode
{
public:
  using SetPayload = crane_msgs::srv::SetPayload;

  SetMpcPayload(const std::string & name, const BT::NodeConfig & conf);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "service_name", "/crane/mpc/set_payload",
        "Service the standalone crane_mpc node advertises"),
      BT::InputPort<int>("timeout_ms", 2000, "How long to wait for the answer"),
      BT::InputPort<bool>(
        "carries", false,
        "Is a payload held after this step? false clears the OCP's payload"),
      BT::InputPort<double>("mass", 0.0, "Payload mass in kg"),
      BT::InputPort<double>("s_log_8_x", 0.0, "Payload CoM x in K8 [m]"),
      BT::InputPort<double>("s_log_8_y", 0.0, "Payload CoM y in K8 [m]"),
      BT::InputPort<double>("s_log_8_z", 0.0, "Payload CoM z in K8 [m]"),
      // world_model.yaml block_dimensions_m; the grasp turns the block about K8's x only
      BT::InputPort<double>("size_x", 0.9, "Payload box edge along K8 x [m]"),
      BT::InputPort<double>("size_y", 0.6, "Payload box edge along K8 y [m]"),
      BT::InputPort<double>("size_z", 0.6, "Payload box edge along K8 z [m]"),
    };
  }

private:
  // Replace a non-finite value with 0.0 and warn: crane_mpc refuses a payload
  // whose CoM is not finite on all three axes, and s_log_8_z sits at NaN on the
  // blackboard until CaptureBlockGraspOffset has run. A refusal here would abort
  // the pick with the block already in the jaws.
  double sanitize(double value, const char * field_name);

  // FAILURE where the MPC drives the crane, SUCCESS where it does not.
  BT::NodeStatus not_taken(const std::string & why);

  rclcpp::Node::SharedPtr node_;
  bool mpc_drives_{false};
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Client<SetPayload>::SharedPtr client_;
  std::string service_name_;
};

}  // namespace concrete_block_behavior_tree
