#pragma once

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "crane_model/model.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace concrete_block_behavior_tree
{

// Fill the trajectory's effort field with C3's dynamic feedforward.
//
// wiki/control_architecture.md §6 item 2: feedback keeps qdot_ref, the feedforward takes C3's
// exact inversion u_i = qdot_d,i + tau_dot_d,i / k_i, advanced by the dead time. The two branches
// need two different signals and JointTrajectoryPoint has one velocity field, so the difference
//
//   effort_i = u_i(t + dead_time) - qdot_d,i(t)
//
// travels in the effort field. The JTC hands it to the control law as a feedforward term when
// `effort_field_is_feedforward` is set (jtc_fork.md delta 3); the PI keeps feeding back on the
// untouched velocities. The arithmetic is c3_feedforward_math.hpp, which is where it is tested.
//
// tau_d is the full RNEA (crane_model::Model::inverse_dynamics), not the per-axis PT2 of
// controller_design.md §2.4: same law, better tau_d. It brings the gravity rate dg/dq*qdot_d, the
// off-diagonal M_ij and Mdot, which a per-axis form cannot see and which controller_design.md §4.3
// puts at 26%-136% of the command domain on a multi-axis move.
//
// Not inverted here: the command PT1 tau_v, which would need a further derivative of a reference
// that is already differentiated numerically (§2.4).
//
// Off unless `c3_feedforward.enabled` is set on the BT server, and a pass-through on any
// trajectory it cannot handle: a wrong feedforward degrades tracking, but a motion that fails to
// start is a stopped machine.
class AddC3Feedforward : public BT::SyncActionNode
{
public:
  using JointTrajectory = trajectory_msgs::msg::JointTrajectory;

  AddC3Feedforward(const std::string & name, const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::BidirectionalPort<JointTrajectory>(
        "trajectory", "Planned trajectory; the effort field is filled in place"),
      BT::InputPort<bool>("carries", false, "Is a payload currently held?"),
      BT::InputPort<double>("mass", 0.0, "Payload mass in kg"),
      BT::InputPort<double>("s_log_8_x", 0.0, "Payload CoM x in K8 [m]"),
      BT::InputPort<double>("s_log_8_y", 0.0, "Payload CoM y in K8 [m]"),
      BT::InputPort<double>("s_log_8_z", 0.0, "Payload CoM z in K8 [m]"),
    };
  }

private:
  // Build the model from /robot_description on first use. Returns false and logs once if the
  // description has not arrived or the model rejects it.
  bool ensure_model();

  // Map trajectory joint index -> crane_model contract index. Returns false if the trajectory
  // does not carry all eight coordinates, which RNEA needs.
  bool build_index_map(const JointTrajectory & trajectory, std::vector<int> & traj_to_model) const;

  rclcpp::Node::SharedPtr node_;
  // own callback group and executor: the BT server spins the shared client node from its own
  // timer, and rclcpp::spin_some on a node that is already associated with an executor throws
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr description_sub_;
  std::string robot_description_;
  std::unique_ptr<crane_model::Model> model_;

  bool enabled_{false};
  double dead_time_s_{0.06};
  // joint names as the parameters list them, before they are reordered into contract order
  std::vector<std::string> parameter_joints_;
  // per contract-order joint: 1/k_i, and the bound on the feedforward branch. 1/k_i is 0 on the
  // joints without a C3 fit, which then get no correction rather than a scaled-by-nothing one.
  std::vector<double> inv_k_;
  std::vector<double> u_min_;
  std::vector<double> u_max_;
  bool warned_{false};
  bool saturation_warned_{false};
};

}  // namespace concrete_block_behavior_tree
