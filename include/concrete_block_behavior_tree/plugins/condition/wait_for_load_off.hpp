#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace concrete_block_behavior_tree
{

// Has the carried block landed: the stop for a place descent, run beside it.
//
// SUCCESS needs both, at once:
//  - the hook lost the load: `joint` effort (the boom) fell below its hanging
//    reference -- median of the first 0.5 s -- by `drop_fraction` of the
//    payload's gravity torque m g r, r the horizontal lever from `lever_frame` to
//    the block, for `hold_s`. Run 11: 111 -> 55 kN m on touchdown against +-7 kN m
//    from the descent itself.
//  - the block is not in the air: its centre (|s_log_8| straight below `k8_frame`) is at most
//    `max_above_target` over `target_z` at the lowest point of the descent (the
//    boom springs up as it unloads). An effort dip mid-air must not open the jaws.
// FAILURE, which halts the descent, when the centre goes `floor_below_target`
// under `target_z` -- the kinematic floor, for when the effort never drops --, when
// the load comes off higher than `max_above_target`, on a non-finite or zero input,
// on a transform older than 0.2 s, or at `timeout_ms`. None opens the jaws: the tree
// stops for the operator. Assumes the effort falls when the hook unloads (boom,
// sim and the 2026-08-19 bags: positive under load).
class WaitForLoadOff : public BT::StatefulActionNode
{
public:
  WaitForLoadOff(const std::string & name, const BT::NodeConfig & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("mass", "Carried mass [kg]"),
      BT::InputPort<double>("s_log_8_x", "Block centre in k8_frame [m]"),
      BT::InputPort<double>("s_log_8_y", "Block centre in k8_frame [m]"),
      BT::InputPort<double>("s_log_8_z", "Block centre in k8_frame [m]"),
      BT::InputPort<double>("target_z", "Block centre target height in world_frame [m]"),
      BT::InputPort<std::string>(
        "joint", "theta2_boom_joint", "Joint whose effort carries the load"),
      BT::InputPort<std::string>("lever_frame", "moved_with_q2", "Frame on that joint's axis"),
      BT::InputPort<std::string>("k8_frame", "K8_rotator_lower_part", "Frame s_log_8 is in"),
      BT::InputPort<std::string>(
        "frame", "K0_mounting_base", "Frame target_z is in, z up: the planning frame"),
      BT::InputPort<double>("drop_fraction", 0.25, "Share of m g r the effort must lose"),
      BT::InputPort<double>("max_above_target", 0.20, "Highest the block may have stayed [m]"),
      BT::InputPort<double>("floor_below_target", 0.01, "Lowest block centre ever [m]"),
      BT::InputPort<int>("timeout_ms", 20000, "Give up after [ms]"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override {}

private:
  using JointState = sensor_msgs::msg::JointState;
  void callback(const JointState::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<JointState>::SharedPtr sub_;
  // /crane/load_off per tick (PlotJuggler): [reference - effort, drop needed, centre above target]
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::mutex mutex_;
  std::string joint_;
  std::vector<double> window_;
  double effort_{0.0};
  bool fresh_{false};

  double reference_{0.0};
  bool referenced_{false};
  std::chrono::steady_clock::time_point start_;
  std::chrono::steady_clock::time_point dropped_since_;
  bool dropped_{false};
  double lowest_{0.0};
};

}  // namespace concrete_block_behavior_tree
