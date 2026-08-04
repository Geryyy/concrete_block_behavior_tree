#pragma once

#include <cmath>
#include <memory>
#include <string>

#include "concrete_block_assembly_interfaces/srv/get_next_assembly_task.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_behavior_tree/bt_service_node.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace concrete_block_behavior_tree
{

class GetNextAssemblyTaskService
  : public nav2_behavior_tree::BtServiceNode<concrete_block_assembly_interfaces::srv::GetNextAssemblyTask>
{
public:
  using ServiceT = concrete_block_assembly_interfaces::srv::GetNextAssemblyTask;
  using ResponseT = concrete_block_assembly_interfaces::srv::GetNextAssemblyTask_Response;

  GetNextAssemblyTaskService(const std::string & service_name, const BT::NodeConfiguration & conf)
  : nav2_behavior_tree::BtServiceNode<ServiceT>(service_name, conf)
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    if (!node_->has_parameter("simulated_placement_disturbance.enabled")) {
      node_->declare_parameter("simulated_placement_disturbance.enabled", false);
    }
    if (!node_->has_parameter("simulated_placement_disturbance.x_m")) {
      node_->declare_parameter("simulated_placement_disturbance.x_m", 0.06);
    }
    if (!node_->has_parameter("simulated_placement_disturbance.y_m")) {
      node_->declare_parameter("simulated_placement_disturbance.y_m", -0.04);
    }
    if (!node_->has_parameter("simulated_placement_disturbance.z_m")) {
      node_->declare_parameter("simulated_placement_disturbance.z_m", 0.0);
    }
    if (!node_->has_parameter("simulated_placement_disturbance.yaw_rad")) {
      node_->declare_parameter("simulated_placement_disturbance.yaw_rad", 0.035);
    }
  }

  void on_tick() override;
  BT::NodeStatus on_completion(std::shared_ptr<ResponseT> response) override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<std::string>("wall_plan_name", "basic_interlocking_3_2", "Wall plan name"),
        BT::InputPort<bool>("reset_plan", false, "Reset plan progress before requesting next task"),
        BT::InputPort<std::string>(
          "planning_frame", "K0_mounting_base",
          "Frame the emitted poses are transformed into (crane planning frame)"),
        BT::OutputPort<std::string>("task_id"),
        BT::OutputPort<std::string>("target_block_id"),
        BT::OutputPort<std::string>("reference_block_id"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("target_block_pose_coarse"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("target_block_pose_precise"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("reference_block_pose_precise"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("placement_approach_pose"),
        BT::OutputPort<bool>("plan_has_task"),
        BT::OutputPort<std::string>("plan_message"),
        // Decomposed pickup pose (for SetGoalPose / CalcA2BMovement compatibility)
        BT::OutputPort<double>("pickup_x"),
        BT::OutputPort<double>("pickup_y"),
        BT::OutputPort<double>("pickup_z"),
        BT::OutputPort<double>("pickup_yaw"),
        BT::OutputPort<double>("pickup_approach_z"),
        // Decomposed target/place pose
        BT::OutputPort<double>("place_x"),
        BT::OutputPort<double>("place_y"),
        BT::OutputPort<double>("place_z"),
        BT::OutputPort<double>("place_yaw"),
        // Decomposed pre-place approach point (above + laterally offset)
        BT::OutputPort<double>("approach_x"),
        BT::OutputPort<double>("approach_y"),
        BT::OutputPort<double>("approach_z"),
        // Simulation-only first-hover output.  It equals approach_* unless
        // the Gazebo launch enables the placement-disturbance parameters.
        BT::OutputPort<double>("disturbed_approach_x"),
        BT::OutputPort<double>("disturbed_approach_y"),
        BT::OutputPort<double>("disturbed_approach_z"),
        BT::OutputPort<double>("disturbed_approach_yaw")
      });
  }

private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace concrete_block_behavior_tree
