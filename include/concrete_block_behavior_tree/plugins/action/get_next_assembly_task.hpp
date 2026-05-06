#pragma once

#include <cmath>
#include <memory>
#include <string>

#include "concrete_block_motion_planning/srv/get_next_assembly_task.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_behavior_tree/bt_service_node.hpp"

namespace concrete_block_behavior_tree
{

class GetNextAssemblyTaskService
  : public nav2_behavior_tree::BtServiceNode<concrete_block_motion_planning::srv::GetNextAssemblyTask>
{
public:
  using ServiceT = concrete_block_motion_planning::srv::GetNextAssemblyTask;
  using ResponseT = concrete_block_motion_planning::srv::GetNextAssemblyTask_Response;

  GetNextAssemblyTaskService(const std::string & service_name, const BT::NodeConfiguration & conf)
  : nav2_behavior_tree::BtServiceNode<ServiceT>(service_name, conf) {}

  void on_tick() override;
  BT::NodeStatus on_completion(std::shared_ptr<ResponseT> response) override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<std::string>("wall_plan_name", "basic_interlocking_3_2", "Wall plan name"),
        BT::InputPort<bool>("reset_plan", false, "Reset plan progress before requesting next task"),
        BT::OutputPort<std::string>("task_id"),
        BT::OutputPort<std::string>("target_block_id"),
        BT::OutputPort<std::string>("reference_block_id"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("target_block_pose_coarse"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("target_block_pose_precise"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("reference_block_pose_precise"),
        BT::OutputPort<bool>("plan_has_task"),
        BT::OutputPort<std::string>("plan_message"),
        // Decomposed pickup pose (for SetGoalPose / CalcA2BMovement compatibility)
        BT::OutputPort<double>("pickup_x"),
        BT::OutputPort<double>("pickup_y"),
        BT::OutputPort<double>("pickup_z"),
        BT::OutputPort<double>("pickup_yaw"),
        // Decomposed target/place pose
        BT::OutputPort<double>("place_x"),
        BT::OutputPort<double>("place_y"),
        BT::OutputPort<double>("place_z"),
        BT::OutputPort<double>("place_yaw")
      });
  }
};

}  // namespace concrete_block_behavior_tree

