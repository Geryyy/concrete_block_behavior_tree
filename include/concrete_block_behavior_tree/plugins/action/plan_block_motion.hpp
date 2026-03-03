#pragma once

#include <memory>
#include <string>

#include "concrete_block_motion_planning/srv/plan_block_motion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_behavior_tree/bt_service_node.hpp"
#include "nav_msgs/msg/path.hpp"

namespace concrete_block_behavior_tree
{

class PlanBlockMotionService
  : public nav2_behavior_tree::BtServiceNode<concrete_block_motion_planning::srv::PlanBlockMotion>
{
public:
  using ServiceT = concrete_block_motion_planning::srv::PlanBlockMotion;
  using ResponseT = concrete_block_motion_planning::srv::PlanBlockMotion_Response;

  PlanBlockMotionService(const std::string & service_name, const BT::NodeConfiguration & conf)
  : nav2_behavior_tree::BtServiceNode<ServiceT>(service_name, conf) {}

  void on_tick() override;
  BT::NodeStatus on_completion(std::shared_ptr<ResponseT> response) override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<geometry_msgs::msg::PoseStamped>("start_pose"),
        BT::InputPort<geometry_msgs::msg::PoseStamped>("goal_pose"),
        BT::InputPort<std::string>("target_block_id", "", "Target block id"),
        BT::InputPort<std::string>("reference_block_id", "", "Reference block id"),
        BT::InputPort<std::string>("planner_method", "POWELL", "Planner method"),
        BT::InputPort<double>("timeout_s", 5.0, "Planning timeout"),
        BT::InputPort<bool>("use_world_model", true, "Use current world model"),
        BT::OutputPort<std::string>("plan_id"),
        BT::OutputPort<nav_msgs::msg::Path>("planned_path"),
        BT::OutputPort<bool>("planning_ok"),
        BT::OutputPort<std::string>("planning_message")
      });
  }
};

}  // namespace concrete_block_behavior_tree
