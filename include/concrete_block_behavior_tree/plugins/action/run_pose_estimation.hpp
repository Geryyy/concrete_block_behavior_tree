#pragma once

#include <memory>
#include <string>

#include "concrete_block_perception/srv/run_pose_estimation.hpp"
#include "nav2_behavior_tree/bt_service_node.hpp"

namespace concrete_block_behavior_tree
{

class RunPoseEstimationService
  : public nav2_behavior_tree::BtServiceNode<concrete_block_perception::srv::RunPoseEstimation>
{
public:
  using ServiceT = concrete_block_perception::srv::RunPoseEstimation;
  using ResponseT = concrete_block_perception::srv::RunPoseEstimation_Response;

  RunPoseEstimationService(const std::string & service_name, const BT::NodeConfiguration & conf)
  : nav2_behavior_tree::BtServiceNode<ServiceT>(service_name, conf) {}

  void on_tick() override;
  BT::NodeStatus on_completion(std::shared_ptr<ResponseT> response) override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<std::string>("mode", "SCENE_DISCOVERY", "Perception mode"),
        BT::InputPort<std::string>("target_block_id", "", "Target block id for refine modes"),
        BT::InputPort<bool>("enable_debug", true, "Enable debug overlay"),
        BT::InputPort<double>("timeout_s", 8.0, "Perception request timeout"),
        BT::OutputPort<bool>("perception_ok"),
        BT::OutputPort<std::string>("perception_message")
      });
  }
};

}  // namespace concrete_block_behavior_tree
