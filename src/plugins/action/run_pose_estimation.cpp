#include "behaviortree_cpp/bt_factory.h"
#include "concrete_block_world_model_interfaces/srv/run_pose_estimation.hpp"
#include "nav2_behavior_tree/bt_service_node.hpp"

namespace concrete_block_behavior_tree
{

class RunPoseEstimationService
  : public nav2_behavior_tree::BtServiceNode<
    concrete_block_world_model_interfaces::srv::RunPoseEstimation>
{
public:
  using ServiceT = concrete_block_world_model_interfaces::srv::RunPoseEstimation;
  using ResponseT = ServiceT::Response;

  RunPoseEstimationService(const std::string & name, const BT::NodeConfig & config)
  : nav2_behavior_tree::BtServiceNode<ServiceT>(name, config) {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<std::string>("mode", "REFINE_GRASPED", "Pose-estimation mode"),
      BT::InputPort<std::string>("target_block_id", "", "Target block ID"),
      BT::InputPort<bool>("enable_debug", true, "Publish debug output"),
      BT::InputPort<double>("timeout_s", 8.0, "Service timeout"),
      BT::OutputPort<std::string>("message")});
  }

  void on_tick() override
  {
    getInput("mode", request_->mode);
    getInput("target_block_id", request_->target_block_id);
    getInput("enable_debug", request_->enable_debug);
    double timeout_s = 8.0;
    getInput("timeout_s", timeout_s);
    request_->timeout_s = static_cast<float>(timeout_s);
  }

  BT::NodeStatus on_completion(std::shared_ptr<ResponseT> response) override
  {
    setOutput("message", response->message);
    return response->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

}  // namespace concrete_block_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<concrete_block_behavior_tree::RunPoseEstimationService>(
    "RunPoseEstimation");
}
