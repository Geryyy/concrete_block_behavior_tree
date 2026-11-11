#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "concrete_block_behavior_tree/status_text.hpp"
#include "concrete_block_world_model_interfaces/srv/run_pose_estimation.hpp"
#include "nav2_behavior_tree/bt_service_node.hpp"
#include "std_msgs/msg/string.hpp"

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
  : nav2_behavior_tree::BtServiceNode<ServiceT>(name, config)
  {
    // Whether this pass got a pose, and why not. Volatile: the panel pairs it with the
    // measurement that follows, and a replay has nothing to pair with.
    status_ = node_->create_publisher<std_msgs::msg::String>(
      "/cbs/status/pose", rclcpp::QoS(10));
  }

  // Wraps the base tick so a timeout, which never reaches on_completion, is reported too.
  BT::NodeStatus tick() override
  {
    has_pose_ = false;
    message_.clear();
    const BT::NodeStatus status = nav2_behavior_tree::BtServiceNode<ServiceT>::tick();
    if (status != BT::NodeStatus::RUNNING) {
      const bool ok = status == BT::NodeStatus::SUCCESS;
      if (!ok && message_.empty()) {
        message_ = "no response";   // timed out: on_completion never ran
      }
      std_msgs::msg::String line;
      line.data = pose_status(
        request_->mode, request_->target_block_id, ok, frame_, has_pose_ ? xyz_ : nullptr,
        message_);
      status_->publish(line);
      if (!ok) {
        // The goal ends here since 303, so say which step and which block ended it.
        RCLCPP_ERROR(
          node_->get_logger(), "%s %s: %s", request_->mode.c_str(),
          request_->target_block_id.c_str(), message_.c_str());
      }
    }
    return status;
  }

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
    message_ = response->message;
    frame_ = response->blocks.header.frame_id;
    for (const auto & block : response->blocks.blocks) {
      if (block.id == request_->target_block_id) {
        xyz_[0] = block.pose.position.x;
        xyz_[1] = block.pose.position.y;
        xyz_[2] = block.pose.position.z;
        has_pose_ = true;
      }
    }
    return response->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  std::string frame_;
  std::string message_;
  double xyz_[3]{};
  bool has_pose_{false};
};

}  // namespace concrete_block_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<concrete_block_behavior_tree::RunPoseEstimationService>(
    "RunPoseEstimation");
}
