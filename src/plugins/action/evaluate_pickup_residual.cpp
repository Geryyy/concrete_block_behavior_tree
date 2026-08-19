#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "concrete_block_world_model_interfaces/srv/get_coarse_blocks.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_behavior_tree/bt_service_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace concrete_block_behavior_tree
{

class EvaluatePickupResidual
  : public nav2_behavior_tree::BtServiceNode<
    concrete_block_world_model_interfaces::srv::GetCoarseBlocks>
{
public:
  using ServiceT = concrete_block_world_model_interfaces::srv::GetCoarseBlocks;
  using ResponseT = ServiceT::Response;

  EvaluatePickupResidual(const std::string & name, const BT::NodeConfiguration & config)
  : nav2_behavior_tree::BtServiceNode<ServiceT>(name, config)
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    status_pub_ = node_->create_publisher<std_msgs::msg::String>(
      "/assembly_operator/pickup_refinement_status", rclcpp::QoS(1).transient_local());
    indicator_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
      "/assembly_operator/pickup_residual_within_indicator", rclcpp::QoS(1).transient_local());
    error_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
      "/assembly_operator/pickup_translation_error_m", rclcpp::QoS(1).transient_local());
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<std::string>("block_id", "", "Ground block to evaluate"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("expected_pickup_pose", "Planned block CoG pose"),
      BT::InputPort<double>("planned_pickup_x", "Uncorrected pickup x (planning frame)"),
      BT::InputPort<double>("planned_pickup_y", "Uncorrected pickup y (planning frame)"),
      BT::InputPort<std::string>("planning_frame", "K0_mounting_base", "Comparison frame"),
      BT::InputPort<double>("translation_tolerance_m", 0.04, "Allowed pickup XY error"),
      BT::OutputPort<bool>("within_tolerance"),
      BT::OutputPort<double>("translation_error_m"),
      BT::OutputPort<double>("residual_dx_m"),
      BT::OutputPort<double>("residual_dy_m"),
      BT::OutputPort<double>("corrected_pickup_x"),
      BT::OutputPort<double>("corrected_pickup_y"),
      BT::OutputPort<std::string>("message")});
  }

  void on_tick() override
  {
    if (!getInput("block_id", block_id_) || block_id_.empty() ||
      !getInput("expected_pickup_pose", expected_) ||
      !getInput("planned_pickup_x", planned_x_) || !getInput("planned_pickup_y", planned_y_))
    {
      throw std::invalid_argument("EvaluatePickupResidual requires block, planned pose, and XY");
    }
    getInput("planning_frame", planning_frame_);
    getInput("translation_tolerance_m", tolerance_m_);
  }

  BT::NodeStatus on_completion(std::shared_ptr<ResponseT> response) override
  {
    if (!response->success) {
      return fail("Pickup residual query failed: " + response->message);
    }
    const auto it = std::find_if(response->blocks.blocks.begin(), response->blocks.blocks.end(),
      [this](const auto & block) {return block.id == block_id_;});
    if (it == response->blocks.blocks.end()) {
      return fail("Pickup block '" + block_id_ + "' is absent from the world model");
    }

    geometry_msgs::msg::PoseStamped observed;
    observed.header = response->blocks.header;
    observed.header.frame_id = observed.header.frame_id.empty() ? "world" : observed.header.frame_id;
    observed.pose = it->pose;
    try {
      if (observed.header.frame_id != planning_frame_) {
        observed = tf_buffer_->transform(observed, planning_frame_, tf2::durationFromSec(0.5));
      }
      if (expected_.header.frame_id.empty()) {
        expected_.header.frame_id = planning_frame_;
      }
      if (expected_.header.frame_id != planning_frame_) {
        expected_ = tf_buffer_->transform(expected_, planning_frame_, tf2::durationFromSec(0.5));
      }
    } catch (const tf2::TransformException & ex) {
      return fail("Cannot transform pickup pose to '" + planning_frame_ + "': " + ex.what());
    }

    // Pickup follows the observed ground block, so its correction is observed-minus-planned.
    const double dx = observed.pose.position.x - expected_.pose.position.x;
    const double dy = observed.pose.position.y - expected_.pose.position.y;
    const double error = std::hypot(dx, dy);
    const bool within = error <= tolerance_m_;
    const double corrected_x = planned_x_ + dx;
    const double corrected_y = planned_y_ + dy;
    std::ostringstream text;
    text << std::fixed << std::setprecision(3)
         << "Pickup residual: dxy=[" << dx << ", " << dy << "] m, |d|=" << error
         << " m; " << (within ? "within tolerance" : "correction recommended");
    publish(text.str(), within, error);
    setOutput("within_tolerance", within);
    setOutput("translation_error_m", error);
    setOutput("residual_dx_m", dx);
    setOutput("residual_dy_m", dy);
    setOutput("corrected_pickup_x", corrected_x);
    setOutput("corrected_pickup_y", corrected_y);
    setOutput("message", text.str());
    RCLCPP_INFO(node_->get_logger(), "%s; corrected pickup=(%.3f, %.3f)",
      text.str().c_str(), corrected_x, corrected_y);
    return BT::NodeStatus::SUCCESS;
  }

private:
  BT::NodeStatus fail(const std::string & message)
  {
    std_msgs::msg::String status;
    status.data = message;
    status_pub_->publish(status);
    setOutput("within_tolerance", false);
    setOutput("message", message);
    RCLCPP_WARN(node_->get_logger(), "%s", message.c_str());
    return BT::NodeStatus::FAILURE;
  }

  void publish(const std::string & text, bool within, double error)
  {
    std_msgs::msg::String status;
    status.data = text;
    status_pub_->publish(status);
    std_msgs::msg::Bool indicator;
    indicator.data = within;
    indicator_pub_->publish(indicator);
    std_msgs::msg::Float64 error_message;
    error_message.data = error;
    error_pub_->publish(error_message);
  }

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr indicator_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr error_pub_;
  std::string block_id_;
  std::string planning_frame_{"K0_mounting_base"};
  geometry_msgs::msg::PoseStamped expected_;
  double planned_x_{0.0};
  double planned_y_{0.0};
  double tolerance_m_{0.04};
};

}  // namespace concrete_block_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<concrete_block_behavior_tree::EvaluatePickupResidual>(
    "EvaluatePickupResidual");
}
