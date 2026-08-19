#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "concrete_block_world_model_interfaces/msg/block.hpp"
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
#include "visualization_msgs/msg/marker_array.hpp"

namespace concrete_block_behavior_tree
{

namespace
{

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace

class EvaluatePlacementResidual
  : public nav2_behavior_tree::BtServiceNode<
    concrete_block_world_model_interfaces::srv::GetCoarseBlocks>
{
public:
  using ServiceT = concrete_block_world_model_interfaces::srv::GetCoarseBlocks;
  using ResponseT = ServiceT::Response;

  EvaluatePlacementResidual(const std::string & name, const BT::NodeConfiguration & config)
  : nav2_behavior_tree::BtServiceNode<ServiceT>(name, config)
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    status_pub_ = node_->create_publisher<std_msgs::msg::String>(
      "/assembly_operator/refinement_status", rclcpp::QoS(1).transient_local());
    markers_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/assembly_operator/refinement_markers", rclcpp::QoS(1).transient_local());
    indicator_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
      "/assembly_operator/placement_residual_within_indicator", rclcpp::QoS(1).transient_local());
    indicator_threshold_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
      "/assembly_operator/placement_indicator_threshold_m", rclcpp::QoS(1).transient_local());
    translation_error_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
      "/assembly_operator/placement_translation_error_m", rclcpp::QoS(1).transient_local());
    // node_ is the bt_action_server's shared rclcpp node and outlives the tree.
    // Every tree (re)load constructs this plugin again, so declaring
    // unconditionally throws ParameterAlreadyDeclared on the second load --
    // the first goal succeeds and every later one fails with "File exists but
    // can't be loaded".
    indicator_threshold_m_ = node_->has_parameter("placement_indicator_threshold_m")
      ? node_->get_parameter("placement_indicator_threshold_m").as_double()
      : node_->declare_parameter<double>("placement_indicator_threshold_m", 0.10);
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<std::string>("block_id", "", "TASK_MOVE block to evaluate"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "expected_placement_pose", "Planned final block CoG pose"),
      BT::InputPort<bool>(
        "compare_z", false,
        "Compare vertical error too; false is appropriate while hovering above the placement"),
      BT::InputPort<std::string>("planning_frame", "K0_mounting_base", "Comparison frame"),
      BT::InputPort<double>("translation_tolerance_m", 0.04, "Allowed CoG error"),
      BT::InputPort<double>("yaw_tolerance_rad", 0.0873, "Allowed yaw error"),
      BT::InputPort<double>("block_length_m", 0.90, "Visualised block length"),
      BT::InputPort<double>("block_width_m", 0.60, "Visualised block width"),
      BT::InputPort<double>("block_height_m", 0.60, "Visualised block height"),
      BT::OutputPort<bool>("within_tolerance"),
      BT::OutputPort<double>("translation_error_m"),
      BT::OutputPort<double>("yaw_error_rad"),
      BT::OutputPort<std::string>("message")});
  }

  void on_tick() override
  {
    if (!getInput("block_id", block_id_) || block_id_.empty()) {
      throw std::invalid_argument("EvaluatePlacementResidual requires block_id");
    }
    if (!getInput("expected_placement_pose", expected_placement_pose_)) {
      throw std::invalid_argument("EvaluatePlacementResidual requires expected_placement_pose");
    }
    getInput("compare_z", compare_z_);
    getInput("planning_frame", planning_frame_);
    getInput("translation_tolerance_m", translation_tolerance_m_);
    getInput("yaw_tolerance_rad", yaw_tolerance_rad_);
    getInput("block_length_m", block_length_m_);
    getInput("block_width_m", block_width_m_);
    getInput("block_height_m", block_height_m_);
  }

  BT::NodeStatus on_completion(std::shared_ptr<ResponseT> response) override
  {
    if (!response->success) {
      return fail("World-model query failed: " + response->message);
    }
    const auto block_it = std::find_if(
      response->blocks.blocks.begin(), response->blocks.blocks.end(),
      [this](const auto & block) {return block.id == block_id_;});
    if (block_it == response->blocks.blocks.end()) {
      return fail("Grasped block '" + block_id_ + "' is absent from the world model");
    }
    if (block_it->task_status !=
      concrete_block_world_model_interfaces::msg::Block::TASK_MOVE)
    {
      return fail("Block '" + block_id_ + "' is not TASK_MOVE; FK pose is unavailable");
    }

    geometry_msgs::msg::PoseStamped observed;
    observed.header = response->blocks.header;
    observed.header.frame_id = observed.header.frame_id.empty() ? "world" : observed.header.frame_id;
    observed.pose = block_it->pose;
    try {
      if (observed.header.frame_id != planning_frame_) {
        observed = tf_buffer_->transform(
          observed, planning_frame_, tf2::durationFromSec(0.5));
      }
    } catch (const tf2::TransformException & ex) {
      return fail("Cannot transform FK block pose to '" + planning_frame_ + "': " + ex.what());
    }

    auto expected = expected_placement_pose_;
    if (expected.header.frame_id.empty()) {
      expected.header.frame_id = planning_frame_;
    }
    try {
      if (expected.header.frame_id != planning_frame_) {
        expected = tf_buffer_->transform(
          expected, planning_frame_, tf2::durationFromSec(0.5));
      }
    } catch (const tf2::TransformException & ex) {
      return fail("Cannot transform planned hover pose to '" + planning_frame_ + "': " + ex.what());
    }

    // A2B's high approach endpoint is a crane planner reference, not a
    // block-CoG setpoint.  While the block is hovering, validate the two
    // placement-relevant lateral coordinates and block yaw against the final
    // plan.  The later grip descent owns the vertical placement coordinate.
    if (!compare_z_) {
      expected.pose.position.z = observed.pose.position.z;
    }

    const double dx = expected.pose.position.x - observed.pose.position.x;
    const double dy = expected.pose.position.y - observed.pose.position.y;
    const double dz = expected.pose.position.z - observed.pose.position.z;
    const double translation_error = std::sqrt(dx * dx + dy * dy + dz * dz);
    const double observed_yaw = yawFromQuaternion(observed.pose.orientation);
    const double expected_yaw = yawFromQuaternion(expected.pose.orientation);
    const double yaw_error = std::atan2(
      std::sin(expected_yaw - observed_yaw), std::cos(expected_yaw - observed_yaw));
    const bool within = translation_error <= translation_tolerance_m_ &&
      std::abs(yaw_error) <= yaw_tolerance_rad_;
    const bool within_indicator = translation_error <= indicator_threshold_m_;

    std::ostringstream text;
    text << std::fixed << std::setprecision(3)
         << "Placement residual (FK " << (compare_z_ ? "XYZ" : "XY") << "+yaw, " << block_id_ << "): dxyz=[" << dx << ", " << dy << ", "
         << dz << "] m, |d|=" << translation_error << " m, dyaw="
         << yaw_error * 180.0 / 3.14159265358979323846 << " deg; "
         << (within ? "within tolerance" : "correction recommended");
    publish(text.str());
    std_msgs::msg::Bool indicator;
    indicator.data = within_indicator;
    indicator_pub_->publish(indicator);
    std_msgs::msg::Float64 indicator_threshold;
    indicator_threshold.data = indicator_threshold_m_;
    indicator_threshold_pub_->publish(indicator_threshold);
    std_msgs::msg::Float64 translation_error_message;
    translation_error_message.data = translation_error;
    translation_error_pub_->publish(translation_error_message);
    publishMarkers(observed, expected, dx, dy, dz, yaw_error, within, text.str());
    setOutput("within_tolerance", within);
    setOutput("translation_error_m", translation_error);
    setOutput("yaw_error_rad", yaw_error);
    setOutput("message", text.str());
    RCLCPP_INFO(node_->get_logger(), "%s", text.str().c_str());
    return BT::NodeStatus::SUCCESS;
  }

private:
  BT::NodeStatus fail(const std::string & message)
  {
    publish("Placement residual failed: " + message);
    setOutput("within_tolerance", false);
    setOutput("message", message);
    RCLCPP_WARN(node_->get_logger(), "%s", message.c_str());
    return BT::NodeStatus::FAILURE;
  }

  void publish(const std::string & text)
  {
    std_msgs::msg::String message;
    message.data = text;
    status_pub_->publish(message);
  }

  visualization_msgs::msg::Marker makeBlockMarker(
    const std_msgs::msg::Header & header, const geometry_msgs::msg::Pose & pose,
    int id, float r, float g, float b, float a, const std::string & label) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = "placement_residual";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = pose;
    marker.scale.x = block_length_m_;
    marker.scale.y = block_width_m_;
    marker.scale.z = block_height_m_;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    marker.text = label;
    return marker;
  }

  void publishMarkers(
    const geometry_msgs::msg::PoseStamped & observed,
    const geometry_msgs::msg::PoseStamped & expected,
    double dx, double dy, double dz, double yaw_error, bool within,
    const std::string & summary)
  {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);

    const auto header = observed.header;
    markers.markers.push_back(makeBlockMarker(
      header, observed.pose, 0, 0.1F, 0.4F, 1.0F, 0.30F, "measured FK pose"));
    markers.markers.push_back(makeBlockMarker(
      header, expected.pose, 1, 0.1F, 1.0F, 0.2F, 0.22F,
      compare_z_ ? "planned placement pose" : "planned XY/yaw projected to hover"));

    visualization_msgs::msg::Marker correction;
    correction.header = header;
    correction.ns = "placement_residual";
    correction.id = 2;
    correction.type = visualization_msgs::msg::Marker::ARROW;
    correction.action = visualization_msgs::msg::Marker::ADD;
    correction.scale.x = 0.035;
    correction.scale.y = 0.075;
    correction.scale.z = 0.10;
    correction.color.r = within ? 0.1F : 1.0F;
    correction.color.g = within ? 1.0F : 0.2F;
    correction.color.b = 0.1F;
    correction.color.a = 1.0F;
    correction.points.resize(2);
    correction.points[0] = observed.pose.position;
    correction.points[1] = expected.pose.position;
    markers.markers.push_back(correction);

    visualization_msgs::msg::Marker label;
    label.header = header;
    label.ns = "placement_residual";
    label.id = 3;
    label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    label.action = visualization_msgs::msg::Marker::ADD;
    label.pose = expected.pose;
    label.pose.position.z += block_height_m_ * 0.7;
    label.scale.z = 0.16;
    label.color.r = 1.0F;
    label.color.g = 1.0F;
    label.color.b = 1.0F;
    label.color.a = 1.0F;
    std::ostringstream text;
    text << std::fixed << std::setprecision(3)
         << "planned Cartesian correction\n"
         << "dxyz=[" << dx << ", " << dy << ", " << dz << "] m\n"
         << "dyaw=" << yaw_error * 180.0 / 3.14159265358979323846 << " deg\n"
         << (within ? "within tolerance" : "correction recommended");
    label.text = text.str();
    markers.markers.push_back(label);
    markers_pub_->publish(markers);
    RCLCPP_DEBUG(node_->get_logger(), "%s", summary.c_str());
  }

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr indicator_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr indicator_threshold_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr translation_error_pub_;
  std::string block_id_;
  std::string planning_frame_{"K0_mounting_base"};
  geometry_msgs::msg::PoseStamped expected_placement_pose_;
  bool compare_z_{false};
  double translation_tolerance_m_{0.04};
  double yaw_tolerance_rad_{0.0873};
  double block_length_m_{0.90};
  double block_width_m_{0.60};
  double block_height_m_{0.60};
  double indicator_threshold_m_{0.10};
};

}  // namespace concrete_block_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<concrete_block_behavior_tree::EvaluatePlacementResidual>(
    "EvaluatePlacementResidual");
}
