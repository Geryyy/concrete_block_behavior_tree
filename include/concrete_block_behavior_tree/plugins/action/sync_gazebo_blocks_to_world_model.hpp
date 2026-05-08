#pragma once

#include <chrono>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "concrete_block_world_model_interfaces/msg/block.hpp"
#include "concrete_block_world_model_interfaces/srv/upsert_block.hpp"
#include "gazebo_msgs/srv/get_entity_state.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"

namespace concrete_block_behavior_tree
{

class SyncGazeboBlocksToWorldModel : public BT::SyncActionNode
{
public:
  SyncGazeboBlocksToWorldModel(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("block_ids", "block_1,block_2", "Comma-separated Gazebo model names"),
      BT::InputPort<std::string>(
        "gazebo_state_service", "/gazebo/get_entity_state", "Gazebo GetEntityState service"),
      BT::InputPort<std::string>(
        "upsert_service", "/world_model_node/upsert_block", "World-model UpsertBlock service"),
      BT::InputPort<std::string>("gazebo_reference_frame", "world", "Gazebo pose reference frame"),
      BT::InputPort<std::string>("world_frame_id", "K0_mounting_base", "World-model frame ID"),
      BT::InputPort<std::string>(
        "gazebo_seed_frame_xyz", "0.0,0.0,0.0", "world_frame_id origin in Gazebo xyz"),
      BT::InputPort<std::string>(
        "gazebo_seed_frame_rpy_deg", "0.0,0.0,0.0", "world_frame_id orientation in Gazebo RPY degrees"),
      BT::InputPort<int>(
        "pose_status",
        concrete_block_world_model_interfaces::msg::Block::POSE_PRECISE,
        "World-model pose status"),
      BT::InputPort<int>(
        "task_status",
        concrete_block_world_model_interfaces::msg::Block::TASK_FREE,
        "World-model task status"),
      BT::InputPort<double>("confidence", 1.0, "World-model pose confidence"),
      BT::InputPort<int>("service_timeout_ms", 5000, "Service timeout in milliseconds")
    };
  }

  BT::NodeStatus tick() override;

private:
  using GetEntityState = gazebo_msgs::srv::GetEntityState;
  using UpsertBlock = concrete_block_world_model_interfaces::srv::UpsertBlock;

  struct Quaternion
  {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double w{1.0};
  };

  struct Vector3
  {
    double x{0.0};
    double y{0.0};
    double z{0.0};
  };

  static std::vector<std::string> splitCsv(const std::string & value);
  static std::vector<double> parseDoubles(const std::string & value, std::size_t expected_size);
  static Quaternion rpyDegToQuaternion(double roll_deg, double pitch_deg, double yaw_deg);
  static Quaternion multiply(const Quaternion & lhs, const Quaternion & rhs);
  static Quaternion conjugate(const Quaternion & q);
  static Quaternion normalize(const Quaternion & q);
  static Vector3 rotate(const Quaternion & q, const Vector3 & v);

  bool queryGazeboPose(const std::string & block_id, geometry_msgs::msg::Pose & pose);
  bool upsertBlock(const std::string & block_id, const geometry_msgs::msg::Pose & pose);
  geometry_msgs::msg::Pose gazeboPoseToWorldModelPose(const geometry_msgs::msg::Pose & gazebo_pose) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Client<GetEntityState>::SharedPtr gazebo_client_;
  rclcpp::Client<UpsertBlock>::SharedPtr upsert_client_;

  std::string gazebo_reference_frame_{"world"};
  std::string world_frame_id_{"K0_mounting_base"};
  Vector3 gazebo_seed_frame_xyz_;
  Quaternion gazebo_seed_frame_quat_;
  int pose_status_{concrete_block_world_model_interfaces::msg::Block::POSE_PRECISE};
  int task_status_{concrete_block_world_model_interfaces::msg::Block::TASK_FREE};
  double confidence_{1.0};
  std::chrono::milliseconds service_timeout_{5000};
};

}  // namespace concrete_block_behavior_tree
