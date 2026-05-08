#include "concrete_block_behavior_tree/plugins/action/sync_gazebo_blocks_to_world_model.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "behaviortree_cpp_v3/bt_factory.h"

namespace concrete_block_behavior_tree
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
}

SyncGazeboBlocksToWorldModel::SyncGazeboBlocksToWorldModel(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  std::string gazebo_service;
  std::string upsert_service;
  getInput("gazebo_state_service", gazebo_service);
  getInput("upsert_service", upsert_service);
  if (gazebo_service.empty() || upsert_service.empty()) {
    throw std::invalid_argument(
      "SyncGazeboBlocksToWorldModel service names must not be empty");
  }

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(
    callback_group_,
    node_->get_node_base_interface());

  gazebo_client_ = node_->create_client<GetEntityState>(
    gazebo_service,
    rmw_qos_profile_services_default,
    callback_group_);
  upsert_client_ = node_->create_client<UpsertBlock>(
    upsert_service,
    rmw_qos_profile_services_default,
    callback_group_);

  RCLCPP_INFO(
    node_->get_logger(),
    "SyncGazeboBlocksToWorldModel initialized | gazebo_service=%s upsert_service=%s",
    gazebo_service.c_str(),
    upsert_service.c_str());
}

BT::NodeStatus SyncGazeboBlocksToWorldModel::tick()
{
  std::string block_ids_string;
  std::string xyz_string;
  std::string rpy_string;
  int timeout_ms = 5000;

  getInput("block_ids", block_ids_string);
  getInput("gazebo_reference_frame", gazebo_reference_frame_);
  getInput("world_frame_id", world_frame_id_);
  getInput("gazebo_seed_frame_xyz", xyz_string);
  getInput("gazebo_seed_frame_rpy_deg", rpy_string);
  getInput("pose_status", pose_status_);
  getInput("task_status", task_status_);
  getInput("confidence", confidence_);
  getInput("service_timeout_ms", timeout_ms);

  const auto block_ids = splitCsv(block_ids_string);
  if (block_ids.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "SyncGazeboBlocksToWorldModel: no block_ids configured");
    return BT::NodeStatus::FAILURE;
  }

  try {
    const auto xyz = parseDoubles(xyz_string, 3);
    const auto rpy = parseDoubles(rpy_string, 3);
    gazebo_seed_frame_xyz_ = {xyz[0], xyz[1], xyz[2]};
    gazebo_seed_frame_quat_ = rpyDegToQuaternion(rpy[0], rpy[1], rpy[2]);
  } catch (const std::exception & exc) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "SyncGazeboBlocksToWorldModel: invalid transform input: %s",
      exc.what());
    return BT::NodeStatus::FAILURE;
  }

  service_timeout_ = std::chrono::milliseconds(timeout_ms);
  if (!gazebo_client_->wait_for_service(service_timeout_)) {
    RCLCPP_ERROR(node_->get_logger(), "Gazebo state service is not available");
    return BT::NodeStatus::FAILURE;
  }
  if (!upsert_client_->wait_for_service(service_timeout_)) {
    RCLCPP_ERROR(node_->get_logger(), "World-model upsert service is not available");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "Syncing %zu Gazebo block pose(s) into world model frame '%s'",
    block_ids.size(),
    world_frame_id_.c_str());

  for (const auto & block_id : block_ids) {
    geometry_msgs::msg::Pose gazebo_pose;
    if (!queryGazeboPose(block_id, gazebo_pose)) {
      return BT::NodeStatus::FAILURE;
    }

    const auto world_model_pose = gazeboPoseToWorldModelPose(gazebo_pose);
    if (!upsertBlock(block_id, world_model_pose)) {
      return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::SUCCESS;
}

std::vector<std::string> SyncGazeboBlocksToWorldModel::splitCsv(const std::string & value)
{
  std::vector<std::string> result;
  std::stringstream stream(value);
  std::string item;
  while (std::getline(stream, item, ',')) {
    item.erase(item.begin(), std::find_if(item.begin(), item.end(), [](unsigned char ch) {
      return !std::isspace(ch);
    }));
    item.erase(std::find_if(item.rbegin(), item.rend(), [](unsigned char ch) {
      return !std::isspace(ch);
    }).base(), item.end());
    if (!item.empty()) {
      result.push_back(item);
    }
  }
  return result;
}

std::vector<double> SyncGazeboBlocksToWorldModel::parseDoubles(
  const std::string & value,
  std::size_t expected_size)
{
  const auto parts = splitCsv(value);
  if (parts.size() != expected_size) {
    throw std::invalid_argument("expected " + std::to_string(expected_size) + " comma-separated values");
  }

  std::vector<double> result;
  result.reserve(parts.size());
  for (const auto & part : parts) {
    result.push_back(std::stod(part));
  }
  return result;
}

SyncGazeboBlocksToWorldModel::Quaternion SyncGazeboBlocksToWorldModel::rpyDegToQuaternion(
  double roll_deg,
  double pitch_deg,
  double yaw_deg)
{
  const double roll = roll_deg * kPi / 180.0;
  const double pitch = pitch_deg * kPi / 180.0;
  const double yaw = yaw_deg * kPi / 180.0;

  const double cr = std::cos(roll / 2.0);
  const double sr = std::sin(roll / 2.0);
  const double cp = std::cos(pitch / 2.0);
  const double sp = std::sin(pitch / 2.0);
  const double cy = std::cos(yaw / 2.0);
  const double sy = std::sin(yaw / 2.0);

  return {
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy,
    cr * cp * cy + sr * sp * sy};
}

SyncGazeboBlocksToWorldModel::Quaternion SyncGazeboBlocksToWorldModel::multiply(
  const Quaternion & lhs,
  const Quaternion & rhs)
{
  return {
    lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y,
    lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x,
    lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w,
    lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z};
}

SyncGazeboBlocksToWorldModel::Quaternion SyncGazeboBlocksToWorldModel::conjugate(
  const Quaternion & q)
{
  return {-q.x, -q.y, -q.z, q.w};
}

SyncGazeboBlocksToWorldModel::Quaternion SyncGazeboBlocksToWorldModel::normalize(
  const Quaternion & q)
{
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (norm <= 0.0) {
    return {};
  }
  return {q.x / norm, q.y / norm, q.z / norm, q.w / norm};
}

SyncGazeboBlocksToWorldModel::Vector3 SyncGazeboBlocksToWorldModel::rotate(
  const Quaternion & q,
  const Vector3 & v)
{
  const auto rotated = multiply(multiply(q, {v.x, v.y, v.z, 0.0}), conjugate(q));
  return {rotated.x, rotated.y, rotated.z};
}

bool SyncGazeboBlocksToWorldModel::queryGazeboPose(
  const std::string & block_id,
  geometry_msgs::msg::Pose & pose)
{
  auto request = std::make_shared<GetEntityState::Request>();
  request->name = block_id;
  request->reference_frame = gazebo_reference_frame_;

  auto future = gazebo_client_->async_send_request(request);
  const auto result_code = callback_group_executor_.spin_until_future_complete(
    future,
    service_timeout_);
  if (result_code != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "GetEntityState timed out for '%s'", block_id.c_str());
    return false;
  }

  const auto response = future.get();
  if (!response->success) {
    RCLCPP_ERROR(node_->get_logger(), "GetEntityState failed for '%s'", block_id.c_str());
    return false;
  }

  pose = response->state.pose;
  return true;
}

bool SyncGazeboBlocksToWorldModel::upsertBlock(
  const std::string & block_id,
  const geometry_msgs::msg::Pose & pose)
{
  auto request = std::make_shared<UpsertBlock::Request>();
  request->block_id = block_id;
  request->pose = pose;
  request->frame_id = world_frame_id_;
  request->pose_status = pose_status_;
  request->task_status = task_status_;
  request->confidence = static_cast<float>(confidence_);

  auto future = upsert_client_->async_send_request(request);
  const auto result_code = callback_group_executor_.spin_until_future_complete(
    future,
    service_timeout_);
  if (result_code != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "UpsertBlock timed out for '%s'", block_id.c_str());
    return false;
  }

  const auto response = future.get();
  if (!response->success) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "UpsertBlock rejected '%s': %s",
      block_id.c_str(),
      response->message.c_str());
    return false;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "Upserted '%s' from Gazebo pose into '%s': (%.3f, %.3f, %.3f)",
    block_id.c_str(),
    world_frame_id_.c_str(),
    pose.position.x,
    pose.position.y,
    pose.position.z);
  return true;
}

geometry_msgs::msg::Pose SyncGazeboBlocksToWorldModel::gazeboPoseToWorldModelPose(
  const geometry_msgs::msg::Pose & gazebo_pose) const
{
  const Vector3 gazebo_position{
    gazebo_pose.position.x,
    gazebo_pose.position.y,
    gazebo_pose.position.z};
  const Vector3 relative{
    gazebo_position.x - gazebo_seed_frame_xyz_.x,
    gazebo_position.y - gazebo_seed_frame_xyz_.y,
    gazebo_position.z - gazebo_seed_frame_xyz_.z};

  const auto seed_inv = conjugate(gazebo_seed_frame_quat_);
  const auto seed_position = rotate(seed_inv, relative);
  const auto gazebo_quat = normalize({
    gazebo_pose.orientation.x,
    gazebo_pose.orientation.y,
    gazebo_pose.orientation.z,
    gazebo_pose.orientation.w});
  const auto seed_quat = normalize(multiply(seed_inv, gazebo_quat));

  geometry_msgs::msg::Pose pose;
  pose.position.x = seed_position.x;
  pose.position.y = seed_position.y;
  pose.position.z = seed_position.z;
  pose.orientation.x = seed_quat.x;
  pose.orientation.y = seed_quat.y;
  pose.orientation.z = seed_quat.z;
  pose.orientation.w = seed_quat.w;
  return pose;
}

}  // namespace concrete_block_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<concrete_block_behavior_tree::SyncGazeboBlocksToWorldModel>(
    "SyncGazeboBlocksToWorldModel");
}
