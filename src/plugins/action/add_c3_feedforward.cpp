#include "concrete_block_behavior_tree/plugins/action/add_c3_feedforward.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "concrete_block_behavior_tree/c3_feedforward_math.hpp"

namespace concrete_block_behavior_tree
{

namespace
{
constexpr char kRobotDescriptionTopic[] = "/robot_description";
}  // namespace

AddC3Feedforward::AddC3Feedforward(const std::string & name, const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  // The parameters live on the BT server so the numbers stay next to the gains they pair with
  // (config/bt_server_override.yaml). k_i is from wiki/diagrams/hydraulic_calibration/
  // c3_full_model.json; no default is compiled in, because a wrong stiffness is a wrong command.
  const auto declare = [this](const std::string & name_, const auto & fallback) {
      return node_->has_parameter(name_) ? node_->get_parameter(name_).get_parameter_value()
             : node_->declare_parameter(name_, rclcpp::ParameterValue(fallback));
    };

  enabled_ = declare("c3_feedforward.enabled", false).get<bool>();
  dead_time_s_ = declare("c3_feedforward.dead_time_s", 0.06).get<double>();
  const auto joints =
    declare("c3_feedforward.joints", std::vector<std::string>{}).get<std::vector<std::string>>();
  const auto k = declare("c3_feedforward.k", std::vector<double>{}).get<std::vector<double>>();
  const auto u_min =
    declare("c3_feedforward.u_min", std::vector<double>{}).get<std::vector<double>>();
  const auto u_max =
    declare("c3_feedforward.u_max", std::vector<double>{}).get<std::vector<double>>();

  if (joints.size() != k.size() || joints.size() != u_min.size() ||
    joints.size() != u_max.size())
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "c3_feedforward.{joints,k,u_min,u_max} are read as parallel lists and their lengths differ "
      "(%zu/%zu/%zu/%zu); feedforward off",
      joints.size(), k.size(), u_min.size(), u_max.size());
    enabled_ = false;
    return;
  }
  if (dead_time_s_ < 0.0) {
    RCLCPP_ERROR(node_->get_logger(), "c3_feedforward.dead_time_s is negative; feedforward off");
    enabled_ = false;
    return;
  }

  inv_k_.assign(joints.size(), 0.0);
  for (size_t i = 0; i < k.size(); ++i) {
    if (k[i] <= 0.0) {
      RCLCPP_ERROR(
        node_->get_logger(), "c3_feedforward.k[%zu] = %f is not positive; feedforward off", i,
        k[i]);
      enabled_ = false;
      return;
    }
    if (!(u_min[i] < 0.0 && u_max[i] > 0.0)) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "c3_feedforward bound [%f, %f] on '%s' does not contain zero; feedforward off", u_min[i],
        u_max[i], joints[i].c_str());
      enabled_ = false;
      return;
    }
    inv_k_[i] = 1.0 / k[i];
  }
  parameter_joints_ = joints;
  u_min_ = u_min;
  u_max_ = u_max;

  // Own callback group, so the description subscription is spun by this node's executor and never
  // by rclcpp::spin_some on the shared client node - which the BT server already spins from its
  // own timer, and which throws when a second executor touches it.
  callback_group_ =
    node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  description_sub_ = node_->create_subscription<std_msgs::msg::String>(
    kRobotDescriptionTopic, rclcpp::QoS(1).transient_local(),
    [this](const std_msgs::msg::String & msg) {robot_description_ = msg.data;}, options);
}

bool AddC3Feedforward::ensure_model()
{
  if (model_) {
    return true;
  }

  if (robot_description_.empty()) {
    // transient-local, so the latched description arrives on the first spin of our own group
    callback_group_executor_.spin_some();
    if (robot_description_.empty()) {
      return false;
    }
  }

  crane_model::ModelConfig config;
  config.robot_description_xml = robot_description_;
  auto result = crane_model::Model::create(config);
  if (!result.ok()) {
    RCLCPP_ERROR(
      node_->get_logger(), "crane_model rejected the robot description: %s",
      result.status().message.c_str());
    return false;
  }
  model_ = std::make_unique<crane_model::Model>(std::move(result).value());

  // reorder the per-joint values into the model's contract order
  const auto & contract = model_->urdf_joint_names();
  std::vector<double> k(contract.size(), 0.0);
  std::vector<double> lower(contract.size(), -std::numeric_limits<double>::infinity());
  std::vector<double> upper(contract.size(), std::numeric_limits<double>::infinity());
  for (size_t p = 0; p < parameter_joints_.size(); ++p) {
    const auto it = std::find(contract.begin(), contract.end(), parameter_joints_[p]);
    if (it == contract.end()) {
      RCLCPP_ERROR(
        node_->get_logger(), "c3_feedforward.joints carries '%s', which the model does not have",
        parameter_joints_[p].c_str());
      model_.reset();
      return false;
    }
    const auto index = static_cast<size_t>(std::distance(contract.begin(), it));
    k[index] = inv_k_[p];
    lower[index] = u_min_[p];
    upper[index] = u_max_[p];
  }
  inv_k_ = k;
  u_min_ = lower;
  u_max_ = upper;
  return true;
}

bool AddC3Feedforward::build_index_map(
  const JointTrajectory & trajectory, std::vector<int> & traj_to_model) const
{
  const auto & contract = model_->urdf_joint_names();
  traj_to_model.assign(trajectory.joint_names.size(), -1);
  size_t matched = 0;
  for (size_t t = 0; t < trajectory.joint_names.size(); ++t) {
    const auto it = std::find(contract.begin(), contract.end(), trajectory.joint_names[t]);
    if (it != contract.end()) {
      traj_to_model[t] = static_cast<int>(std::distance(contract.begin(), it));
      ++matched;
    }
  }
  // RNEA needs the whole configuration, the two passive coordinates included
  return matched == contract.size();
}

BT::NodeStatus AddC3Feedforward::tick()
{
  JointTrajectory trajectory;
  if (!getInput("trajectory", trajectory)) {
    RCLCPP_ERROR(node_->get_logger(), "AddC3Feedforward: no trajectory on the blackboard");
    return BT::NodeStatus::FAILURE;
  }

  const auto pass_through = [this, &trajectory](const char * why) {
      if (!warned_) {
        RCLCPP_WARN(node_->get_logger(), "C3 feedforward not applied: %s", why);
        warned_ = true;
      }
      // hand the trajectory on untouched: without an effort field the controller falls back to
      // its static velocity feedforward, which is what runs today
      for (auto & point : trajectory.points) {
        point.effort.clear();
      }
      setOutput("trajectory", trajectory);
      return BT::NodeStatus::SUCCESS;
    };

  if (!enabled_) {
    return pass_through("c3_feedforward.enabled is false");
  }
  if (!ensure_model()) {
    return pass_through("no usable robot description yet");
  }

  const auto & points = trajectory.points;
  if (points.size() < 3) {
    return pass_through("fewer than three points, so tau_dot has no difference to take");
  }

  std::vector<int> traj_to_model;
  if (!build_index_map(trajectory, traj_to_model)) {
    return pass_through("the trajectory does not carry all eight generalized coordinates");
  }

  crane_model::Payload payload;
  bool carries = false;
  getInput("carries", carries);
  if (carries) {
    getInput("mass", payload.mass_kg);
    getInput("s_log_8_x", payload.center_of_mass_k8_m.x());
    getInput("s_log_8_y", payload.center_of_mass_k8_m.y());
    getInput("s_log_8_z", payload.center_of_mass_k8_m.z());
    // inertia stays zero: nothing on the blackboard carries it, and a point mass at the measured
    // CoM is the payload the A2B planner is given as well
    payload.valid = payload.center_of_mass_k8_m.allFinite() && std::isfinite(payload.mass_kg);
    if (!payload.valid) {
      return pass_through("the payload on the blackboard is not finite");
    }
  }

  // sort the plan into contract order
  const size_t n = points.size();
  const size_t m = crane_model::kGeneralizedDof;
  FeedforwardPlan plan;
  plan.time.resize(n);
  plan.dq_d.assign(n, std::vector<double>(m, 0.0));
  std::vector<std::vector<double>> q(n, std::vector<double>(m, 0.0));
  std::vector<std::vector<double>> ddq(n, std::vector<double>(m, 0.0));
  bool has_accelerations = true;
  for (size_t s = 0; s < n; ++s) {
    const auto & point = points[s];
    if (point.positions.size() != trajectory.joint_names.size() ||
      point.velocities.size() != trajectory.joint_names.size())
    {
      return pass_through("a point is missing positions or velocities");
    }
    has_accelerations =
      has_accelerations && point.accelerations.size() == trajectory.joint_names.size();
    plan.time[s] = rclcpp::Duration(point.time_from_start).seconds();
    for (size_t t = 0; t < traj_to_model.size(); ++t) {
      const int j = traj_to_model[t];
      if (j < 0) {
        continue;
      }
      q[s][static_cast<size_t>(j)] = point.positions[t];
      plan.dq_d[s][static_cast<size_t>(j)] = point.velocities[t];
      if (has_accelerations) {
        ddq[s][static_cast<size_t>(j)] = point.accelerations[t];
      }
    }
  }
  // Both A2B servers send no accelerations on purpose (a2b_server_base.cpp:615), so this is the
  // normal path rather than the fallback. Cost: tau_dot then rests on a second difference of the
  // planned velocity.
  if (!has_accelerations) {
    ddq = differentiate(plan.time, plan.dq_d);
  }

  plan.tau.assign(n, std::vector<double>(m, 0.0));
  for (size_t s = 0; s < n; ++s) {
    crane_model::Q q_s = crane_model::Q::Zero();
    crane_model::DQ dq_s = crane_model::DQ::Zero();
    crane_model::DQ ddq_s = crane_model::DQ::Zero();
    for (size_t j = 0; j < m; ++j) {
      q_s[static_cast<int>(j)] = q[s][j];
      dq_s[static_cast<int>(j)] = plan.dq_d[s][j];
      ddq_s[static_cast<int>(j)] = ddq[s][j];
    }
    auto result = model_->inverse_dynamics(q_s, dq_s, ddq_s, payload);
    if (!result.ok()) {
      return pass_through(result.status().message.c_str());
    }
    for (size_t j = 0; j < m; ++j) {
      plan.tau[s][j] = result.value()[static_cast<int>(j)];
    }
  }

  plan.inv_k = inv_k_;
  plan.u_min = u_min_;
  plan.u_max = u_max_;
  plan.dead_time_s = dead_time_s_;

  bool saturated = false;
  const auto effort = compute_feedforward(plan, &saturated);
  if (saturated && !saturation_warned_) {
    // the bound is Psi's identified domain: past it the feedforward is asking for a velocity the
    // compensator was never fitted on (control_architecture.md §2.1, controller_design.md §4.4)
    RCLCPP_WARN(
      node_->get_logger(),
      "C3 feedforward saturated against the identified command domain; the plan asks for more "
      "third derivative than the axis has authority for");
    saturation_warned_ = true;
  }

  for (size_t s = 0; s < n; ++s) {
    auto & point = trajectory.points[s];
    point.effort.assign(trajectory.joint_names.size(), 0.0);
    for (size_t t = 0; t < traj_to_model.size(); ++t) {
      const int j = traj_to_model[t];
      if (j >= 0) {
        point.effort[t] = effort[s][static_cast<size_t>(j)];
      }
    }
  }

  setOutput("trajectory", trajectory);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace concrete_block_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<concrete_block_behavior_tree::AddC3Feedforward>("AddC3Feedforward");
}
