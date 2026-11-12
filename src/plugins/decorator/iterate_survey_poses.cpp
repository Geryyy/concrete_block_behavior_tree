#include "concrete_block_behavior_tree/plugins/decorator/iterate_survey_poses.hpp"

#include <algorithm>
#include <string>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"

namespace concrete_block_behavior_tree
{

IterateSurveyPoses::IterateSurveyPoses(const std::string & name, const BT::NodeConfig & conf)
: BT::DecoratorNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  site_ = node_->has_parameter("survey_site")
    ? node_->get_parameter("survey_site").as_string()
    : node_->declare_parameter<std::string>("survey_site", "");
  if (site_.empty()) {
    return;
  }
  const std::string prefix = "survey_poses." + site_ + ".";
  const auto x = array_param(prefix + "x");
  const auto y = array_param(prefix + "y");
  const auto z = array_param(prefix + "z");
  const auto yaw = array_param(prefix + "yaw");
  if (!config_error_.empty()) {
    return;
  }
  if (x.empty()) {
    config_error_ = "survey_site is '" + site_ + "' and " + prefix + "x is empty";
    return;
  }
  if (y.size() != x.size() || z.size() != x.size() || yaw.size() != x.size()) {
    config_error_ = prefix + "{x,y,z,yaw} are not the same length";
    return;
  }
  for (std::size_t i = 0; i < x.size(); ++i) {
    poses_.push_back({x[i], y[i], z[i], yaw[i]});
  }
}

// Declared dynamically typed: an unset one is a site this run does not configure, not an
// error, and a statically typed declaration with an empty default throws instead.
std::vector<double> IterateSurveyPoses::array_param(const std::string & name)
{
  if (!node_->has_parameter(name)) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.dynamic_typing = true;
    node_->declare_parameter(name, rclcpp::ParameterValue{}, descriptor);
  }
  const rclcpp::Parameter value = node_->get_parameter(name);
  if (value.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
    return {};
  }
  if (value.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
    config_error_ = name + " is not a list of floats";   // `[0, 0]` is a list of ints
    return {};
  }
  return value.as_double_array();
}

BT::NodeStatus IterateSurveyPoses::tick()
{
  if (status() == BT::NodeStatus::IDLE) {
    index_ = 0;
    blocks_ = 0;
    empty_ = 0;
  }
  if (!config_error_.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "No site survey: %s", config_error_.c_str());
    return BT::NodeStatus::FAILURE;
  }
  if (poses_.empty()) {
    RCLCPP_INFO(node_->get_logger(), "No survey_site configured: no site survey");
    return BT::NodeStatus::SUCCESS;
  }

  if (index_ >= poses_.size()) {
    if (blocks_ == 0) {
      // 303: the binding has nothing to choose from, so the goal ends here and says why.
      RCLCPP_ERROR(
        node_->get_logger(),
        "Site survey of '%s' found no block at any of its %zu poses: nothing to pick",
        site_.c_str(), poses_.size());
      return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(
      node_->get_logger(), "Site survey of '%s': %d blocks known, %d of %zu poses found none",
      site_.c_str(), blocks_, empty_, poses_.size());
    return BT::NodeStatus::SUCCESS;
  }

  setStatus(BT::NodeStatus::RUNNING);
  const Pose & pose = poses_[index_];
  setOutput("survey_x", pose.x);
  setOutput("survey_y", pose.y);
  setOutput("survey_z", pose.z);
  setOutput("survey_yaw", pose.yaw);

  switch (child_node_->executeTick()) {
    case BT::NodeStatus::SUCCESS: {
      int blocks = blocks_;
      getInput("blocks", blocks);   // unwritten on the first pose: stays at the count so far
      if (blocks > blocks_) {
        RCLCPP_INFO(
          node_->get_logger(), "Survey pose %zu/%zu (%.2f, %.2f): %d blocks known",
          index_ + 1, poses_.size(), pose.x, pose.y, blocks);
      } else {
        ++empty_;
        RCLCPP_WARN(
          node_->get_logger(), "Survey pose %zu/%zu (%.2f, %.2f) found no block; %d known",
          index_ + 1, poses_.size(), pose.x, pose.y, blocks);
      }
      blocks_ = std::max(blocks_, blocks);
      ++index_;
      resetChild();   // the next pose runs the same move and discovery again
      return BT::NodeStatus::RUNNING;
    }
    case BT::NodeStatus::RUNNING:
      return BT::NodeStatus::RUNNING;
    default:
      return BT::NodeStatus::FAILURE;
  }
}

void IterateSurveyPoses::halt()
{
  index_ = 0;
  blocks_ = 0;
  empty_ = 0;
  BT::DecoratorNode::halt();
}

}  // namespace concrete_block_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<concrete_block_behavior_tree::IterateSurveyPoses>(
    "IterateSurveyPoses");
}
