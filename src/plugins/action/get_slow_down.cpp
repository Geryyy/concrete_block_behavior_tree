#include "concrete_block_behavior_tree/plugins/action/get_slow_down.hpp"

#include <cmath>

#include "behaviortree_cpp/bt_factory.h"

namespace concrete_block_behavior_tree
{

GetSlowDown::GetSlowDown(const std::string & name, const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::NodeStatus GetSlowDown::tick()
{
  const double slow_down = node_->has_parameter("slow_down")
    ? node_->get_parameter("slow_down").as_double()
    : node_->declare_parameter<double>("slow_down", 1.0);

  // Below one asks the planner to spend the authority `kappa` holds back for the
  // controller, and it refuses there -- on the first move, after the tree has started.
  if (!std::isfinite(slow_down) || slow_down < 1.0) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "GetSlowDown: slow_down is %f; it is a time stretch, so it is 1.0 or slower",
      slow_down);
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(node_->get_logger(), "GetSlowDown: every move paced at %.2fx", slow_down);
  setOutput("slow_down", slow_down);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace concrete_block_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<concrete_block_behavior_tree::GetSlowDown>("GetSlowDown");
}
