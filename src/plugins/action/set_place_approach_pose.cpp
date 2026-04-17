#include "concrete_block_behavior_tree/plugins/action/set_place_approach_pose.hpp"

#include <cmath>

#include "behaviortree_cpp_v3/bt_factory.h"

namespace concrete_block_behavior_tree
{

SetPlaceApproachPose::SetPlaceApproachPose(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
}

BT::NodeStatus SetPlaceApproachPose::tick()
{
  double place_x = 0.0;
  double place_y = 0.0;
  double place_yaw = 0.0;
  double approach_z = 0.0;
  double place_approach_offset = 0.0;

  getInput("place_x", place_x);
  getInput("place_y", place_y);
  getInput("place_yaw", place_yaw);
  getInput("approach_z", approach_z);
  getInput("place_approach_offset", place_approach_offset);

  Point goal;
  goal.x = place_x + place_approach_offset * std::cos(place_yaw);
  goal.y = place_y + place_approach_offset * std::sin(place_yaw);
  goal.z = approach_z;

  setOutput("phi", place_yaw);
  setOutput("goal", goal);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace concrete_block_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<concrete_block_behavior_tree::SetPlaceApproachPose>(
    "SetPlaceApproachPose");
}
