#ifndef CONCRETE_BLOCK_BEHAVIOR_TREE__PLUGINS__ACTION__SET_PLACE_APPROACH_POSE_HPP_
#define CONCRETE_BLOCK_BEHAVIOR_TREE__PLUGINS__ACTION__SET_PLACE_APPROACH_POSE_HPP_

#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/point.hpp"

namespace concrete_block_behavior_tree
{

class SetPlaceApproachPose : public BT::SyncActionNode
{
public:
  using Point = geometry_msgs::msg::Point;

  SetPlaceApproachPose(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("place_x"),
      BT::InputPort<double>("place_y"),
      BT::InputPort<double>("place_yaw"),
      BT::InputPort<double>("approach_z"),
      BT::InputPort<double>("place_approach_offset", 0.0, "Lateral place approach offset"),
      BT::OutputPort<double>("phi"),
      BT::OutputPort<Point>("goal"),
    };
  }

private:
  BT::NodeStatus tick() override;
};

}  // namespace concrete_block_behavior_tree

#endif  // CONCRETE_BLOCK_BEHAVIOR_TREE__PLUGINS__ACTION__SET_PLACE_APPROACH_POSE_HPP_
