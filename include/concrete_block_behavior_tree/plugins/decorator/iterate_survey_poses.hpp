#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "behaviortree_cpp/decorator_node.h"
#include "rclcpp/rclcpp.hpp"

namespace concrete_block_behavior_tree
{

// Ticks its child once per configured survey pose, writing that pose first: the child moves
// there and runs SCENE_DISCOVERY, so the site is known before the first task is asked for
// (issue 323). The poses come from the BT server's own `survey_poses.<survey_site>.{x,y,z,yaw}`
// parameters (config/survey_sites.yaml), so a site is picked with a launch argument instead of
// an edited tree. No `survey_site` means no survey and the child never runs; a site whose
// poses are missing is a misconfiguration and fails.
//
// `blocks` is what the discovery under the child reported, and it counts the whole world
// snapshot: a pose that does not raise it found nothing new, which is a warning, and zero
// after the last pose is a site with nothing to pick, which ends the goal (303).
class IterateSurveyPoses : public BT::DecoratorNode
{
public:
  IterateSurveyPoses(const std::string & name, const BT::NodeConfig & conf);

  BT::NodeStatus tick() override;

  void halt() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("blocks", "Blocks the discovery under the child knows of"),
      BT::OutputPort<double>("survey_x", "Survey hover x, world"),
      BT::OutputPort<double>("survey_y", "Survey hover y, world"),
      BT::OutputPort<double>("survey_z", "Survey hover z, world"),
      BT::OutputPort<double>("survey_yaw", "Tool yaw at the hover, rad"),
    };
  }

private:
  struct Pose
  {
    double x;
    double y;
    double z;
    double yaw;
  };

  std::vector<double> array_param(const std::string & name);

  rclcpp::Node::SharedPtr node_;
  std::string site_;
  std::string config_error_;
  std::vector<Pose> poses_;
  std::size_t index_{0};
  int blocks_{0};
  int empty_{0};
};

}  // namespace concrete_block_behavior_tree
