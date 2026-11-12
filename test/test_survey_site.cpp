// The site survey of issue 323: one pass per configured pose, and what the passes together
// found decides the goal. A site where nothing is found has nothing for the binding to choose
// from, so it ends the goal (303) instead of walking into the first ask empty-handed.

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "concrete_block_behavior_tree/plugins/decorator/iterate_survey_poses.hpp"
#include "rclcpp/rclcpp.hpp"

using concrete_block_behavior_tree::IterateSurveyPoses;

namespace
{

struct Outcome
{
  BT::NodeStatus status;
  int passes;   // how often the move-and-discover child ran
};

// `found_at_x` stands in for the discovery: the child reports a block only at the pose with
// that x, as the real one reports the world snapshot after SCENE_DISCOVERY; an x no pose has
// is a site where nothing is found. Two poses, which is the shape the acceptance asks about.
Outcome run(const std::string & site, double found_at_x)
{
  static int serial = 0;
  auto node = std::make_shared<rclcpp::Node>("test_survey_site_" + std::to_string(serial++));
  node->declare_parameter<std::string>("survey_site", site);
  node->declare_parameter<std::vector<double>>("survey_poses.storage.x", {1.0, 2.0});
  node->declare_parameter<std::vector<double>>("survey_poses.storage.y", {1.7, 3.5});
  node->declare_parameter<std::vector<double>>("survey_poses.storage.z", {3.141, 3.141});
  node->declare_parameter<std::vector<double>>("survey_poses.storage.yaw", {0.0, 0.0});

  auto blackboard = BT::Blackboard::create();
  blackboard->set<rclcpp::Node::SharedPtr>("node", node);
  blackboard->set("blocks", 0);
  blackboard->set("passes", 0);
  blackboard->set("found_at_x", found_at_x);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<IterateSurveyPoses>("IterateSurveyPoses");
  // R"xml(: the script's ternary ends in `)"`, which closes a plain raw string.
  auto tree = factory.createTreeFromText(
    R"xml(<root BTCPP_format="4"><BehaviorTree ID="T">
         <IterateSurveyPoses blocks="{blocks}" survey_x="{mv_x}" survey_y="{mv_y}"
           survey_z="{mv_z}" survey_yaw="{mv_yaw}">
           <Script code="passes := passes + 1; blocks := blocks + (mv_x == found_at_x ? 1 : 0)"/>
         </IterateSurveyPoses>
       </BehaviorTree></root>)xml", blackboard);

  const BT::NodeStatus status = tree.tickWhileRunning();
  return {status, blackboard->get<int>("passes")};
}

}  // namespace

TEST(SurveySite, an_unconfigured_site_moves_the_crane_nowhere)
{
  // Every launch but the one that loads survey_sites.yaml, and the operator trees: the run
  // has to be the run it was before 323.
  const auto outcome = run("", 1.0);
  EXPECT_EQ(outcome.status, BT::NodeStatus::SUCCESS);
  EXPECT_EQ(outcome.passes, 0);
}

TEST(SurveySite, a_site_where_nothing_is_found_at_any_pose_fails_the_goal)
{
  const auto outcome = run("storage", 99.0);
  EXPECT_EQ(outcome.status, BT::NodeStatus::FAILURE);
  EXPECT_EQ(outcome.passes, 2) << "both poses are tried before the verdict";
}

TEST(SurveySite, one_empty_pose_of_two_is_not_a_failure)
{
  for (const double found_at_x : {1.0, 2.0}) {
    const auto outcome = run("storage", found_at_x);
    EXPECT_EQ(outcome.status, BT::NodeStatus::SUCCESS) << found_at_x;
    EXPECT_EQ(outcome.passes, 2) << found_at_x;
  }
}

TEST(SurveySite, a_site_name_with_no_poses_is_a_misconfiguration_and_does_not_pass_silently)
{
  // survey_site:=typo would otherwise read as "no survey" and the run would start blind.
  EXPECT_EQ(run("not_a_site", 1.0).status, BT::NodeStatus::FAILURE);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
