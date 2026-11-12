// BT.CPP rejects an attribute that names no declared port, and it does so while the tree is
// being constructed, not while the XML is parsed. So loading a tree is the only check that
// catches a port rename in the epsilon plugins (issue 295).

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_prefix.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/bt_factory.h"
// For convertFromString<milliseconds>: the blackboard entry below carries the converter of
// the TU that created it, and without this the trees' server_timeout="10000" is unparsable.
#include "nav2_behavior_tree/bt_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace
{

namespace fs = std::filesystem;

// Every libBT_*.so of these, not bt_server_override.yaml's plugin_lib_names: a superset
// registers the same node models and does not rot when that list changes.
const char * kPluginPackages[] = {
  "concrete_block_behavior_tree", "epsilon_crane_behavior_tree", "lsrl_behavior_tree"};

std::string read_file(const fs::path & path)
{
  std::ifstream in(path);
  return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

std::vector<fs::path> xml_files(const fs::path & dir)
{
  std::vector<fs::path> files;
  for (const auto & entry : fs::directory_iterator(dir)) {
    if (entry.path().extension() == ".xml") {
      files.push_back(entry.path());
    }
  }
  std::sort(files.begin(), files.end());
  return files;
}

// Replaces every leaf plugin node with a TestNode, so no service or action server has to be
// up. Only the leaves: a substituted decorator or SubTree would take its children out of the
// tree, and those children are most of what is being checked. `failing`, if given, is the one
// node that returns FAILURE instead -- the step whose measurement never arrived.
//
// `with_decorators` adds the plugin decorators, which a *ticked* tree cannot keep: the epsilon
// ones wait for a message (CheckUserApproval is a GetMsgCondition on
// /gpio_controller/remote_ctrl_states, and the wait_*_msg family likewise) and stay RUNNING
// for ever with nothing publishing. It drops their children with them, so the construction
// test above leaves them in.
//
// `flaky` fails `flaky_failures` ticks, succeeds on the next and repeats -- a /tf stall that
// clears and comes back (issue 320). One counter for every instance in the tree, so the
// pattern has to recur: a stub that only fails once is absorbed by the first call site and the
// later ones never see a failure at all.
void stub_out_leaf_plugins(
  BT::BehaviorTreeFactory & factory, const char * failing = nullptr,
  bool with_decorators = false, const char * flaky = nullptr, int flaky_failures = 0)
{
  std::set<std::string> builtins;
  for (const auto & [name, manifest] : factory.manifests()) {
    builtins.insert(name);
  }
  for (const auto & package : kPluginPackages) {
    const fs::path lib_dir = fs::path(ament_index_cpp::get_package_prefix(package)) / "lib";
    for (const auto & entry : fs::directory_iterator(lib_dir)) {
      const std::string name = entry.path().filename().string();
      if (name.rfind("libBT_", 0) == 0 && entry.path().extension() == ".so") {
        factory.registerFromPlugin(entry.path().string());
      }
    }
  }
  BT::TestNodeConfig stub;
  stub.return_status = BT::NodeStatus::SUCCESS;
  BT::TestNodeConfig fails;
  fails.return_status = BT::NodeStatus::FAILURE;
  for (const auto & [name, manifest] : factory.manifests()) {
    if (builtins.count(name) != 0) {
      continue;
    }
    // PlanComplete reads nothing but the blackboard, and it is what tells a finished plan
    // from a failed task: stubbed to SUCCESS it reports every failure as a finished wall.
    if (name == "PlanComplete") {
      continue;
    }
    if (flaky != nullptr && name == flaky) {
      auto config = std::make_shared<BT::TestNodeConfig>();
      auto ticks = std::make_shared<int>(0);
      config->complete_func = [ticks, flaky_failures]() {
          const bool fails = (*ticks)++ % (flaky_failures + 1) < flaky_failures;
          return fails ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
        };
      factory.addSubstitutionRule(name, config);
      continue;
    }
    if (manifest.type == BT::NodeType::ACTION || manifest.type == BT::NodeType::CONDITION ||
      (with_decorators && manifest.type == BT::NodeType::DECORATOR))
    {
      factory.addSubstitutionRule(name, failing != nullptr && name == failing ? fails : stub);
    }
  }
}

BT::Blackboard::Ptr server_blackboard(const rclcpp::Node::SharedPtr & node)
{
  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  // What nav2_behavior_tree's BtActionServer puts there; the values do not matter with the
  // leaves stubbed out, their presence does -- the goal-pose decorators read tf_buffer while
  // they are being constructed.
  blackboard->set<std::shared_ptr<tf2_ros::Buffer>>(
    "tf_buffer", std::make_shared<tf2_ros::Buffer>(node->get_clock()));
  blackboard->set<std::chrono::milliseconds>("server_timeout", std::chrono::milliseconds(10));
  blackboard->set<std::chrono::milliseconds>("bt_loop_duration", std::chrono::milliseconds(10));
  blackboard->set<std::chrono::milliseconds>(
    "wait_for_service_timeout", std::chrono::milliseconds(10));
  return blackboard;
}

fs::path trees_path()
{
  return fs::path(ament_index_cpp::get_package_share_directory("concrete_block_behavior_tree")) /
         "behavior_trees";
}

// What GetNextAssemblyTask and the steps before would have written. The stubs set no output
// port, so every entry a Script or a SetBlackboard reads has to be seeded here; a missing one
// throws, which is why each case below is also run with nothing failing.
void seed_task(const BT::Blackboard::Ptr & blackboard)
{
  for (const char * key : {"pickup_x", "pickup_y", "pickup_z", "pickup_yaw", "approach_z",
      "corrected_pickup_x", "corrected_pickup_y", "place_x", "place_y", "place_z", "place_yaw",
      "place_approach_x", "place_approach_y", "place_approach_z", "place_attempt",
      "seed_off_x", "seed_off_y", "meas_x", "meas_y", "meas_z", "s_log_8_x", "s_log_8_y",
      "s_log_8_z", "slow_down", "payload_mass"})
  {
    blackboard->set(key, 0.0);
  }
  blackboard->set("grasp_object_contains", std::string("block_0"));
  blackboard->set("moving_block_id", std::string());
  blackboard->set("log_shape_text", std::string("radius_top: 0.541, radius_bottom: 0.541"));
  // The block where the step aimed it: place_z + 0.45 with every place_* zero. Lower and
  // SubTreePlaceApproach's own height gate fails the pass, which is not what is under test.
  blackboard->set("meas_z", 0.45);
}

// Bounded in wall time, not in ticks: the /tf retry of issue 320 waits between its attempts,
// and a tick budget would spin through the wait in microseconds and report a RUNNING tree.
// A node left RUNNING still fails the test here instead of timing the whole binary out.
BT::NodeStatus tick_to_completion(BT::Tree & tree)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(25);
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  while (status == BT::NodeStatus::RUNNING && std::chrono::steady_clock::now() < deadline) {
    status = tree.tickExactlyOnce();
    tree.sleep(std::chrono::milliseconds(5));  // as the server does; a wake-up ends it early
  }
  return status;
}

BT::NodeStatus tick_subtree(
  const rclcpp::Node::SharedPtr & node, const std::string & tree_id, const char * failing,
  const char * flaky = nullptr, int flaky_failures = 0)
{
  BT::BehaviorTreeFactory factory;
  stub_out_leaf_plugins(factory, failing, true, flaky, flaky_failures);
  factory.registerBehaviorTreeFromFile((trees_path() / "wall_assembly.xml").string());
  auto blackboard = server_blackboard(node);
  seed_task(blackboard);
  auto tree = factory.createTree(tree_id, blackboard);
  return tick_to_completion(tree);
}

}  // namespace

TEST(TreeLoad, every_root_tree_constructs_with_the_installed_plugin_ports)
{
  const fs::path trees_dir =
    fs::path(ament_index_cpp::get_package_share_directory("concrete_block_behavior_tree")) /
    "behavior_trees";
  const auto trees = xml_files(trees_dir);
  ASSERT_FALSE(trees.empty()) << "no trees in " << trees_dir.string();

  auto node = std::make_shared<rclcpp::Node>("test_tree_load");

  std::vector<fs::path> roots;
  std::ostringstream failures;
  for (const auto & tree : trees) {
    // Only the files the panel can load have a main tree; the rest are subtrees, reached
    // through the includes of those, and covered by the loop below.
    if (read_file(tree).find("main_tree_to_execute") == std::string::npos) {
      continue;
    }
    roots.push_back(tree);
    BT::BehaviorTreeFactory factory;
    stub_out_leaf_plugins(factory);
    try {
      const auto loaded = factory.createTreeFromFile(tree.string(), server_blackboard(node));
      EXPECT_FALSE(loaded.subtrees.empty()) << tree.filename().string();
    } catch (const std::exception & e) {
      failures << "  " << tree.filename().string() << ": " << e.what() << '\n';
    }
  }
  ASSERT_FALSE(roots.empty()) << "no tree in " << trees_dir.string() << " has a main tree";
  EXPECT_TRUE(failures.str().empty()) << '\n' << failures.str();

  // The roots are the coverage: a subtree file no root includes would never be constructed.
  std::ostringstream orphans;
  for (const auto & tree : trees) {
    if (std::count(roots.begin(), roots.end(), tree) != 0) {
      continue;
    }
    const std::string include = "path=\"behavior_trees/" + tree.filename().string() + "\"";
    const bool included = std::any_of(
      roots.begin(), roots.end(),
      [&](const fs::path & root) {return read_file(root).find(include) != std::string::npos;});
    if (!included) {
      orphans << "  " << tree.filename().string() << '\n';
    }
  }
  EXPECT_TRUE(orphans.str().empty()) << "subtrees no root tree includes:\n" << orphans.str();
}

// Issue 303. On the lab day every refine after the first returned ok=0 and the ForceSuccess
// wrappers passed it on: the grip, the grasp offset and the placed pose all came from FK, and
// block_1 ended up on the studs. There is no path around a missing measurement any more, so
// the subtree that owns the step has to report the failure at its own root.
TEST(FailLoud, a_refine_that_returns_no_pose_fails_the_step_it_belongs_to)
{
  auto node = std::make_shared<rclcpp::Node>("test_fail_loud_refine");
  for (const char * tree_id : {"SubTreePickBlock", "SubTreePlaceApproach", "SubTreeRelease"}) {
    EXPECT_EQ(tick_subtree(node, tree_id, nullptr), BT::NodeStatus::SUCCESS) << tree_id;
    EXPECT_EQ(tick_subtree(node, tree_id, "RunPoseEstimation"), BT::NodeStatus::FAILURE)
      << tree_id;
  }
}

// The other half of 303: /tf froze, GetNextAssemblyTask could not transform the pickup, and
// the discovery fallback's Script read the pickup_x it never wrote -- an exception, not a
// failed goal. The fallback is for an answer without a pose, so plan_ok keeps it out of the
// error path, and PlanComplete must not read the aborted ask as a finished wall.
TEST(FailLoud, a_failing_get_next_assembly_task_fails_the_goal_without_an_exception)
{
  auto node = std::make_shared<rclcpp::Node>("test_fail_loud_task");
  BT::BehaviorTreeFactory factory;
  stub_out_leaf_plugins(factory, "GetNextAssemblyTask", true);
  auto tree = factory.createTreeFromFile(
    (trees_path() / "wall_assembly.xml").string(), server_blackboard(node));
  BT::NodeStatus status = BT::NodeStatus::IDLE;
  ASSERT_NO_THROW({status = tick_to_completion(tree);});
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

// Issue 320. OptimizeTargetAngle stamps now() and looks the TCP transform up: 21 s into a wall
// goal the transform stood still, both attempts of replan_on_missed_goal hit it 0.69 s apart
// and the goal aborted before the crane moved. Retrying the lookup alone, with wall time
// between the attempts, gets past a stall the publisher recovers from; a silent /tf must still
// fail the move. SubTreePickBlock reaches both call sites, SubTreeMoveAbove and
// SubTreeDescendTo, and three failures in a row at each is red without the retry at that one.
TEST(TfStall, a_transient_target_angle_failure_is_retried_and_a_persistent_one_fails)
{
  auto node = std::make_shared<rclcpp::Node>("test_tf_stall");
  // Three: more than replan_on_missed_goal's two attempts of the whole move can absorb, so
  // only a retry of the lookup itself gets past them, and the descent has no outer retry.
  EXPECT_EQ(
    tick_subtree(node, "SubTreePickBlock", nullptr, "OptimizeTargetAngle", 3),
    BT::NodeStatus::SUCCESS);
  EXPECT_EQ(
    tick_subtree(node, "SubTreePickBlock", "OptimizeTargetAngle"), BT::NodeStatus::FAILURE);
}

// Issue 327: with another block TASK_MOVE the pick would lift it as an empty hook and open the
// jaws over this one. GetBlockTaskStatus names it; the pick must not start.
TEST(NoDrop, a_pick_does_not_start_while_a_block_is_task_move)
{
  auto node = std::make_shared<rclcpp::Node>("test_no_drop");
  BT::BehaviorTreeFactory factory;
  stub_out_leaf_plugins(factory, nullptr, true);
  factory.registerBehaviorTreeFromFile((trees_path() / "wall_assembly.xml").string());
  auto blackboard = server_blackboard(node);
  seed_task(blackboard);
  blackboard->set("moving_block_id", std::string("block_5"));
  auto tree = factory.createTree("SubTreePickBlock", blackboard);
  EXPECT_EQ(tick_to_completion(tree), BT::NodeStatus::FAILURE);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
