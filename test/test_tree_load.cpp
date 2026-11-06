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
// tree, and those children are most of what is being checked.
void stub_out_leaf_plugins(BT::BehaviorTreeFactory & factory)
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
  for (const auto & [name, manifest] : factory.manifests()) {
    if (builtins.count(name) != 0) {
      continue;
    }
    if (manifest.type == BT::NodeType::ACTION || manifest.type == BT::NodeType::CONDITION) {
      factory.addSubstitutionRule(name, stub);
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

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
