#include <cstdint>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/decorator_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace concrete_block_behavior_tree
{

class WaitForOperatorCommand : public BT::StatefulActionNode
{
public:
  WaitForOperatorCommand(const std::string & name, const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config), node_(config.blackboard->get<rclcpp::Node::SharedPtr>("node"))
  {
    getInput("command_topic", command_topic_);
    getInput("state_topic", state_topic_);
    if (command_topic_.empty() || state_topic_.empty()) {
      throw std::invalid_argument("WaitForOperatorCommand topics must not be empty");
    }
    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;
    command_sub_ = node_->create_subscription<std_msgs::msg::String>(
      command_topic_, rclcpp::QoS(1).transient_local(),
      [this](const std_msgs::msg::String::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_command_ = message->data;
        ++command_sequence_;
      }, options);
    state_pub_ = node_->create_publisher<std_msgs::msg::String>(
      state_topic_, rclcpp::QoS(1).transient_local());
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "expected", "", "Command required to continue; use '|' to accept a decision set"),
      BT::InputPort<std::string>(
        "failure_commands", "", "Accepted commands that return FAILURE; use '|' to separate"),
      BT::OutputPort<std::string>("received", "Accepted operator command"),
      BT::OutputPort<bool>(
        "exit_requested", "True only when a failure_command deliberately exits a workflow loop"),
      BT::InputPort<std::string>(
        "command_topic", "/assembly_operator/command", "Operator command topic"),
      BT::InputPort<std::string>(
        "state_topic", "/assembly_operator/state", "Workflow state topic")};
  }

  BT::NodeStatus onStart() override
  {
    if (!getInput("expected", expected_) || expected_.empty()) {
      throw std::invalid_argument("WaitForOperatorCommand requires a non-empty expected command");
    }
    allowed_commands_.clear();
    failure_commands_.clear();
    std::istringstream commands(expected_);
    std::string command;
    while (std::getline(commands, command, '|')) {
      if (!command.empty()) {
        allowed_commands_.push_back(command);
      }
    }
    if (allowed_commands_.empty()) {
      throw std::invalid_argument("WaitForOperatorCommand has no valid expected command");
    }
    std::string failure_commands;
    getInput("failure_commands", failure_commands);
    std::istringstream failure_stream(failure_commands);
    while (std::getline(failure_stream, command, '|')) {
      if (!command.empty()) {
        failure_commands_.push_back(command);
      }
    }
    // The command publisher is transient-local so a newly-created wait node
    // receives the last UI command.  Drain that cached sample and only accept
    // a command issued after this particular wait began.  This makes commands
    // edge-triggered: a prior "correct" or "place" cannot advance a later
    // workflow stage.
    executor_.spin_some();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      command_sequence_at_start_ = command_sequence_;
    }
    setOutput("exit_requested", false);
    publishState();
    return onRunning();
  }

  BT::NodeStatus onRunning() override
  {
    executor_.spin_some();
    std::lock_guard<std::mutex> lock(mutex_);
    if (command_sequence_ <= command_sequence_at_start_) {
      publishState();
      return BT::NodeStatus::RUNNING;
    }
    for (const auto & command : failure_commands_) {
      if (last_command_ == command) {
        setOutput("received", last_command_);
        setOutput("exit_requested", true);
        return BT::NodeStatus::FAILURE;
      }
    }
    for (const auto & command : allowed_commands_) {
      if (last_command_ == command) {
        setOutput("received", last_command_);
        setOutput("exit_requested", false);
        return BT::NodeStatus::SUCCESS;
      }
    }
    publishState();
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {}

private:
  void publishState()
  {
    std_msgs::msg::String message;
    message.data = "waiting:" + expected_;
    for (const auto & command : failure_commands_) {
      message.data += "|" + command;
    }
    state_pub_->publish(message);
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  std::mutex mutex_;
  std::string command_topic_;
  std::string state_topic_;
  std::string expected_;
  std::vector<std::string> allowed_commands_;
  std::vector<std::string> failure_commands_;
  std::string last_command_;
  std::uint64_t command_sequence_{0};
  std::uint64_t command_sequence_at_start_{0};
};

// KeepRunningUntilFailure is useful for an operator-controlled refinement
// loop, but an ordinary Sequence treats its FAILURE as an assembly failure.
// This decorator converts that failure to SUCCESS only when the waiter
// explicitly recorded a fresh operator "place" command.  Sensor, planning,
// and motion failures therefore still abort the workflow normally.
class PlacementExitAsSuccess : public BT::DecoratorNode
{
public:
  PlacementExitAsSuccess(const std::string & name, const BT::NodeConfiguration & config)
  : BT::DecoratorNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<bool>("exit_requested", false, "Explicit placement-loop exit")};
  }

  BT::NodeStatus tick() override
  {
    const auto status = child_node_->executeTick();
    if (status != BT::NodeStatus::FAILURE) {
      return status;
    }
    bool exit_requested = false;
    getInput("exit_requested", exit_requested);
    return exit_requested ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

}  // namespace concrete_block_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<concrete_block_behavior_tree::WaitForOperatorCommand>(
    "WaitForOperatorCommand");
  factory.registerNodeType<concrete_block_behavior_tree::PlacementExitAsSuccess>(
    "PlacementExitAsSuccess");
}
