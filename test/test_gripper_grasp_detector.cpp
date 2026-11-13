// The grasp signal on the machine, replayed against the node over its own topics.
//
// Two failures of lab day 2026-11-09, both invisible in sim because the sim gripper holds its
// effort over the threshold for as long as it holds the block:
//  - the clamp closes the valve, so the measured effort decays to ~200 within a few seconds
//    (11-09 15-43-38). The signal expired `signal_duration_s` after the squeeze and
//    SubTreeResumeCarry could never pass again: every resume needed a manual reset. Now the
//    squeeze latches and q9 staying in the window holds it (framework review item 11).
//  - a free close runs past the window to ~0.0 at ~1.34e5 of effort, i.e. over the threshold,
//    and at the measured ~0.09 /s it spent ~1.6 s inside the old [0.05, 0.2]: longer than
//    `hold_time_s`, so an empty close fired. From 0.15 it is ~0.55 s (item 5).

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "concrete_block_behavior_tree/nodes/gripper_grasp_detector.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"

using concrete_block_behavior_tree::GripperGraspDetector;
using sensor_msgs::msg::JointState;

namespace
{

// q9 [m] and its effort, one /joint_states sample.
struct Sample
{
  double q9;
  double effort;
};

// Open stop, the machine clamp on a block and the free-closing effort, off the 11-09 bags.
constexpr double kOpen = 0.535;
constexpr double kClamped = 0.187;
constexpr double kCloseEffort = 1.34e5;
// What the effort decays to once the valve is shut on a held block.
constexpr double kHeldEffort = 200.0;
// How fast the jaws run: (0.2 - 0.15) / 0.55 s, from the free close of the same bags.
constexpr double kJawSpeed = 0.09;

// config/gripper_grasp_detector_real.yaml. `window_min` is the one value item 5 changes, so
// the empty-close case can be replayed against the old one too.
rclcpp::NodeOptions real_config(double window_min)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
  {
    {"position_window.min", window_min},
    {"position_window.max", 0.2},
    {"effort_threshold", 100000.0},
    {"hold_time_s", 1.0},
    {"signal_duration_s", 4.0},
    {"publish_rate_hz", 10.0},
  });
  return options;
}

// What /gripper/grasp_detected did over one replay: `ended` is its state after the last
// sample, `fired` whether it was ever true at all.
struct Signal
{
  bool ended;
  bool fired;
};

// Replays the trace at `dt` of sample-stamp time. The stamps run forward from now(), so the
// node's own publish timer -- which stamps with now() -- cannot outrun the trace and expire a
// live signal.
Signal replay(const std::vector<Sample> & trace, double dt, double window_min = 0.15)
{
  auto detector = std::make_shared<GripperGraspDetector>(real_config(window_min));
  auto probe = std::make_shared<rclcpp::Node>("grasp_probe");
  Signal signal{false, false};
  int received = 0;
  auto sub = probe->create_subscription<std_msgs::msg::Bool>(
    "/gripper/grasp_detected", rclcpp::QoS(10).transient_local(),
    [&](std_msgs::msg::Bool::SharedPtr msg) {
      signal.ended = msg->data;
      signal.fired = signal.fired || msg->data;
      ++received;
    });
  auto pub = probe->create_publisher<JointState>("/joint_states", rclcpp::QoS(100));
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(detector);
  executor.add_node(probe);

  const rclcpp::Time start = probe->now();
  for (size_t i = 0; i < trace.size(); ++i) {
    JointState state;
    state.header.stamp = start + rclcpp::Duration::from_seconds(static_cast<double>(i) * dt);
    // By name and not first: the stacks publish different name sets in different orders.
    state.name = {"theta6_tip_joint", "q9_left_rail_joint"};
    state.position = {0.0, trace[i].q9};
    state.effort = {0.0, trace[i].effort};
    pub->publish(state);
    executor.spin_some();
  }
  // The last sample only publishes when it is a candidate, so the state after an open or a
  // released close comes from the node's own 10 Hz timer: wait two of its periods out.
  const int before = received;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (received < before + 2 && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return signal;
}

// Open -> clamped on a block, 2 s of full effort, then `hold_s` of the decayed effort the
// valve leaves behind. dt 0.1 s, so 190 s of carry is 1900 samples of stamp time.
std::vector<Sample> carry(double hold_s)
{
  std::vector<Sample> trace;
  for (double q9 = kOpen; q9 > kClamped; q9 -= kJawSpeed * 0.1) {
    trace.push_back({q9, kCloseEffort});
  }
  for (int i = 0; i < 20; ++i) {
    trace.push_back({kClamped, kCloseEffort});
  }
  for (int i = 0; i < static_cast<int>(hold_s * 10.0); ++i) {
    trace.push_back({kClamped, kHeldEffort});
  }
  return trace;
}

}  // namespace

TEST(GripperGraspDetector, the_signal_survives_a_190_s_hold_on_a_decayed_effort)
{
  // 11-09 15-43-38: the signal dropped 4 s after the clamp, which is signal_duration_s.
  EXPECT_TRUE(replay(carry(190.0), 0.1).ended);
}

TEST(GripperGraspDetector, the_signal_goes_false_when_the_jaws_open_again)
{
  // The latch is held by q9, not by time: leaving the window drops it on the next sample,
  // which is what makes it safe to extend. Not `jaws_empty`, which also reads true when
  // the detector is dead.
  std::vector<Sample> trace = carry(190.0);
  for (double q9 = kClamped; q9 < kOpen; q9 += kJawSpeed * 0.1) {
    trace.push_back({q9, 0.0});
  }
  const Signal signal = replay(trace, 0.1);
  EXPECT_TRUE(signal.fired);
  EXPECT_FALSE(signal.ended);
}

TEST(GripperGraspDetector, a_free_close_never_fires)
{
  // The jaws run on past the window to ~0.0 at over the effort threshold, so what rejects
  // an empty close is the window's min alone: at 0.15 it is ~0.55 s inside, under the 1.0 s
  // hold. 0.05 is the value this failed at, and it is here to say why the number moved.
  std::vector<Sample> trace;
  for (double q9 = kOpen; q9 > 0.0; q9 -= kJawSpeed * 0.05) {
    trace.push_back({q9, kCloseEffort});
  }
  EXPECT_FALSE(replay(trace, 0.05).fired);
  EXPECT_TRUE(replay(trace, 0.05, 0.05).fired);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
