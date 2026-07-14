#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"

namespace concrete_block_behavior_tree
{

class GripperGraspDetector : public rclcpp::Node
{
public:
  GripperGraspDetector()
  : Node("gripper_grasp_detector")
  {
    joint_states_topic_ = declare_parameter<std::string>("joint_states_topic", "/joint_states");
    output_topic_ = declare_parameter<std::string>("output_topic", "/gripper/grasp_detected");
    q9_joint_name_ = declare_parameter<std::string>("q9_joint_name", "q9_left_rail_joint");
    position_min_ = declare_parameter<double>("position_window.min", -std::numeric_limits<double>::infinity());
    position_max_ = declare_parameter<double>("position_window.max", std::numeric_limits<double>::infinity());
    effort_threshold_ = declare_parameter<double>("effort_threshold", 1000.0);
    hold_time_s_ = declare_parameter<double>("hold_time_s", 0.5);
    signal_duration_s_ = declare_parameter<double>("signal_duration_s", 1.0);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 20.0);

    validateParameters();

    grasp_pub_ = create_publisher<std_msgs::msg::Bool>(output_topic_, rclcpp::QoS(10).transient_local());
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic_,
      rclcpp::QoS(100),
      std::bind(&GripperGraspDetector::jointStateCallback, this, std::placeholders::_1));

    const auto publish_period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(publish_period),
      [this]() { publishSignal(); });

    RCLCPP_INFO(
      get_logger(),
      "Gripper grasp detector | topic=%s output=%s q9=%s position=[%.4f, %.4f] "
      "effort_threshold=%.3f hold_time_s=%.3f signal_duration_s=%.3f",
      joint_states_topic_.c_str(),
      output_topic_.c_str(),
      q9_joint_name_.c_str(),
      position_min_,
      position_max_,
      effort_threshold_,
      hold_time_s_,
      signal_duration_s_);

    publishSignal();
  }

private:
  using JointState = sensor_msgs::msg::JointState;

  void validateParameters()
  {
    if (q9_joint_name_.empty()) {
      throw std::invalid_argument("q9_joint_name must not be empty");
    }
    if (position_min_ > position_max_) {
      throw std::invalid_argument("position_window.min must be <= position_window.max");
    }
    if (effort_threshold_ < 0.0) {
      throw std::invalid_argument("effort_threshold must be >= 0");
    }
    if (hold_time_s_ < 0.0) {
      throw std::invalid_argument("hold_time_s must be >= 0");
    }
    if (signal_duration_s_ < 0.0) {
      throw std::invalid_argument("signal_duration_s must be >= 0");
    }
    if (publish_rate_hz_ <= 0.0) {
      throw std::invalid_argument("publish_rate_hz must be > 0");
    }
  }

  rclcpp::Time sampleTime(const JointState & msg) const
  {
    if (msg.header.stamp.sec == 0 && msg.header.stamp.nanosec == 0) {
      return now();
    }
    return rclcpp::Time(msg.header.stamp);
  }

  void jointStateCallback(const JointState::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    const auto it = std::find(msg->name.begin(), msg->name.end(), q9_joint_name_);
    if (it == msg->name.end()) {
      return;
    }

    const size_t idx = static_cast<size_t>(std::distance(msg->name.begin(), it));
    if (idx >= msg->position.size() || idx >= msg->effort.size()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "JointState for '%s' is missing position or effort at index %zu",
        q9_joint_name_.c_str(),
        idx);
      return;
    }

    const auto stamp = sampleTime(*msg);
    if (have_last_sample_time_ && stamp < last_sample_time_) {
      resetCandidate();
      signal_active_ = false;
      signal_until_ = stamp;
      RCLCPP_INFO(get_logger(), "JointState time moved backward; reset grasp detector state");
    }
    have_last_sample_time_ = true;
    last_sample_time_ = stamp;

    const double position = msg->position[idx];
    const double effort = msg->effort[idx];
    const bool in_window = position >= position_min_ && position <= position_max_;
    const bool above_effort = std::abs(effort) >= effort_threshold_;
    const bool candidate = in_window && above_effort;

    if (!candidate) {
      resetCandidate();
      return;
    }

    if (!candidate_active_) {
      candidate_active_ = true;
      candidate_since_ = stamp;
    }

    const double held_s = (stamp - candidate_since_).seconds();
    if (held_s >= hold_time_s_) {
      signal_until_ = stamp + rclcpp::Duration::from_seconds(signal_duration_s_);
      signal_active_ = true;
      if (!triggered_for_candidate_) {
        triggered_for_candidate_ = true;
        RCLCPP_INFO(
          get_logger(),
          "Grasp detected | q9 position=%.4f effort=%.3f held_s=%.3f signal_duration_s=%.3f",
          position,
          effort,
          held_s,
          signal_duration_s_);
      }
      publishSignal(stamp);
    }
  }

  void publishSignal()
  {
    publishSignal(now());
  }

  void publishSignal(const rclcpp::Time & stamp)
  {
    std_msgs::msg::Bool msg;
    msg.data = signal_active_ && stamp <= signal_until_;
    grasp_pub_->publish(msg);
  }

  void resetCandidate()
  {
    candidate_active_ = false;
    triggered_for_candidate_ = false;
  }

  std::string joint_states_topic_;
  std::string output_topic_;
  std::string q9_joint_name_;
  double position_min_{-std::numeric_limits<double>::infinity()};
  double position_max_{std::numeric_limits<double>::infinity()};
  double effort_threshold_{1000.0};
  double hold_time_s_{0.5};
  double signal_duration_s_{1.0};
  double publish_rate_hz_{20.0};

  bool candidate_active_{false};
  bool triggered_for_candidate_{false};
  bool signal_active_{false};
  bool have_last_sample_time_{false};
  rclcpp::Time candidate_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_sample_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time signal_until_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr grasp_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace concrete_block_behavior_tree

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<concrete_block_behavior_tree::GripperGraspDetector>());
  rclcpp::shutdown();
  return 0;
}
