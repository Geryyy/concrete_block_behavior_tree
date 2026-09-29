// Relays gz /world/<world>/pose/info to a ROS TFMessage keeping each entity's name as
// child_frame_id. ros_gz_bridge's Pose_V -> TFMessage drops pose.name, so the entities
// arrive anonymous. Poses are in the parent frame: models in the world, links in their model.
// `names` (regex, full match) keeps only the entities a consumer needs.

#include <gz/msgs/pose_v.pb.h>

#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <memory>
#include <regex>
#include <string>

class GzPoseRelay : public rclcpp::Node
{
public:
  GzPoseRelay()
  : Node("gz_pose_relay")
  {
    const auto world = declare_parameter<std::string>("world", "default");
    names_ = std::regex(declare_parameter<std::string>("names", ".*"));
    pub_ = create_publisher<tf2_msgs::msg::TFMessage>("gz_poses", 10);
    gz_.Subscribe("/world/" + world + "/pose/info", &GzPoseRelay::relay, this);
  }

private:
  void relay(const gz::msgs::Pose_V & in)
  {
    tf2_msgs::msg::TFMessage out;
    out.transforms.reserve(in.pose_size());
    for (const auto & p : in.pose()) {
      if (!std::regex_match(p.name(), names_)) {
        continue;
      }
      auto & t = out.transforms.emplace_back();
      t.header.stamp.sec = static_cast<int32_t>(in.header().stamp().sec());
      t.header.stamp.nanosec = static_cast<uint32_t>(in.header().stamp().nsec());
      t.child_frame_id = p.name();
      t.transform.translation.x = p.position().x();
      t.transform.translation.y = p.position().y();
      t.transform.translation.z = p.position().z();
      t.transform.rotation.x = p.orientation().x();
      t.transform.rotation.y = p.orientation().y();
      t.transform.rotation.z = p.orientation().z();
      t.transform.rotation.w = p.orientation().w();
    }
    pub_->publish(out);
  }

  std::regex names_;
  gz::transport::Node gz_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GzPoseRelay>());
  rclcpp::shutdown();
  return 0;
}
