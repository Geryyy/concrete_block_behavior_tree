#include "concrete_block_behavior_tree/nodes/gripper_grasp_detector.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<concrete_block_behavior_tree::GripperGraspDetector>());
  rclcpp::shutdown();
  return 0;
}
