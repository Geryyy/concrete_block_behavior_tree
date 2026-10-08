cmake_minimum_required(VERSION 3.8)
project(concrete_block_behavior_tree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  behaviortree_cpp
  concrete_block_assembly_interfaces
  concrete_block_world_model_interfaces
  crane_msgs
  epsilon_crane_control_interfaces
  geometry_msgs
  lsrl_behavior_tree
  nav2_behavior_tree
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  trajectory_msgs
  visualization_msgs
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(include)

function(add_bt_plugin plugin_name source_file)
  add_library(${plugin_name} SHARED ${source_file})
  ament_target_dependencies(${plugin_name} ${ARGN})
  target_compile_definitions(${plugin_name} PRIVATE BT_PLUGIN_EXPORT)
  list(APPEND plugin_libs ${plugin_name})
  set(plugin_libs "${plugin_libs}" PARENT_SCOPE)
endfunction()

add_bt_plugin(BT_cb_get_next_assembly_task_action src/plugins/action/get_next_assembly_task.cpp ${THIS_PACKAGE_INCLUDE_DEPENDS})
add_bt_plugin(BT_cb_set_block_task_status_action src/plugins/action/set_block_task_status.cpp ${THIS_PACKAGE_INCLUDE_DEPENDS})
add_bt_plugin(BT_cb_wait_for_grasp_signal_condition src/plugins/condition/wait_for_grasp_signal.cpp ${THIS_PACKAGE_INCLUDE_DEPENDS})
add_bt_plugin(BT_cb_capture_block_grasp_offset_action src/plugins/action/capture_block_grasp_offset.cpp ${THIS_PACKAGE_INCLUDE_DEPENDS})
add_bt_plugin(BT_cb_write_block_pose_from_gripper_action
  src/plugins/action/write_block_pose_from_gripper.cpp ${THIS_PACKAGE_INCLUDE_DEPENDS})
add_bt_plugin(BT_cb_measure_block_from_gripper_action
  src/plugins/action/measure_block_from_gripper.cpp ${THIS_PACKAGE_INCLUDE_DEPENDS})
add_bt_plugin(BT_cb_plan_complete_condition src/plugins/condition/plan_complete.cpp ${THIS_PACKAGE_INCLUDE_DEPENDS})
add_bt_plugin(BT_cb_wait_for_move_complete_condition
  src/plugins/condition/wait_for_move_complete.cpp ${THIS_PACKAGE_INCLUDE_DEPENDS})
add_bt_plugin(BT_cb_wait_for_load_off_condition
  src/plugins/condition/wait_for_load_off.cpp ${THIS_PACKAGE_INCLUDE_DEPENDS})
add_bt_plugin(BT_cb_publish_mpc_reference_action
  src/plugins/action/publish_mpc_reference.cpp ${THIS_PACKAGE_INCLUDE_DEPENDS})
add_bt_plugin(BT_cb_wait_for_operator_command_action src/plugins/action/wait_for_operator_command.cpp ${THIS_PACKAGE_INCLUDE_DEPENDS})
add_bt_plugin(BT_cb_run_pose_estimation_action src/plugins/action/run_pose_estimation.cpp ${THIS_PACKAGE_INCLUDE_DEPENDS})
add_bt_plugin(BT_cb_evaluate_placement_residual_action src/plugins/action/evaluate_placement_residual.cpp ${THIS_PACKAGE_INCLUDE_DEPENDS})
add_bt_plugin(BT_cb_evaluate_pickup_residual_action src/plugins/action/evaluate_pickup_residual.cpp ${THIS_PACKAGE_INCLUDE_DEPENDS})
add_bt_plugin(BT_cb_publish_gripper_state_action src/plugins/action/publish_gripper_state.cpp ${THIS_PACKAGE_INCLUDE_DEPENDS})
add_bt_plugin(BT_cb_set_mpc_payload_action src/plugins/action/set_mpc_payload.cpp ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_executable(gripper_grasp_detector
  src/nodes/gripper_grasp_detector.cpp
)
ament_target_dependencies(gripper_grasp_detector
  rclcpp
  sensor_msgs
  std_msgs
)

# sim only: CPU lidar gz system (Harmonic has no physics lidar), see src/sim/seyond_lidar.cpp
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_common_vendor REQUIRED)
find_package(gz-common REQUIRED COMPONENTS graphics)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)
find_package(Bullet REQUIRED)
add_library(SeyondLidarSystem SHARED src/sim/seyond_lidar.cpp)
target_include_directories(SeyondLidarSystem PRIVATE ${BULLET_INCLUDE_DIRS})
target_link_libraries(SeyondLidarSystem
  gz-sim::gz-sim gz-common::graphics gz-plugin::register gz-transport::core gz-msgs::core
  ${BULLET_LIBRARIES})

install(DIRECTORY
  behavior_trees
  config
  launch
  models
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/gazebo_block_spawner.py
  scripts/gazebo_grasp_weld.py
  scripts/gazebo_block_oracle.py
  scripts/sim_loaded_hold_probe.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  gripper_grasp_detector
  SeyondLidarSystem
  ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_wait_for_move_complete test/test_wait_for_move_complete.cpp)
  target_link_libraries(test_wait_for_move_complete BT_cb_wait_for_move_complete_condition)
  ament_target_dependencies(test_wait_for_move_complete ${THIS_PACKAGE_INCLUDE_DEPENDS})
  ament_add_gtest(test_set_mpc_payload test/test_set_mpc_payload.cpp)
  target_link_libraries(test_set_mpc_payload BT_cb_set_mpc_payload_action)
  ament_target_dependencies(test_set_mpc_payload ${THIS_PACKAGE_INCLUDE_DEPENDS})
  ament_add_pytest_test(
    test_mpc_payload_tree
    test/test_mpc_payload_tree.py
  )
  ament_add_pytest_test(
    test_basic_pick_and_place_launch
    test/test_basic_pick_and_place_launch.py
  )
  ament_add_pytest_test(
    test_cbs_a2b_tree
    test/test_cbs_a2b_tree.py
  )
  ament_add_pytest_test(
    test_dds_sim_profile
    test/test_dds_sim_profile.py
  )
  ament_add_pytest_test(
    test_start_pose_is_control_safe
    test/test_start_pose_is_control_safe.py
  )
endif()

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_export_include_directories(include)
ament_export_libraries(${plugin_libs})
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
