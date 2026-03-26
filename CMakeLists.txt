cmake_minimum_required(VERSION 3.8)
project(concrete_block_behavior_tree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  behaviortree_cpp_v3
  concrete_block_motion_planning
  concrete_block_perception
  geometry_msgs
  lsrl_behavior_tree
  nav2_behavior_tree
  nav_msgs
  pluginlib
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
  trajectory_msgs
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(include)

function(add_bt_plugin plugin_name source_file)
  add_library(${plugin_name} SHARED ${source_file})
  ament_target_dependencies(${plugin_name} ${THIS_PACKAGE_INCLUDE_DEPENDS})
  target_compile_definitions(${plugin_name} PRIVATE BT_PLUGIN_EXPORT)
  list(APPEND plugin_libs ${plugin_name})
  set(plugin_libs "${plugin_libs}" PARENT_SCOPE)
endfunction()

add_bt_plugin(BT_cb_run_pose_estimation_action src/plugins/action/run_pose_estimation.cpp)
add_bt_plugin(BT_cb_plan_geometric_path_action src/plugins/action/plan_geometric_path.cpp)
add_bt_plugin(BT_cb_plan_and_compute_trajectory_action src/plugins/action/plan_and_compute_trajectory.cpp)
add_bt_plugin(BT_cb_compute_trajectory_action src/plugins/action/compute_trajectory.cpp)
add_bt_plugin(BT_cb_execute_trajectory_action src/plugins/action/execute_trajectory.cpp)
add_bt_plugin(BT_cb_prepare_move_empty_request_action src/plugins/action/prepare_move_empty_request.cpp)
add_bt_plugin(BT_cb_prepare_single_block_request_action src/plugins/action/prepare_single_block_request.cpp)
add_bt_plugin(BT_cb_move_to_named_configuration_action src/plugins/action/move_to_named_configuration.cpp)
add_bt_plugin(BT_cb_get_next_assembly_task_action src/plugins/action/get_next_assembly_task.cpp)
add_bt_plugin(BT_cb_set_block_task_status_action src/plugins/action/set_block_task_status.cpp)
add_bt_plugin(BT_cb_gripper_action src/plugins/action/gripper_action.cpp)

install(DIRECTORY
  behavior_trees
  config
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(${plugin_libs})
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
