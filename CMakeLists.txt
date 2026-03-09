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
  trajectory_msgs
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(include)

add_library(BT_cb_run_pose_estimation_action SHARED src/plugins/action/run_pose_estimation.cpp)
list(APPEND plugin_libs BT_cb_run_pose_estimation_action)

add_library(BT_cb_plan_geometric_path_action SHARED src/plugins/action/plan_geometric_path.cpp)
list(APPEND plugin_libs BT_cb_plan_geometric_path_action)

add_library(BT_cb_compute_trajectory_action SHARED src/plugins/action/compute_trajectory.cpp)
list(APPEND plugin_libs BT_cb_compute_trajectory_action)

add_library(BT_cb_execute_trajectory_action SHARED src/plugins/action/execute_trajectory.cpp)
list(APPEND plugin_libs BT_cb_execute_trajectory_action)

add_library(BT_cb_move_to_named_configuration_action SHARED src/plugins/action/move_to_named_configuration.cpp)
list(APPEND plugin_libs BT_cb_move_to_named_configuration_action)

add_library(BT_cb_get_next_assembly_task_action SHARED src/plugins/action/get_next_assembly_task.cpp)
list(APPEND plugin_libs BT_cb_get_next_assembly_task_action)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${THIS_PACKAGE_INCLUDE_DEPENDS})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

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
