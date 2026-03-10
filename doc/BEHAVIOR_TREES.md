# Behavior Tree Editing Guide

This document explains how behavior trees are wired in `concrete_block_behavior_tree`, what you need to edit them safely, and how to validate changes.

## 1) How it works (high level)

1. `bt.launch.py` starts `lsrl_behavior_tree/bt_action_server` and `nav2_lifecycle_manager`.
2. The BT server loads parameters from a YAML file (`bt_params_file`).
3. In that YAML, `behaviortree` points to the XML tree file to execute.
4. The BT server loads all shared libraries listed in `plugin_lib_names`.
5. XML node tags (for example `<PlanGeometricPath .../>`) must match a node type registered by those plugins.
6. Each node calls a ROS service (`service_name` port), then maps response fields to blackboard outputs.

Current default wiring:
- Launch file: `launch/bt.launch.py`
- Default config: `config/default.yaml`
- Default tree: `behavior_trees/concrete_block_assembly.xml`

## 2) What is required to edit a BT

You need 3 things to stay consistent:

1. XML tree structure
- File in `behavior_trees/*.xml`
- Uses control nodes (`Sequence`, `Fallback`, etc.) and action/service nodes.

2. Matching plugin libraries in YAML
- `config/*.yaml` key: `plugin_lib_names`
- Must contain every custom node library used by your XML.

3. Correct ports in XML attributes
- Attributes in XML must match `providedPorts()` of each plugin class.
- Port type and naming must be consistent across producer/consumer nodes.

## 3) Edit workflow (recommended)

1. Copy an existing tree as a template
- Start from `behavior_trees/dummy_start.xml` or `behavior_trees/scan_sequence_smoke.xml`.

2. Add or reorder nodes in XML
- Use blackboard variables with `{var_name}` to pass outputs to later inputs.

3. Create/update a matching YAML config
- Set `behaviortree: ".../your_tree.xml"`
- Keep only needed plugin libs.

4. Launch with explicit config
- `ros2 launch concrete_block_behavior_tree bt.launch.py bt_params_file:=<absolute_or_package_resolved_yaml>`

5. Validate runtime behavior
- Ensure services exist and names match XML `service_name` fields.
- Check BT server logs for missing node types, port parse errors, or service timeouts.

## 4) Node library mapping in this repo

From `CMakeLists.txt` and plugin registrations:

- `BT_cb_run_pose_estimation_action` -> XML node `RunPoseEstimation`
- `BT_cb_plan_geometric_path_action` -> XML node `PlanGeometricPath`
- `BT_cb_compute_trajectory_action` -> XML node `ComputeTrajectory`
- `BT_cb_execute_trajectory_action` -> XML node `ExecuteTrajectory`
- `BT_cb_move_to_named_configuration_action` -> XML node `MoveToNamedConfiguration`
- `BT_cb_get_next_assembly_task_action` -> XML node `GetNextAssemblyTask`

If an XML tag is not registered by a loaded plugin library, tree loading fails.

## 5) Port reference (custom action/service nodes)

### RunPoseEstimation
Inputs:
- `service_name` (required in XML)
- `mode` (default `SCENE_DISCOVERY`)
- `target_block_id` (default empty)
- `enable_debug` (default `true`)
- `timeout_s` (default `8.0`)
Outputs:
- `perception_ok`
- `perception_message`

### PlanGeometricPath
Inputs:
- `service_name`
- `start_pose`
- `goal_pose`
- `target_block_id`
- `reference_block_id`
- `method` (default `POWELL`)
- `timeout_s` (default `5.0`)
- `use_world_model` (default `true`)
Outputs:
- `geometric_plan_id`
- `cartesian_path`
- `geometric_ok`
- `geometric_message`

### ComputeTrajectory
Inputs:
- `service_name`
- `geometric_plan_id`
- `cartesian_path`
- `use_direct_path` (default `false`)
- `method` (default `ACADOS_PATH_FOLLOWING`)
- `timeout_s` (default `10.0`)
- `validate_dynamics` (default `true`)
Outputs:
- `trajectory_id`
- `joint_trajectory`
- `trajectory_ok`
- `trajectory_message`

### ExecuteTrajectory
Inputs:
- `service_name`
- `trajectory_id`
- `dry_run` (default `true`)
Outputs:
- `execution_ok`
- `execution_message`

### MoveToNamedConfiguration
Inputs:
- `service_name`
- `configuration_name`
- `timeout_s` (default `20.0`)
- `dry_run` (default `false`)
Outputs:
- `trajectory_id`
- `move_ok`
- `move_message`

### GetNextAssemblyTask
Inputs:
- `service_name`
- `wall_plan_name` (default `basic_interlocking_3_2`)
- `reset_plan` (default `false`)
Outputs:
- `task_id`
- `target_block_id`
- `reference_block_id`
- `target_block_pose_coarse`
- `target_block_pose_precise`
- `reference_block_pose_precise`
- `plan_has_task`
- `plan_message`

## 6) Common failure modes

1. Plugin not loaded
- Symptom: unknown BT node type.
- Fix: add the correct library name to `plugin_lib_names`.

2. Port mismatch
- Symptom: XML parse/runtime error for missing/invalid port.
- Fix: align XML attributes with `providedPorts()` definitions.

3. Service name mismatch
- Symptom: timeout or repeated failures even though tree parses.
- Fix: verify `service_name` in XML matches actual running service.

4. Type mismatch on blackboard variable reuse
- Symptom: conversion or runtime blackboard errors.
- Fix: ensure producer output type matches consumer input type.

## 7) Minimal examples

Dummy tree only:
- XML: `behavior_trees/dummy_start.xml`
- Config: `config/dummy_start.yaml`
- Launch: `launch/simple_sim_bt_dummy.launch.py`

Smoke sequence with real service nodes:
- XML: `behavior_trees/scan_sequence_smoke.xml`
- Config: `config/scan_smoke.yaml`
- Launch: `launch/scan_sequence_smoke.launch.py`

Wall-build simulation baseline (phase-1 roadmap entrypoint):
- Launch: `launch/sim_wall_build.launch.py`
- Optional perception: `use_perception:=True`

## 8) Adding a new custom BT node (C++)

1. Add header/source under `include/.../plugins/action/` and `src/plugins/action/`.
2. Implement `providedPorts()`, `on_tick()`, and `on_completion()`.
3. Register node type with `BT_REGISTER_NODES(factory)`.
4. Add a shared library target in `CMakeLists.txt`.
5. Add the new library to `plugin_lib_names` in your YAML.
6. Use the registered XML tag in your tree.

## 9) Practical tip for prototyping

Use separate YAML + XML pairs for each experiment (`config/*.yaml`, `behavior_trees/*.xml`) instead of constantly editing `default.yaml` and `concrete_block_assembly.xml`. This keeps test scenarios isolated and easier to debug.
