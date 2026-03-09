# Groot Guide (Behavior Tree GUI)

This repo can use **Groot** as GUI for:
- Editing BT XML files visually
- Monitoring a running tree (when monitoring is enabled)

For your current stack (`behaviortree_cpp_v3`), use **Groot v1** style workflow.

## 1) What you already have in this repo

- BT server launch: `launch/bt.launch.py`
- Default BT config: `config/default.yaml`
- Monitoring flag: `logging.enable_groot_monitoring`

`config/default.yaml` and `config/scan_smoke.yaml` already set:
- `enable_groot_monitoring: true`

`config/dummy_start.yaml` sets it to `false` (expected for minimal/dummy run).

## 2) Install Groot

Use one of these options (depending on your environment):

1. Prebuilt package/binary (fastest)
- Install Groot from your ROS/OS package source if available.

2. Build from source (reliable when package is unavailable)
- Clone Groot repository and build with its documented dependencies.
- Prefer matching major version to BehaviorTree.CPP v3.

Note: exact install command differs by distro and image. If you want, we can pin one method for your devcontainer and add it to setup docs.

## 3) Edit BT XML with Groot

1. Open Groot GUI.
2. Load tree XML from:
- `behavior_trees/concrete_block_assembly.xml`
- or `behavior_trees/scan_sequence_smoke.xml`
- or `behavior_trees/dummy_start.xml`
3. Edit/control flow and node attributes.
4. Save XML back into `behavior_trees/`.
5. If you changed node types, make sure corresponding plugin libraries are present in your YAML `plugin_lib_names`.

## 4) Run and monitor from Groot

1. Launch BT stack with monitoring enabled config:
```bash
ros2 launch concrete_block_behavior_tree bt.launch.py \
  bt_params_file:=src/concrete_block_stack/concrete_block_behavior_tree/config/default.yaml
```

2. Open Groot monitor/live view and connect to the running BT publisher.

3. Trigger the BT action flow you want to observe (depends on your orchestrating launch/test setup).

## 5) Quick profiles in this repo

1. Full/default behavior tree
```bash
ros2 launch concrete_block_behavior_tree bt.launch.py \
  bt_params_file:=src/concrete_block_stack/concrete_block_behavior_tree/config/default.yaml
```

2. Smoke tree
```bash
ros2 launch concrete_block_behavior_tree scan_sequence_smoke.launch.py
```

3. Dummy startup stack (Gazebo + RViz + dummy BT)
```bash
ros2 launch concrete_block_behavior_tree simple_sim_bt_dummy.launch.py
```

If you want Groot monitoring for dummy mode too, set in `config/dummy_start.yaml`:
- `logging.enable_groot_monitoring: true`

## 6) Troubleshooting

1. Groot connects but no updates
- Verify `enable_groot_monitoring: true` in the active `bt_params_file`.
- Ensure you launched the config you think you launched.

2. Tree fails to load
- Check XML node tag names match registered plugin node types.
- Check all required plugin libs are listed in `plugin_lib_names`.

3. Node exists but always fails
- Usually service mismatch/timeouts.
- Verify each XML `service_name` points to an available ROS service.

4. Edited XML but behavior unchanged
- Confirm `behaviortree` path in the active config points to your edited file.
- Restart BT server after edits if needed.

## 7) Recommended prototyping workflow

1. Duplicate an XML to a new experiment file in `behavior_trees/`.
2. Duplicate config YAML in `config/` and point `behaviortree` to that XML.
3. Enable Groot monitoring in that YAML.
4. Launch with explicit `bt_params_file:=...`.
5. Iterate quickly without touching `default.yaml` until stable.
