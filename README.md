# concrete_block_behavior_tree

Behavior-tree launch and plugin package for concrete block pick-and-place and
wall assembly demos with the PZS100 crane in Gazebo.

This package glues together:

- PZS100 Gazebo bringup from `epsilon_crane_bringup_sim`
- behavior-tree execution through `lsrl_behavior_tree`
- motion-planning helper nodes from `concrete_block_motion_planning`
- block state from `concrete_block_perception/world_model_node`
- Gazebo block spawning from the same world-model seed data

## Contents

```text
behavior_trees/      BehaviorTree.CPP XML trees and reusable subtrees
config/              BT server overrides, panel catalog, wall plans, profiles
launch/              PZS100 Gazebo demo launch files
models/concrete_block Gazebo model spawned for each concrete block
scripts/             Gazebo block spawner
src/plugins/action/  CBS-specific BT service nodes
```

## Build

From the workspace root:

```bash
colcon build --packages-select concrete_block_behavior_tree --symlink-install
source install/setup.bash
```

The package installs its launch files, BT XML files, configs, Gazebo model, and
`gazebo_block_spawner.py`.

## Launch Files

### Wall Assembly

```bash
ros2 launch concrete_block_behavior_tree gazebo_wall_assembly_pzs100.launch.py
```

Starts the full PZS100 Gazebo simulation and adds the concrete block wall
assembly stack:

- `world_model_node`, launched by `epsilon_crane_bringup_sim/launch/gazebo_model_bt_pzs100.launch.py`
- `grip_traj_server_simple.py`
- `wall_plan_server.py`
- `lsrl_behavior_tree/bt_action_server`
- `nav2_lifecycle_manager`
- `gazebo_block_spawner.py`
- optional keyboard TUI

### Basic Pick And Place

```bash
ros2 launch concrete_block_behavior_tree gazebo_basic_pick_and_place_pzs100.launch.py
```

Starts a simpler pick-and-place behavior tree using
`behavior_trees/basic_pick_and_place.xml`.

Common launch arguments:

```text
gui:=True|False
initial_pose:=<epsilon crane initial pose id>
controller:=pid|mpc
```

## Behavior Trees

Main trees:

```text
behavior_trees/basic_pick_and_place.xml
behavior_trees/wall_assembly.xml
```

Reusable subtrees:

```text
behavior_trees/subtree_pick_and_place_block.xml
behavior_trees/subtree_execute_trajectory.xml
```

`config/bt_server_override.yaml` extends the epsilon crane BT plugin list with
the concrete-block-specific plugins:

```text
BT_cb_get_next_assembly_task_action
BT_cb_set_block_task_status_action
```

These are service-backed BehaviorTree.CPP nodes:

- `GetNextAssemblyTask` calls the wall-plan server and writes pickup/place poses
  to the blackboard.
- `SetBlockTaskStatus` updates a block in the world model after placement.

## Block Configuration

Initial block poses are configured in the world-model seed file owned by the
perception package:

```text
concrete_block_perception/config/world_model_seed_pick_place.yaml
```

That file is loaded into `world_model_node` and contains:

```yaml
world_model_node:
  ros__parameters:
    world_frame: K0_mounting_base
    world_model:
      initial_blocks: |
        - id: block_1
          frame_id: K0_mounting_base
          position: [4.5, 0.25, -0.960]
          yaw_deg: 0.0
          pose_status: POSE_PRECISE
          task_status: TASK_FREE
          confidence: 1.0
```

Edit `world_model.initial_blocks` in that YAML to add, remove, or move blocks.
The `position` and `yaw_deg` fields are the planning/RViz source of truth and
are expressed in `K0_mounting_base`.

The RViz markers are published by `world_model_node` on:

```text
/cbp/block_world_model_markers
```

## Gazebo Block Spawning

PZS100 launch files start:

```text
scripts/gazebo_block_spawner.py
```

after Gazebo has started. The spawner:

1. reads the same `concrete_block_perception/config/world_model_seed_pick_place.yaml`
2. parses `world_model.initial_blocks`
3. loads the Gazebo model from `models/concrete_block/model.sdf`
4. calls Gazebo `/spawn_entity` once per block

For PZS100, precomputed `gazebo_pose` entries are not used. Instead, the
spawner transforms each block from `K0_mounting_base` into Gazebo `world` using
parameters in the PZS100 launch files:

```yaml
use_precomputed_gazebo_pose: False
seed_frame_id: K0_mounting_base
gazebo_seed_frame_xyz: [6.93852, -6.35, 1.1407]
gazebo_seed_frame_rpy_deg: [0.0, 0.0, 0.0]
spawn_height_offset: 0.3
```

`gazebo_seed_frame_xyz` is the pose of `K0_mounting_base` in Gazebo coordinates
for the current crane spawn setup. `spawn_height_offset` raises each block
before spawning so it can settle onto the terrain.

This keeps the Gazebo blocks and RViz markers aligned relative to
`K0_mounting_base`.

## Wall Plans

Wall assembly targets are configured in:

```text
config/wall_plans.yaml
```

Example:

```yaml
wall_plans:
  pzs100_pick_place:
    sequence:
      - id: "block_1"
        absolute_position: [5.0, 2.0, -0.84]
        yaw_deg: 0.0
```

The wall-plan server reads this file and serves the next task to the
`GetNextAssemblyTask` BT node. Block IDs in a wall plan should match block IDs
in the world-model seed file.

## Useful Topics And Services

```text
/cbp/block_world_model
/cbp/block_world_model_markers
/block_world_model_node/get_coarse_blocks
/block_world_model_node/set_block_task_status
/concrete_block_motion_planning_node/get_next_assembly_task
/spawn_entity
```

## Developer Notes

- The BT panel catalog is configured by `config/bt_panel_catalog.yaml`.
- Launch files set `BEHAVIOR_TREE_PANEL_BT_PACKAGE` and
  `BEHAVIOR_TREE_PANEL_BT_CATALOG` for RViz/HMI integration.
- If the crane spawn pose changes, update the PZS100 launch parameters
  `gazebo_seed_frame_xyz` and `gazebo_seed_frame_rpy_deg` so Gazebo block poses
  still match the world model.
- Keep block IDs consistent across `world_model_seed_pick_place.yaml` and
  `config/wall_plans.yaml`.
