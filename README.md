# concrete_block_behavior_tree

Behavior-tree launch and plugin package for concrete block pick-and-place and
wall assembly demos with the PZS100 crane in Gazebo.

This package glues together:

- PZS100 Gazebo bringup from `epsilon_crane_bringup_sim`
- behavior-tree execution through `lsrl_behavior_tree`
- motion-planning helper nodes from `concrete_block_motion_planning`
- block state from `concrete_block_world_model/world_model_node`
- Gazebo block spawning from the same world-model seed data

## Dependencies & interactions

This is the **orchestrator** of the stack — its BT plugins (`src/plugins/`) are clients of nearly every other package, and the tree itself runs inside `lsrl_behavior_tree`'s `bt_action_server` (BehaviorTree.CPP **v3**). The `wall_assembly.xml` tree composes the subtrees `MoveAbove → Gripper → DescendTo → Lift → ExecuteTrajectory`.

| BT node / plugin | Talks to | Interface |
|---|---|---|
| `GetNextAssemblyTask`, `PlanComplete` | [concrete_block_assembly_planning](../concrete_block_assembly_planning/) | `concrete_block_assembly_interfaces/GetNextAssemblyTask` |
| `CalcGripMovement` (descend / gripper / lift) | [concrete_block_motion_planning](../concrete_block_motion_planning/) | `grip_traj_movement` |
| `CalcA2BMovement` (move-above) | timber_crane A2B server | `a2b_movement` |
| `SetBlockTaskStatus`, `CaptureBlockGraspOffset`, `WriteBlockPoseFromGripper` | [concrete_block_world_model](../concrete_block_world_model/) | `concrete_block_world_model_interfaces` (`SetBlockTaskStatus`, `GetCoarseBlocks`, `UpsertBlock`) |
| `ExecuteTrajectory` (`SwitchController` + `FollowJointTrajectory`) | `controller_manager` / `ros2_control` | controller switch + FJT action |
| `CheckGripperEffort` | `/joint_states` | grip verification from sustained gripper effort |
| `PublishGraspCommand`, `WaitForGazeboGrasp` | `gazebo_grasp_plugin_ros` / Gazebo | optional simulation-only grasp attach/detach helpers |

Bringup / infrastructure deps: `lsrl_behavior_tree` (BT engine), `epsilon_crane_bringup_sim` / `epsilon_crane_bringup_mp` (PZS100 Gazebo + MP bringup), `pzs100_description`, `epsilon_7040_description`, `collision_body_handler`, `nav2_behavior_tree` / `nav2_lifecycle_manager`. Because it pulls in the epsilon/timber stacks, `--packages-up-to concrete_block_behavior_tree` builds ~76 packages.

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

### Commissioning Stack

```bash
ros2 launch concrete_block_behavior_tree gazebo_basic_pick_and_place_pzs100.launch.py
```

Starts a smaller commissioning behavior tree using
`behavior_trees/stack_block_1_on_block_2.xml`. It runs the same
wall-plan/world-model path as wall assembly, but with one task: stack `c0_b0`
on `c0_b1`. In Gazebo this launch loads
`config/profiles/grip_sim.yaml`, which keeps BT selection separate from the
grip effort threshold for simulated joint-state effort.

Common launch arguments:

```text
gui:=True|False
initial_pose:=<epsilon crane initial pose id>
controller:=pid|mpc
```

## Behavior Trees

Main trees:

```text
behavior_trees/stack_block_1_on_block_2.xml
behavior_trees/wall_assembly.xml
```

`behavior_trees/basic_pick_and_place.xml` is a legacy hard-coded simulation demo
kept for direct launch testing. The BT panel catalog lists only trees that use
the wall-plan/world-model path.

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
BT_cb_check_gripper_effort_condition
BT_cb_capture_block_grasp_offset_action
BT_cb_write_block_pose_from_gripper_action
BT_cb_plan_complete_condition
```

These are service-backed BehaviorTree.CPP nodes:

- `GetNextAssemblyTask` calls the wall-plan server and writes pickup/place poses
  to the blackboard.
- `SetBlockTaskStatus` updates a block in the world model after placement.
- `CheckGripperEffort` waits until the gripper effort stays above threshold for
  a minimum duration before the crane starts lifting the block.

## Block Configuration

Initial block poses are configured in the spawn seed file owned by the
perception package:

```text
concrete_block_world_model/config/world_model_seed_pick_place.yaml
```

For PZS100 Gazebo BT launches, this file is used as the spawn recipe. The
`world_model_node` itself starts with:

```text
concrete_block_world_model/config/world_model_seed_none.yaml
```

and is populated after Gazebo reports the settled block poses.

The spawn seed contains:

```yaml
world_model_node:
  ros__parameters:
    world_frame: world
    world_model:
      initial_blocks: |
        - id: c0_b0
          frame_id: world
          position: [-13.0, 0.0, 0.3]
          yaw_deg: 0.0
          pose_status: POSE_PRECISE
          task_status: TASK_FREE
          confidence: 1.0
```

Edit `world_model.initial_blocks` in the pick/place seed YAML to add, remove,
or move spawned blocks. The `position` and `yaw_deg` fields are expressed in
`K0_mounting_base` and are only the initial spawn guess; the world model used by
planning/RViz is updated by the behavior tree from settled Gazebo poses.

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

1. reads the same `concrete_block_world_model/config/world_model_seed_pick_place.yaml`
2. parses `world_model.initial_blocks`
3. loads the Gazebo model from `models/concrete_block/model.sdf`
4. calls Gazebo `/spawn_entity` once per block

For PZS100, precomputed `gazebo_pose` entries are not used. Instead, the
spawner transforms each block from the CBS `world` frame into Gazebo `world`
using parameters in the PZS100 launch files:

```yaml
use_precomputed_gazebo_pose: False
seed_frame_id: world
gazebo_seed_frame_xyz: [0.0, -6.0, 0.0]
gazebo_seed_frame_rpy_deg: [0.0, 0.0, 180.0]
spawn_height_offset: 0.15
sync_world_model_from_gazebo: False
```

`gazebo_seed_frame_xyz` and `gazebo_seed_frame_rpy_deg` describe the CBS
`world` frame in Gazebo coordinates for the current crane spawn setup.
`spawn_height_offset` raises each block before spawning so it can settle onto
the terrain.

The behavior trees do not query Gazebo directly. In simulation the world model
is seeded from the YAML file; on the real crane perception updates the same
world model before planning.

## Grasp Verification

The shared pick tree verifies a grasp with `CheckGripperEffort`, which watches
the configured gripper joint in `/joint_states` and requires `|effort|` to stay
above `min_effort` for `min_duration_ms`. This works for both simulation and
the real crane as long as the gripper effort/pressure is exposed through joint
state effort.

Deployment-specific thresholds are ROS parameters on the BT action server:

```yaml
check_gripper_effort:
  topic: "/joint_states"
  joint: "q9_left_rail_joint"
  min_effort: 1000.0
  min_duration_ms: 500
  timeout_ms: 8000
```

For the real crane, tune these in `config/profiles/grip_real.yaml`. For Gazebo,
tune them in `config/profiles/grip_sim.yaml`. These profiles intentionally do
not set `behaviortree`, so RViz panel tree selection and grip tuning stay
independent.

The PZS100 Gazebo gripper may also load `gazebo_grasp_fix` in:

```text
crane_tools_description/pzs100/gazebo/gripper.gazebo.xacro
```

When `gazebo_grasp_plugin_ros` is available, this package also builds optional
Gazebo-only BT plugins for legacy/simulation-specific trees:

```text
WaitForGazeboGrasp
PublishGraspCommand
```

The default BT plugin config does not load those Gazebo-only plugins, so the
behavior-tree package does not require `gazebo_grasp_plugin_ros` on the real
crane.

## Wall Plans

Wall assembly targets are configured in:

```text
config/wall_plans.yaml
```

Example:

```yaml
wall_plans:
  stack_block_1_on_block_2:
    sequence:
      - id: c0_b0
        relative_to_world_model: c0_b1
        offset: [0.0, 0.0, 0.6]
        fallback_absolute_position: [-14.0, 0.0, 0.9]
        yaw_deg: 0.0
        gripper_yaw_offset_deg: 90.0

  example_wall:
    sequence:
      - id: c0_b0
        yaw_deg: 0.0
        gripper_yaw_offset_deg: 90.0
        absolute_position: [-15.0, -3.0, 0.3]
      - id: c0_b1
        yaw_deg: 0.0
        gripper_yaw_offset_deg: 90.0
        absolute_position: [-14.08, -3.0, 0.3]
```

The wall-plan server reads this file and serves the next task to the
`GetNextAssemblyTask` BT node. Block IDs in a wall plan should match block IDs
in the world-model seed file.

Both simulation and the real system use the world model as the block-pose
boundary. In simulation, `world_model_seed_pick_place.yaml` seeds the world
model and the Gazebo spawner uses the same IDs. On the real crane, perception
updates the world model with those IDs instead. The behavior trees and wall-plan
server stay unchanged.

## Useful Topics And Services

```text
/cbp/block_world_model
/cbp/block_world_model_markers
/world_model_node/get_coarse_blocks
/world_model_node/set_block_task_status
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
