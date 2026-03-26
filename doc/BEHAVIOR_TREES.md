# Behavior Tree Guide

## Current operator surface

The commissioning BT surface is now intent-based.

Canonical operator trees:

- `move_empty.xml`
- `single_block_plan.xml`
- `single_block_execute.xml`

These are the trees the RViz BT panel should expose to operators.

Legacy aliases such as `scan_sequence_smoke.xml` still exist only for compatibility and should not be treated as the main commissioning interface.

## What each tree is for

### `Move empty`

- operator-driven move to a 2D goal
- approval gated
- proven on the timber execution path

### `Single block plan`

- gets the next wall-plan task
- queries the world model
- plans from the live tool pose
- plans only
- no approval, no execution

Use this when validating:

- world-model task context
- backend planning behavior
- RViz planned-path output

### `Single block execute`

- same task/world-model/planning flow
- then approval
- then execution
- concrete backend uses the same online TOPP-RA commissioning path as planning

Use this when validating:

- approval behavior
- trajectory execution
- live motion after planning succeeds

## BT configuration model

`bt.launch.py` now supports composable BT parameter layers:

- `bt_common_params_file`
- `bt_mode_params_file`
- `bt_profile_params_file`

This is the preferred structure.

The old single-file `bt_params_file` override still exists for compatibility, but it is no longer the recommended way to work on commissioning profiles.

## Editing rules

Keep these concerns separate:

1. operator intent
   - choose the right tree (`Move empty`, `Single block plan`, `Single block execute`)
2. backend choice
   - keep this in motion-planning launch/config
   - do not fork BT XML just because `timber` and `concrete` differ internally
3. long-term automation
   - `concrete_block_assembly.xml` remains separate because it represents a materially different workflow

## Active plugin/service expectations

Current shared service nodes that matter most:

- `PlanAndComputeTrajectory`
- `ExecuteTrajectory`
- `MoveToNamedConfiguration`
- `GetNextAssemblyTask`
- `RunPoseEstimation`
- `SetBlockTaskStatus`

Current commissioning emphasis:

- planning and execution tasks first
- perception-driven scan/refine later

## Practical workflow

1. Pick the commissioning intent.
2. Use the canonical profile from the RViz BT panel.
3. Keep backend selection in launch arguments such as `planner.backend`.
4. Watch BT logs for service response lines and state transitions.
5. Only create a new BT when operator workflow truly differs.

That keeps the BT layer small and avoids backend-specific duplication.
