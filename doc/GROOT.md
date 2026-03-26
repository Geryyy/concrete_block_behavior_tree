# Groot Guide

## What to open in Groot

For current commissioning work, focus on the canonical intent-based trees:

- `behavior_trees/move_empty.xml`
- `behavior_trees/single_block_plan.xml`
- `behavior_trees/single_block_execute.xml`

Only open `concrete_block_assembly.xml` when you are working on the longer-term automatic assembly workflow.

## Current recommended launch shape

The BT stack now uses composable parameter layers rather than one monolithic BT YAML.

Relevant launch parameters:

- `bt_common_params_file`
- `bt_mode_params_file`
- `bt_profile_params_file`

That means when a tree seems different from what Groot shows, the first thing to verify is which profile file is actually active.

## Canonical commissioning tasks

### `Move empty`

- operator movement tool
- approval-gated execution

### `Single block plan`

- world-model and planner commissioning
- plans from the current tool pose to the target task pose
- no live execution

### `Single block execute`

- planning plus approval-gated execution
- concrete execution should now correspond to the displayed planned path

This is the current operator vocabulary and should stay aligned with the BT panel catalog.

## Monitoring advice

Use Groot/live BT monitoring to answer:

- which canonical tree is currently running
- which service node succeeded or failed
- whether the tree is in planning-only or execution phase

That is especially important now that backend choice is deliberately hidden below the BT layer.

## Common pitfalls

1. Editing a legacy alias instead of the canonical tree.
2. Assuming a backend issue requires a BT fork.
3. Forgetting that `Single block plan` intentionally does not execute.
4. Forgetting that `Single block execute` includes explicit approval gating.

## Rule of thumb

If the operator workflow is unchanged, prefer changing:

- motion-planning config
- backend adapters
- world-model interfaces

instead of creating another BT variant.
