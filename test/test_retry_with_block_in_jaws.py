"""A task served again keeps the block already in the jaws (issue 274).

268 made the plan server hand a task out again until its block is TASK_PLACED, which
makes SubTreePickAndPlaceBlock the retry path. The contract pinned here: the pick is
reachable only through the block's task status, a TASK_MOVE block is taken over rather
than picked, and the take-over neither opens the jaws nor descends -- and fails if the
jaws turn out to be empty instead of descending on a pose the block left long ago.
"""

from pathlib import Path
from xml.etree import ElementTree

import yaml

PACKAGE_DIR = Path(__file__).resolve().parents[1]
TREE = PACKAGE_DIR / "behavior_trees" / "subtree_pick_and_place_block.xml"


def _subtree(tree_id):
    root = ElementTree.parse(TREE).getroot()
    return next(e for e in root.iter("BehaviorTree") if e.get("ID") == tree_id)


def test_the_pick_is_reachable_only_through_the_blocks_task_status():
    cycle = _subtree("SubTreePickAndPlaceBlock")
    status = [e for e in cycle.iter() if e.tag == "GetBlockTaskStatus"]
    assert len(status) == 1 and status[0].get("block_id") == "{grasp_object_contains}"
    key = status[0].get("task_status")

    branch = next(e for e in cycle.iter() if e.tag == "IfThenElse")
    condition, on_move, otherwise = list(branch)
    # Block.TASK_MOVE == 2.
    assert condition.get("code").replace(" ", "") == f"{key[1:-1]}==2"
    assert on_move.get("ID") == "SubTreeResumeCarry"
    assert otherwise.get("ID") == "SubTreePickBlock"

    # Within this cycle the branch is the only way into the pick. SubTreeRegrip's own
    # direct SubTreePickBlock is a different path and stays legal: SubTreeSetDown's
    # WriteBlockPoseFromGripper clears TASK_MOVE before it re-picks.
    picks = [e for e in cycle.iter() if e.get("ID") == "SubTreePickBlock"]
    assert picks == [otherwise]


def test_taking_over_a_held_block_neither_opens_the_jaws_nor_descends():
    resume = list(_subtree("SubTreeResumeCarry").iter())
    ids = [e.get("ID") for e in resume]
    assert "SubTreeGripper" not in ids, "the jaws must stay closed on the block"
    assert "SubTreeDescendTo" not in ids, (
        "nothing may be aimed at the stale pickup pose"
    )

    # The grasp signal gates everything else, so a TASK_MOVE block that is not in the
    # jaws fails the task instead of being picked blind.
    body = list(next(e for e in resume if e.tag == "Sequence"))
    assert body[0].get("ID") == "WaitForGraspSignal", [e.tag for e in body]
    assert body[0].get("topic") == "/gripper/grasp_detected"
    at = ids.index("WaitForGraspSignal")

    # Carry state the place needs, rebuilt from the world model's FK-tracked TASK_MOVE pose.
    assert ids.index("CaptureBlockGraspOffset") > at
    assert ids.index("SetMpcPayload") > ids.index("CaptureBlockGraspOffset")
    assert resume[ids.index("SetMpcPayload")].get("carries") == "true"
    # grip_tilt for the regrip gate in SubTreePickAndPlaceBlock.
    assert resume[ids.index("MeasureBlockFromGripper")].get("tilt") == "{grip_tilt}"


def test_the_plugin_is_loadable_by_the_bt_server():
    with (PACKAGE_DIR / "config" / "bt_server_override.yaml").open() as handle:
        override = yaml.safe_load(handle)
    assert (
        "BT_cb_get_block_task_status_action"
        in override["/**"]["ros__parameters"]["plugin_lib_names"]
    )
