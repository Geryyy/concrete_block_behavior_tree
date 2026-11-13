"""Issue 327: no failure, cancel or restart path opens the jaws over the wrong place.

docs/features/cbs-bt-failure-paths.md: a restart with a block clamped ran the pick path, which
lifts as an empty hook and opens the jaws at the hover. The plan-server half is in
concrete_block_assembly_planning's tests; these pin the tree's half.
"""

from pathlib import Path
from xml.etree import ElementTree

TREE_DIR = Path(__file__).resolve().parents[1] / "behavior_trees"
JAWS_EMPTY = "/gripper/jaws_empty"


def _subtree(xml, tree_id):
    root = ElementTree.parse(TREE_DIR / xml).getroot()
    return next(e for e in root.iter("BehaviorTree") if e.get("ID") == tree_id)


def _body(xml, tree_id):
    return list(next(iter(_subtree(xml, tree_id))))


def _is_jaws_check(e, topic=JAWS_EMPTY):
    return e.get("ID") == "WaitForGraspSignal" and e.get("topic") == topic


def test_a_pick_refuses_to_start_with_anything_in_the_jaws():
    status = next(
        _subtree("subtree_pick_and_place_block.xml", "SubTreePickAndPlaceBlock").iter(
            "GetBlockTaskStatus"
        )
    )
    assert status.get("moving_block_id") == "{moving_block_id}"

    first, second = _body("subtree_pick_block.xml", "SubTreePickBlock")[:2]
    assert first.tag == "ScriptCondition"
    assert first.get("code").replace(" ", "") == "moving_block_id==''"
    assert _is_jaws_check(second)
    for gate in (first, second):
        assert gate.get("name"), "the failure has to name its reason"


def test_a_failed_close_is_released_on_the_ground():
    fallback = next(
        e
        for e in _subtree("subtree_pick_block.xml", "SubTreePickFromHover").iter(
            "Fallback"
        )
        if e.get("name") == "register_grip_or_release"
    )
    register, _release = list(fallback)
    phases = [e.get("value") for e in register if e.get("output_key") == "gr_phase"]
    assert phases == ["2"], (
        "the jaw close belongs inside the release-on-failure fallback"
    )
    assert register[2].get("ID") == "SubTreeGripper"


def test_jaws_that_did_not_open_leave_the_block_in_hand():
    body = _body("subtree_place_block.xml", "SubTreeRelease")

    def at(match):
        return next(i for i, e in enumerate(body) if match(e))

    checked = at(lambda e: e.tag == "Fallback")
    cleared = at(lambda e: e.get("ID") == "SetMpcPayload")
    assert body[cleared].get("carries") == "false"
    assert checked < cleared
    assert body[checked - 2].get("value") == "3"  # gr_phase: open

    # One arbiter, and it is the jaw sensor (framework review item 14): the open and
    # the sensor that confirms it are one branch, so a failed open is not a state of
    # its own. It used to leave TASK_PLACED where a timed-out open left TASK_MOVE --
    # the same physical state, reached by two restart paths.
    opened, otherwise = list(body[checked])
    gripper, confirmed = list(opened)
    assert gripper.get("ID") == "SubTreeGripper"
    assert _is_jaws_check(confirmed, "/gripper/jaws_open")
    not_open, back, fail = list(otherwise)
    assert not_open.tag == "Inverter"
    assert _is_jaws_check(list(not_open)[0], "/gripper/jaws_open")
    assert back.get("ID") == "SetBlockTaskStatus" and back.get("task_status") == "2"
    assert back.get("grasp_offset") == "{grasp_offset}"  # FK-tracked from here on
    assert fail.tag == "AlwaysFailure"


def test_the_discovery_move_does_not_run_with_anything_in_the_jaws():
    root = ElementTree.parse(TREE_DIR / "wall_assembly.xml").getroot()
    gate = next(
        e
        for e in root.iter("ScriptCondition")
        if "plan_has_task" in (e.get("code") or "")
    )
    branch = next(e for e in root.iter("Sequence") if gate in list(e))
    body = list(branch)
    assert _is_jaws_check(body[body.index(gate) + 1])
