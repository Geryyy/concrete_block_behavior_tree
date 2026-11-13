"""The MPC payload is set before every move, carrying or not (issues 175, 341).

What the node does with its inputs is pinned in test_set_mpc_payload.cpp. What can
only be pinned here is where the tree calls it: since 341 it is the four move
subtrees, each from its own `*_carries` key and ahead of its trajectory, so the
payload is also re-sent after a crane_mpc restart. What those keys are set to then
decides whether the MPC solves a loaded crane: the pick raises lf_carries after the
grasp offset is captured, the place raises mv_carries before its first move, and the
release clears the payload itself once the jaws are open.
"""

from pathlib import Path
from xml.etree import ElementTree

import yaml

PACKAGE_DIR = Path(__file__).resolve().parents[1]
TREE_DIR = PACKAGE_DIR / "behavior_trees"

# The subtrees that execute a planned trajectory, i.e. what the payload has to precede.
MOVE_SUBTREES = ("SubTreeCbsA2BMovement", "SubTreeExecuteTrajectory")


def _sequence(tree_file, tree_id):
    root = ElementTree.parse(tree_file).getroot()
    tree = next(e for e in root.iter("BehaviorTree") if e.get("ID") == tree_id)
    return list(tree.iter())


def _index(elements, predicate):
    return next(i for i, e in enumerate(elements) if predicate(e))


def test_every_move_hands_the_mpc_the_payload_the_planner_already_gets():
    for file_name, tree_id, carries_key in (
        ("subtree_move_above.xml", "SubTreeMoveAbove", "mv_carries"),
        ("subtree_descend_to.xml", "SubTreeDescendTo", "dz_carries"),
        ("subtree_gripper.xml", "SubTreeGripper", "gr_carries"),
        ("subtree_lift.xml", "SubTreeLift", "lf_carries"),
    ):
        elements = _sequence(TREE_DIR / file_name, tree_id)
        at = _index(elements, lambda e: e.get("ID") == "SetMpcPayload")
        node = elements[at]

        assert node.get("carries") == f"{{{carries_key}}}", file_name
        # Same source as the A2B planner's m_log / s_log_8_*.
        assert node.get("mass") == "{payload_mass}", file_name
        assert [node.get(f"s_log_8_{axis}") for axis in "xyz"] == [
            "{s_log_8_x}",
            "{s_log_8_y}",
            "{s_log_8_z}",
        ], file_name
        # Before the move, or the MPC drives this one against the last payload.
        assert at < _index(elements, lambda e: e.get("ID") in MOVE_SUBTREES), file_name


def test_grip_marks_the_first_carrying_move_as_carrying():
    elements = _sequence(TREE_DIR / "subtree_pick_block.xml", "SubTreePickFromHover")
    at = _index(
        elements,
        lambda e: e.get("output_key") == "lf_carries" and e.get("value") == "true",
    )

    # The offset is only on the blackboard once the capture has run, and the lift
    # is the first move that carries the block.
    assert at > _index(elements, lambda e: e.get("ID") == "CaptureBlockGraspOffset")
    assert at < _index(elements, lambda e: e.get("ID") == "SubTreeLift")


def test_the_place_tells_the_mpc_it_still_carries():
    # SubTreeResumeCarry (274) hands its measured block centre on and sets no payload of
    # its own since 341, so the place's first move is what states it.
    elements = _sequence(TREE_DIR / "subtree_place_block.xml", "SubTreePlaceBlock")
    at = _index(
        elements,
        lambda e: e.get("output_key") == "mv_carries" and e.get("value") == "true",
    )
    assert at < _index(
        elements,
        lambda e: e.get("ID") in ("SubTreeMoveAbove", "SubTreePlaceSimple"),
    )


def test_release_clears_the_payload_after_the_jaws_open():
    # The release is its own subtree since 289: early contact reaches it without a
    # touchdown, and a payload left set there would drive the next move loaded.
    # Its own call, not SubTreeLift's: that one is behind a plan call and the approval
    # press, and an abort there would leave the payload set on an empty hook.
    elements = _sequence(TREE_DIR / "subtree_place_block.xml", "SubTreeRelease")
    assert "SubTreeRelease" in [
        e.get("ID")
        for e in _sequence(TREE_DIR / "subtree_place_block.xml", "SubTreeTouchdown")
    ]
    at = _index(elements, lambda e: e.get("ID") == "SetMpcPayload")

    assert elements[at].get("carries") == "false"
    assert elements[at].get("mass") is None, "a released hook carries nothing"

    # Until the gripper subtree has run, the MPC is still driving a loaded crane.
    gripper = [i for i, e in enumerate(elements) if e.get("ID") == "SubTreeGripper"]
    assert gripper and at > gripper[-1]


def test_the_plugin_is_loadable_by_the_bt_server():
    # Without this the tree fails to construct and the panel reports the
    # misleading "File exists but can't be loaded".
    with (PACKAGE_DIR / "config" / "bt_server_override.yaml").open() as handle:
        override = yaml.safe_load(handle)
    names = override["/**"]["ros__parameters"]["plugin_lib_names"]
    assert "BT_cb_set_mpc_payload_action" in names
