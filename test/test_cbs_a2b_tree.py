"""Static checks on the CBS A2B trees.

Whether the tree actually runs is a graph-level, human-run gate (PRD §14). What can
be pinned offline is the contract that made these files necessary: on the new stack
`crane_supervisor` is the sole caller of /controller_manager/switch_controller, so no
CBS A2B tree may carry a SwitchController node -- and the timber-backed trees, which
still own their controller, must stay exactly as they are.
"""

from pathlib import Path
from xml.etree import ElementTree

import yaml

PACKAGE_DIR = Path(__file__).resolve().parents[1]
TREE_DIR = PACKAGE_DIR / "behavior_trees"
CATALOG = PACKAGE_DIR / "config" / "bt_panel_catalog.yaml"

CBS_SUBTREE = TREE_DIR / "cbs_subtree_a2b_movement.xml"
CBS_MOVE_EMPTY = TREE_DIR / "cbs_move_empty_pzs100.xml"


def _nodes(tree_file):
    """Every element of a BT file, as (tag, ID attribute) pairs."""
    root = ElementTree.parse(tree_file).getroot()
    return [(element.tag, element.get("ID")) for element in root.iter()]


def _switches(tree_file):
    return [
        node for node in _nodes(tree_file) if "SwitchController" in (node[0], node[1])
    ]


def _catalog_entries():
    with CATALOG.open() as handle:
        catalog = yaml.safe_load(handle)
    return catalog["behavior_trees"]


def test_cbs_a2b_subtree_leaves_switching_to_the_supervisor():
    assert not _switches(CBS_SUBTREE), (
        "The CBS A2B subtree must not switch trajectory_controller_a2b; "
        "crane_supervisor owns it"
    )
    assert not _switches(CBS_MOVE_EMPTY), (
        "The CBS move-empty tree must not switch any controller"
    )


def test_cbs_a2b_subtree_keeps_approval_and_trajectory_following():
    nodes = _nodes(CBS_SUBTREE)

    assert ("BehaviorTree", "SubTreeCbsA2BMovement") in nodes
    assert ("Decorator", "GetUserApproval") in nodes, "operator approval gate is gone"
    assert ("Action", "FollowJointTrajectory") in nodes, "trajectory following is gone"


def test_cbs_move_empty_uses_the_cbs_subtree():
    root = ElementTree.parse(CBS_MOVE_EMPTY).getroot()

    assert root.get("main_tree_to_execute") == "CBS Move Empty PZS100"

    includes = [element.get("path") for element in root.iter("include")]
    assert includes == [
        "install/concrete_block_behavior_tree/share/concrete_block_behavior_tree"
        "/behavior_trees/cbs_subtree_a2b_movement.xml"
    ], "the CBS tree must include its own subtree, not epsilon's"

    subtrees = [element.get("ID") for element in root.iter("SubTree")]
    assert subtrees == ["SubTreeCbsA2BMovement"]

    # The planning call and the approval condition are what the harness exercises.
    nodes = _nodes(CBS_MOVE_EMPTY)
    assert ("CalcA2BMovement", None) in nodes
    assert ("Condition", "CheckUserApproval") in nodes
    assert ("Decorator", "WaitGoalPose") in nodes

    # log_state_estimator is not spawned on this profile; CalcGripState would throw
    # during tree construction. Same reason as move_empty_pzs100.xml.
    assert ("CalcGripState", None) not in nodes


def test_bt_panel_catalog_offers_the_cbs_tree_and_keeps_the_timber_ones():
    entries = _catalog_entries()

    assert entries[:3] == [
        {
            "name": "Basic pick and place",
            "path": "/behavior_trees/basic_pick_and_place.xml",
        },
        {"name": "Wall assembly", "path": "/behavior_trees/wall_assembly.xml"},
        {
            "name": "Move empty (A2B to RViz goal)",
            "path": "/behavior_trees/move_empty_pzs100.xml",
        },
    ], "the existing catalog entries must stay unchanged"

    assert {
        "name": "CBS move empty (A2B to RViz goal, new stack)",
        "path": "/behavior_trees/cbs_move_empty_pzs100.xml",
    } in entries

    for entry in entries:
        assert (PACKAGE_DIR / entry["path"].lstrip("/")).is_file(), entry["path"]


def test_timber_backed_trees_still_own_their_controller():
    # Not a style preference: the timber chain spawns trajectory_controller_a2b
    # --inactive and nothing else switches it, so removing these would break it.
    assert _switches(TREE_DIR / "subtree_execute_trajectory.xml")
    assert not _switches(TREE_DIR / "move_empty_pzs100.xml"), (
        "move_empty_pzs100.xml never switched a controller itself; it delegates to "
        "epsilon's subtree_a2b_movement.xml"
    )
