"""The MPC payload is set at the grasp and cleared at the release (issue 175).

What the node does with its inputs is pinned in test_set_mpc_payload.cpp. What can
only be pinned here is that the pick and place subtrees still call it, from the
right blackboard keys and in the right place: the grip call has to sit after the
grasp offset is captured and before the first carrying move, the release call
after the jaws open.
"""

from pathlib import Path
from xml.etree import ElementTree

import yaml

PACKAGE_DIR = Path(__file__).resolve().parents[1]
TREE_DIR = PACKAGE_DIR / "behavior_trees"


def _sequence(tree_file, tree_id):
    root = ElementTree.parse(tree_file).getroot()
    tree = next(e for e in root.iter("BehaviorTree") if e.get("ID") == tree_id)
    return list(tree.iter())


def _index(elements, predicate):
    return next(i for i, e in enumerate(elements) if predicate(e))


def test_grip_hands_the_mpc_the_block_the_planner_already_gets():
    elements = _sequence(TREE_DIR / "subtree_pick_block.xml", "SubTreePickFromHover")
    at = _index(elements, lambda e: e.get("ID") == "SetMpcPayload")
    node = elements[at]

    assert node.get("carries") == "true"
    # Same source as the A2B planner's m_log / s_log_8_* and PublishGripperState.
    assert node.get("mass") == "{payload_mass}"
    assert [node.get(f"s_log_8_{axis}") for axis in "xyz"] == [
        "{s_log_8_x}",
        "{s_log_8_y}",
        "{s_log_8_z}",
    ]

    # The offset is only on the blackboard once the capture has run, and the lift
    # is the first move that carries the block.
    assert at > _index(elements, lambda e: e.get("ID") == "CaptureBlockGraspOffset")
    assert at < _index(elements, lambda e: e.get("ID") == "SubTreeLift")


def test_release_clears_the_payload_after_the_jaws_open():
    elements = _sequence(TREE_DIR / "subtree_place_block.xml", "SubTreePlaceFinish")
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
