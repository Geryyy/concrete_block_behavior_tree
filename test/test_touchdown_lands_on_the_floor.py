"""Issue 342: a block that stopped on its neighbour must not be let go there.

Course 0 is pitch 0.90 with a 0.90 m block, so the planned face-to-face gap is zero and the
entry's own lateral travel drives the block into the placed neighbour: in every oracle
example_wall run with noise seed 0 the stroke moved 1.3 cm and the load came off 16-19 cm
over the slot, which the touchdown's WaitForLoadOff took for a landing on its 0.20 m port
default. Here: the band above which nothing opens the jaws, and (345) the entry that comes
down beside the slot instead of over the neighbour.
"""

from pathlib import Path
from xml.etree import ElementTree

import yaml

PKG = Path(__file__).resolve().parents[1]
TREE_DIR = PKG / "behavior_trees"
PLACE_TREE = TREE_DIR / "subtree_place_block.xml"


def _subtree(tree_id):
    root = ElementTree.parse(PLACE_TREE).getroot()
    return next(e for e in root.iter("BehaviorTree") if e.get("ID") == tree_id)


def test_the_entry_comes_down_beside_the_slot_and_moves_in_low():
    """Issue 345: down on the open side first, then in at place_z + 0.35, so the block never
    crosses a neighbour's top edge; the stroke then holds where it ended, not the target."""
    codes = [
        node.get("code") or "" for node in _subtree("SubTreePlaceEntry").iter("Script")
    ]
    moves = [c for c in codes if "dz_x :=" in c]
    assert moves == [
        "dz_x := place_approach_x + off_x; dz_y := place_approach_y + off_y; "
        "dz_z := place_z + 0.35",
        "dz_x := place_x + off_x; dz_y := place_y + off_y",
        "dz_x := place_x + off_x + off_b_x; dz_y := place_y + off_y + off_b_y; "
        "ex_until_landed := false; ex_watch_landed := true; load_off := false",
        "dz_x := meas_x + off_x; dz_y := meas_y + off_y",
    ], moves


def test_the_touchdown_band_is_the_servers_parameter_not_the_nodes_default():
    """The node's 0.20 m default accepted a block resting on its neighbour. It is a
    lab-day decision -- on the machine the FK height carries the loaded boom's bend -- so
    the tree leaves the port unset and the server parameter carries it."""
    gates = [
        node
        for node in ElementTree.parse(TREE_DIR / "cbs_subtree_a2b_movement.xml")
        .getroot()
        .iter()
        if node.get("ID") == "WaitForLoadOff"
    ]
    landing = next(g for g in gates if g.get("_skipIf") == "!ex_until_landed")
    assert landing.get("max_above_target") is None
    header = PKG / "include/concrete_block_behavior_tree/plugins/condition"
    ports = (header / "wait_for_load_off.hpp").read_text()
    assert 'BT::InputPort<double>("max_above_target", "' in ports, "no port default"
    override = yaml.safe_load((PKG / "config/bt_server_override.yaml").read_text())
    # A range, not the number: the file tells a lab day to raise it, and on the real launch
    # that file is the only way to.
    assert 0.0 < override["/**"]["ros__parameters"]["touchdown_max_above_target"] < 0.20


def test_a_high_touchdown_lifts_clear_and_enters_once_more_without_releasing():
    """Never above the band: the release is the Fallback's sibling, not inside either
    branch, so a stroke that fails takes the jaws nowhere near open."""
    touchdown = _subtree("SubTreeTouchdown")
    fallback, release = list(next(touchdown.iter("Sequence")))
    assert release.get("ID") == "SubTreeRelease"
    first, retry = list(fallback)
    assert first.get("ID") == "SubTreeTouchdownStroke"
    # Only a load-off is retried: a stroke that reached its kinematic floor or timed out
    # must not be flown into the ground a second time, which a Retry would do.
    guard = next(node for node in retry if node.tag == "ScriptCondition")
    assert guard.get("code") == "load_off"
    assert [node.get("ID") for node in retry.iter("SubTree")] == [
        "SubTreeLift",
        "SubTreePlaceEntry",
        "SubTreeTouchdownStroke",
    ]
    # Lifted carrying, and at the run's own pace: the stroke failed before it could put
    # slow_down back, and SubTreeLift multiplies whatever it finds.
    assert [
        node.get("value")
        for node in retry.iter("SetBlackboard")
        if node.get("output_key") == "lf_carries"
    ] == ["true"]
    codes = [node.get("code") for node in retry.iter("Script")]
    assert codes.index("slow_down := sd_saved") == 0, codes
    assert "SubTreeRelease" not in [node.get("ID") for node in retry.iter("SubTree")]
