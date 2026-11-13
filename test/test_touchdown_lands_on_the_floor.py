"""Issue 342: a block that stopped on its neighbour must not be let go there.

Course 0 is pitch 0.90 with a 0.90 m block, so the planned face-to-face gap is zero and the
entry's own lateral travel drives the block into the placed neighbour: in every oracle
example_wall run with noise seed 0 the stroke moved 1.3 cm and the load came off 16-19 cm
over the slot, which the touchdown's WaitForLoadOff took for a landing on its 0.20 m port
default. The two halves are the keep-out that leaves a gap to descend into, and the band
above which nothing opens the jaws.
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


def test_the_entry_moves_in_over_the_wall_and_then_only_downwards():
    """The one lateral move that crosses the slot's boundary happens at a height where the
    carried block's underside is over the neighbour's top (place_z + 0.75 against the
    place_z + 0.30 of a 0.6 m block); everything under it is vertical, so a face that meets
    its neighbour is grazed and not pressed. Landing short of the slot instead buys nothing:
    every block of a course takes the same offset, so the spacing is unchanged."""
    entry = _subtree("SubTreePlaceEntry")
    codes = [node.get("code") or "" for node in entry.iter("Script")]
    lateral = next(c for c in codes if "dz_x :=" in c)
    assert lateral == (
        "dz_x := place_x + off_x; dz_y := place_y + off_y; dz_z := place_z + 0.75"
    ), lateral
    heights = [c for c in codes if "dz_z :=" in c]
    assert heights == [lateral, "dz_z := place_z + 0.20"], heights
    # The second one moves nothing sideways, and the correction loop is the same height.
    approach = [
        node.get("code") or ""
        for node in _subtree("SubTreePlaceApproach").iter("Script")
    ]
    assert any("dz_z := place_z + 0.75" in c for c in approach), approach
    # And no keep-out: an along offset every block of the course shares is not a gap.
    assert "place_entry_keep" not in " ".join(codes + approach)


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
