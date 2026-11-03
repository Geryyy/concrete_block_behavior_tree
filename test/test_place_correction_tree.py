"""Static check on the place correction loop (issue 256)."""

from pathlib import Path
from xml.etree import ElementTree

PLACE_TREE = (
    Path(__file__).resolve().parents[1] / "behavior_trees" / "subtree_place_block.xml"
)


def _subtree(tree_id):
    root = ElementTree.parse(PLACE_TREE).getroot()
    return next(
        element for element in root.iter("BehaviorTree") if element.get("ID") == tree_id
    )


def _scripts(tree_id):
    return [
        (element.tag, element.get("code"))
        for element in _subtree(tree_id).iter()
        if element.get("code") is not None
    ]


def _node_order(tree_id):
    """Node names in document order; inside the Repeat body that is also tick order."""
    return [element.get("ID") or element.tag for element in _subtree(tree_id).iter()]


def test_place_correction_converges_laterally_only():
    """
    The correction cycle must not test or shift the waypoint's z.

    miss_z is a reference mismatch, not a miss: the grip server turns a commanded block
    centre into a K8 target with the nominal block_grip_z_offset + tcp_z_offset (1.05 m)
    while MeasureBlockFromGripper reads the block back through the lidar-refined grasp
    offset (1.005 m in run2_pp). No cycle removes those 45 mm, so a 3-D convergence test
    never fires and every place pays all 3 cycles. Nothing downstream wants the vertical
    correction either -- SubTreePlaceFinish re-commands dz_z absolutely.
    """
    codes = [code for _, code in _scripts("SubTreePlaceApproach")]
    convergence = [code for code in codes if "still_off := miss" in code]
    assert len(convergence) == 1, codes
    assert "miss_z" not in convergence[0]
    # No cycle may move the waypoint vertically, by off_z or otherwise.
    assert not [code for code in codes if "dz_z :=" in code and "off" in code]


def test_the_approach_ends_on_a_measurement():
    """
    Nothing may leave place_approach after a shift that was not measured (issue 281).

    `Repeat num_cycles="3"` succeeds after the third corrective descent, so the tree used to
    hand SubTreePlaceFinish a waypoint it had only commanded. The descent is the last node of
    the loop body, so "the last descent precedes the last measurement" holds only if a
    measurement follows the loop.
    """
    order = _node_order("SubTreePlaceApproach")
    last_descend = max(i for i, name in enumerate(order) if name == "SubTreeDescendTo")
    last_measure = max(
        i for i, name in enumerate(order) if name == "MeasureBlockFromGripper"
    )
    assert last_descend < last_measure, order
    # And that measurement gates: 2.5e-3 is the 5 cm lateral miss, 6.4e-3 the miss_z band.
    codes = [code for _, code in _scripts("SubTreePlaceApproach")]
    gates = [code for code in codes if "0.0025" in code and "pass_verdict" not in code]
    assert len(gates) == 2, codes


def test_the_in_air_loop_starts_from_the_operators_correction_and_the_auto_path_from_zero():
    """
    A manual hover correction must not be undone by the loop's first descent, nor counted
    twice at the finish; with no correction the automatic tree is unchanged.
    """
    codes = [code for _, code in _scripts("SubTreePlaceApproach")]
    assert codes.index(
        next(c for c in codes if "off_x := seed_off_x" in c)
    ) < codes.index(
        "dz_x := place_approach_x + off_x; dz_y := place_approach_y + off_y"
    ), codes
    # The miss stays against the unshifted waypoint, so off_x is the total shift.
    assert any("miss_x := place_approach_x - meas_x" in c for c in codes), codes
    assert "seed_off_x := 0.0" in _scripts("SubTreePlaceBlock")[0][1]

    operator = ElementTree.parse(PLACE_TREE.parent / "operator_guided_single_block.xml")
    correct = next(
        e
        for e in operator.getroot().iter("Sequence")
        if e.get("name") == "correct_hover_by_residual"
    )
    assert [
        e.get("code") for e in correct.iter("Script") if "seed_off_x" in e.get("code")
    ] == [
        "seed_off_x := corrected_hover_x - place_approach_x; "
        "seed_off_y := corrected_hover_y - place_approach_y"
    ]
    assert not [
        e
        for e in correct.iter("SetBlackboard")
        if e.get("output_key") in ("place_x", "place_y")
    ]


def test_a_correction_counts_as_done_only_after_its_move():
    """A failed corrective descent must fail the approach: corr_done set before it would let
    the Fallback after the loop succeed with the block never sent."""
    body = next(_subtree("SubTreePlaceApproach").iter("Repeat"))
    order = [
        (element.get("ID") or element.tag, element.get("code") or "")
        for element in body.iter()
    ]
    descend = next(i for i, (name, _) in enumerate(order) if name == "SubTreeDescendTo")
    after_move = [
        i for i, (_, code) in enumerate(order) if "corr_done := !still_off" in code
    ]
    assert after_move and min(after_move) > descend, order
