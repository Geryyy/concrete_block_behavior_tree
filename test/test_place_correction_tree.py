"""Static check on the place correction loop (issue 256) and early ground contact (289)."""

from pathlib import Path
from xml.etree import ElementTree

PLACE_TREE = (
    Path(__file__).resolve().parents[1] / "behavior_trees" / "subtree_place_block.xml"
)
CYCLE_TREE = PLACE_TREE.parent / "subtree_pick_and_place_block.xml"


def _subtree(tree_id, tree_file=None):
    root = ElementTree.parse(tree_file or PLACE_TREE).getroot()
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


def _ancestors(subtree):
    """Child -> parent over one subtree, for "is this guarded anywhere above" tests."""
    return {child: parent for parent in subtree.iter() for child in parent}


def test_every_loaded_descent_runs_the_landed_watch():
    """Issue 289: the ground can end any of them, not only the touchdown.

    Lab day 2026-11-04: the block reached the ground 0.3-0.5 m before the waypoint and the
    descent went on pushing, with the boom's p_A falling from 190 to 86 bar, because only
    SubTreeTouchdown watched the load. Each of these owns a loaded descent, and exactly one
    of them -- the touchdown -- is the descent that *ends* on landing.
    """
    watched = ("SubTreePlaceSimple", "SubTreePlaceApproach", "SubTreePlaceFinish")
    for tree_id in watched:
        codes = [code for _, code in _scripts(tree_id)]
        assert [c for c in codes if "ex_watch_landed := true" in c], tree_id
    setdown = [
        node.get("code", "")
        for node in _subtree("SubTreeSetDown", CYCLE_TREE).iter("Script")
    ]
    assert [c for c in setdown if "ex_watch_landed := true" in c], setdown

    touchdown = [code for _, code in _scripts("SubTreeTouchdown")]
    assert [c for c in touchdown if "ex_until_landed := true" in c], touchdown
    for tree_id in watched:
        codes = [code for _, code in _scripts(tree_id)]
        assert not [c for c in codes if "ex_until_landed := true" in c], tree_id


def test_nothing_measures_or_descends_again_after_an_early_contact():
    """A block on the ground reads as a descent that stopped 0.3-0.8 m short.

    Every pass after the contact would fail on its own height gate, and every corrective
    descent would push into the ground. So the whole correction chain is skipped on
    load_off: the measuring branches behind the guard, the loop bodies breaking on it.
    """
    for tree_id in ("SubTreePlaceSimple", "SubTreePlaceApproach"):
        subtree = _subtree(tree_id)
        parents = _ancestors(subtree)
        measures = [
            node
            for node in subtree.iter()
            if (node.get("ID") or node.tag)
            in ("MeasureBlockFromGripper", "SubTreePlaceMiss")
        ]
        assert measures, tree_id
        for node in measures:
            guards = []
            while node in parents:
                node = parents[node]
                guards.append(node.get("_skipIf"))
            assert "load_off" in guards, (tree_id, guards)

        # The corrective descent is the last node of the loop body, so the break after it
        # is what keeps the next cycle from measuring. corr_done, as for convergence,
        # tells the Fallback around the loop that this failure was benign.
        body = next(subtree.iter("Repeat"))
        order = [
            (node.get("ID") or node.tag, node.get("code") or "") for node in body.iter()
        ]
        descend = max(
            i for i, (name, _) in enumerate(order) if name == "SubTreeDescendTo"
        )
        breaks = [i for i, (_, code) in enumerate(order) if code == "!load_off"]
        assert breaks and min(breaks) > descend, order
        assert [code for _, code in order[descend:] if code == "corr_done := true"]


def test_early_contact_lets_go_only_inside_the_lateral_band():
    """The ground decides the height, the plan only the lateral position.

    place_z comes from a ground-model snapshot that was 0.11 m out on the lab day, so a
    block resting laterally on its slot is placed, however far that is from place_z. A
    block resting anywhere else has to come off the ground again: leaving it there puts it
    where the next block is planned, and the existing carrying lift is what clears it.
    """
    handler = _subtree("SubTreeEarlyContact")
    release = next(
        node for node in handler.iter() if node.get("ID") == "SubTreeRelease"
    )
    parents = _ancestors(handler)
    band = [
        node.get("code")
        for node in parents[release].iter("ScriptCondition")
        if "miss_x" in (node.get("code") or "")
    ]
    assert band == ["miss_x * miss_x + miss_y * miss_y < 0.0025"], band
    # The band alone picks the branch. Under a Fallback a release that failed after the
    # jaws opened would fall through to the lift below, which runs lf_carries true: 880 kg
    # of feedforward onto an empty hook.
    assert parents[release].tag == "IfThenElse"
    _, chosen, other = list(parents[release])
    assert chosen is release and other.get("ID") == "ForceFailure"
    # Measured first, and against place_x/place_y, not the descent's shifted target.
    order = [node.get("ID") or node.tag for node in handler.iter()]
    assert order.index("MeasureBlockFromGripper") < order.index("SubTreeRelease")
    assert [
        node.get("code")
        for node in handler.iter("Script")
        if "miss_x :=" in node.get("code")
    ] == ["miss_x := place_x - meas_x; miss_y := place_y - meas_y"]
    # No height gate: there is nothing left to descend to, and no z to re-aim.
    assert "SubTreeDescendTo" not in order
    assert "SubTreeTouchdown" not in order
    # The other branch lifts clear and still fails the place.
    lift = next(node for node in handler.iter() if node.get("ID") == "SubTreeLift")
    assert parents[parents[lift]].get("ID") == "ForceFailure"
    assert [
        node.get("value")
        for node in handler.iter("SetBlackboard")
        if node.get("output_key") == "lf_carries"
    ] == ["true"]


def test_early_contact_replaces_the_touchdown_everywhere_a_descent_is_watched():
    """The release is the touchdown's tail, so early contact must reach it without one.

    A tree that ran SubTreeTouchdown after the contact would command the final stroke to
    place_z - 0.20 against a block already on the ground, which is the push this issue
    removes; one that only failed would abandon a block standing on its slot.
    """
    for tree_id, tree_file in (
        ("SubTreePlaceSimple", None),
        ("SubTreePlaceFinish", None),
        ("SubTreeSetDown", CYCLE_TREE),
    ):
        subtree = _subtree(tree_id, tree_file)
        branch = next(
            node
            for node in subtree.iter("IfThenElse")
            if any(child.get("ID") == "SubTreeEarlyContact" for child in node)
        )
        condition, early, normal = list(branch)
        assert condition.get("code") == "load_off", tree_id
        assert early.get("ID") == "SubTreeEarlyContact", tree_id
        touchdowns = [
            node for node in normal.iter() if node.get("ID") == "SubTreeTouchdown"
        ]
        assert len(touchdowns) == 1, tree_id
        # And it is the only touchdown left in that path.
        assert (
            len(
                [
                    node
                    for node in subtree.iter()
                    if node.get("ID") == "SubTreeTouchdown"
                ]
            )
            == 1
        ), tree_id

    # The angled entry is a descent too, so a contact in the approach before it has to
    # skip it rather than fly it with the block on the ground.
    finish = _subtree("SubTreePlaceFinish")
    entry = next(
        node
        for node in finish.iter("Sequence")
        if any(child.tag == "Decorator" for child in node)
    )
    assert entry.get("_skipIf") == "load_off"


def test_the_simple_hover_clears_the_overrun():
    """0.5 m left 4 cm: bend 0.17 plus the boom's downward overrun 0.29 (lab day 11-04).

    The waypoint and the gate that measures against it are the same height by
    construction, and the touchdown's pacing is for the stroke under it -- 0.25 m at
    td_scale 2.0, so a 1.0 m stroke needs 4x that to meet the ground at the same speed.
    Through td_scale, not slow_down: scaled around the whole subtree, the release's empty
    lift ran at the touchdown's pace too (issue 290).
    """
    codes = [code for _, code in _scripts("SubTreePlaceSimple")]
    assert [c for c in codes if "dz_z := place_z + 0.80" in c], codes
    miss = [code for _, code in _scripts("SubTreePlaceMiss") if "miss_z :=" in code]
    assert all("place_z + 0.80 - meas_z" in code for code in miss), miss
    assert [c for c in codes if "td_scale := 8.0" in c], codes
    assert not [c for c in codes if "slow_down :=" in c], codes
