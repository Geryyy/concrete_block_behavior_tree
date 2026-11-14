"""Issue 332: a dropped refine answer must not end a goal, and a misplaced block must.

The lab-day refine failures were transport -- the cloud stopped reaching the detector while
the PC recorded (docs/features/cbs-refine-timeout-analysis.md) -- so every mandatory refine
gets one spaced retry, and the duplicate that only gave a second chance to abort is gone. The
other half is the two gates that were missing: early contact released on a lateral miss alone,
whatever the block was resting on (readiness S1), and a block the post-place refine had to
move 15-31 cm was accepted as placed.
"""

from pathlib import Path
from xml.etree import ElementTree

TREE_DIR = Path(__file__).resolve().parents[1] / "behavior_trees"
REFINE_TREE = TREE_DIR / "subtree_refine.xml"
PLACE_TREE = TREE_DIR / "subtree_place_block.xml"

# Every caller of the shared in-hand measurement, and the window each asks for.
MEASURE_CALLERS = (
    ("subtree_pick_block.xml", "SubTreePickFromHover"),
    ("subtree_place_block.xml", "SubTreePlaceMiss"),
    ("subtree_place_block.xml", "SubTreePlaceApproach"),
    ("subtree_place_block.xml", "SubTreeEarlyContact"),
    ("subtree_pick_and_place_block.xml", "SubTreeResumeCarry"),
)


def _subtree(xml, tree_id):
    root = ElementTree.parse(TREE_DIR / xml).getroot()
    return next(e for e in root.iter("BehaviorTree") if e.get("ID") == tree_id)


def _refines(root):
    """The mandatory refines of one parsed tree: the ones whose failure ends the goal."""
    return [
        node
        for node in root.iter("RunPoseEstimation")
        if (node.get("mode") or "").startswith("REFINE_")
    ]


def _ids(element):
    return [node.get("ID") or node.tag for node in element.iter()]


def test_every_mandatory_refine_sits_under_one_spaced_retry():
    """A second ask 1 s later is a fresh draw; it costs 9 s on a cloud that is really gone."""
    root = ElementTree.parse(REFINE_TREE).getroot()
    refines = _refines(root)
    assert len(refines) == 2, "one REFINE_BLOCK, one REFINE_GRASPED, both shared"
    parents = {child: parent for parent in root.iter() for child in parent}
    for refine in refines:
        fallback = parents[refine]
        retry = parents[fallback]
        assert retry.get("ID") == "RetryUntilSuccessful", refine.get("mode")
        assert retry.get("num_attempts") == "2", refine.get("mode")
        # RetryUntilSuccessful re-ticks a synchronous child without letting time pass, so the
        # wait has to be a failing sibling branch, as subtree_move_above.xml's TF retry is.
        delay = next(node for node in fallback.iter() if node.get("delay_msec"))
        assert (delay.get("ID") or delay.tag) == "Delay"
        assert int(delay.get("delay_msec")) >= 1000

    # And no tree asks for one any other way, retry or not. The operator tree is the one
    # exception, as it is for 303: there the operator drives each step and its refine is
    # wrapped in a ForceSuccess with the FK pose as the deliberate fallback.
    for xml in sorted(TREE_DIR.glob("*.xml")):
        if xml not in (REFINE_TREE, TREE_DIR / "operator_guided_single_block.xml"):
            assert not _refines(ElementTree.parse(xml).getroot()), xml.name


def test_exactly_one_post_place_refine_and_no_second_status_write():
    """Both ran after the place and either could abort a physically finished wall (303).

    WriteBlockPoseFromGripper writes TASK_PLACED itself, so the loop's SetBlockTaskStatus and
    its REFINE_BLOCK duplicated the pair at the end of the place.
    """
    release = _subtree("subtree_place_block.xml", "SubTreeRelease")
    refines = [n for n in release.iter() if n.get("ID") == "SubTreeRefineBlock"]
    # One mandatory, plus the second opinion the misplaced gate asks for below (340).
    assert [n.get("_skipIf") for n in refines] == [None, "on_slot"]
    write = next(
        node for node in release.iter() if node.get("ID") == "WriteBlockPoseFromGripper"
    )
    assert write.get("task_status") == "3"

    loop = next(
        node
        for node in ElementTree.parse(TREE_DIR / "wall_assembly.xml").getroot().iter()
        if node.get("name") == "pick_and_place_one_block"
    )
    assert "SubTreeRefineBlock" not in _ids(loop)
    assert "SetBlockTaskStatus" not in _ids(loop)


def test_a_misplaced_block_fails_the_goal_by_name():
    """Refined against written: FK took the block for landed, the lidar moved it 15-31 cm.

    And it takes two refines, the smaller deciding: 333's wall5 aborted a wall whose course 0
    was within 2.6 cm of truth because one REFINE_BLOCK lost a face and moved block_2 0.109 m
    (340). Two that both read over the arm still fail, however far apart they are.
    """
    release = _subtree("subtree_place_block.xml", "SubTreeRelease")
    body = list(next(release.iter("Sequence")))
    gate = body[-1]
    assert gate.tag == "ScriptCondition"
    assert "misplaced" in gate.get("name").lower()
    assert gate.get("code") == "on_slot || on_slot_2"
    arms = [
        node.get("code")
        for node in body
        if node.tag == "Script" and "on_slot" in (node.get("code") or "")
    ]
    assert len(arms) == 2, "the same gate on each refine"
    for code in arms:
        assert (
            code.count("refine_x - written_x") == 2
            and code.count("refine_y - written_y") == 2
        )
        assert "0.01" in code
        # In z only *below* counts: the lidar reads a ground block 0.12-0.25 m too high
        # (readiness A3), so a symmetric z test would fail every place on the machine.
        assert "refine_z - written_z > -0.10" in code

    write = next(
        node for node in release.iter() if node.get("ID") == "WriteBlockPoseFromGripper"
    )
    assert [write.get(axis) for axis in "xyz"] == [f"{{written_{a}}}" for a in "xyz"]
    refine = next(
        node
        for node in _refines(ElementTree.parse(REFINE_TREE).getroot())
        if node.get("mode") == "REFINE_BLOCK"
    )
    assert [refine.get(axis) for axis in "xyz"] == [f"{{refine_{a}}}" for a in "xyz"]


def test_early_contact_releases_only_near_the_slot():
    """Load-off alone is also a block resting on a neighbour, the hedge or an occupied slot."""
    handler = _subtree("subtree_place_block.xml", "SubTreeEarlyContact")
    condition, chosen, _lift = list(next(handler.iter("IfThenElse")))
    assert chosen.get("ID") == "SubTreeRelease"
    # The lateral band it always had, and now the slot's own height: 0.30 m covers the lidar's
    # one-sided z bias on a ground block, where a neighbour's top is ~0.6 m up.
    assert "miss_x * miss_x + miss_y * miss_y < 0.0025" in condition.get("code")
    assert "miss_z * miss_z < 0.09" in condition.get("code")
    assert any(
        "miss_z := place_z - meas_z" in (node.get("code") or "")
        for node in handler.iter("Script")
    )
    # The trigger stays as it was: no height on the watch that calls it early contact.
    watch = next(
        node
        for node in ElementTree.parse(TREE_DIR / "cbs_subtree_a2b_movement.xml")
        .getroot()
        .iter()
        if node.get("name") == "Watch for early contact"
    )
    assert watch.get("max_above_target") == "-1.0"


def test_nothing_re_measures_after_the_touchdown():
    """It fed only the COARSE write the post-release refine overwrites, while its own failure
    left the block landed, the jaws shut and the block TASK_MOVE (303)."""
    ids = _ids(_subtree("subtree_place_block.xml", "SubTreeTouchdown"))
    assert "SubTreeMeasureInHand" not in ids and "CaptureBlockGraspOffset" not in ids
    assert ids.index("SubTreeTouchdownStroke") < ids.index("SubTreeRelease")
    stroke = _ids(_subtree("subtree_place_block.xml", "SubTreeTouchdownStroke"))
    assert "SubTreeMeasureInHand" not in stroke and "SubTreeDescendTo" in stroke


def test_the_in_hand_measurement_is_defined_once_and_parameterised_per_caller():
    definitions = [
        tree
        for xml in sorted(TREE_DIR.glob("*.xml"))
        for tree in ElementTree.parse(xml).getroot().iter("BehaviorTree")
        if tree.get("ID") == "SubTreeMeasureInHand"
    ]
    assert len(definitions) == 1
    for xml, tree_id in MEASURE_CALLERS:
        tree = _subtree(xml, tree_id)
        calls = _ids(tree).count("SubTreeMeasureInHand")
        # The window is the only difference between the copies, and the blackboard is shared
        # across the whole run, so every call site sets it.
        windows = [
            node
            for node in tree.iter("Script")
            if "mi_window_s :=" in (node.get("code") or "")
        ]
        assert calls and len(windows) == calls, tree_id
