"""Static check on the place correction loop (issue 256)."""

from pathlib import Path
from xml.etree import ElementTree

PLACE_TREE = (
    Path(__file__).resolve().parents[1] / "behavior_trees" / "subtree_place_block.xml"
)


def _scripts(tree_id):
    root = ElementTree.parse(PLACE_TREE).getroot()
    subtree = next(
        element for element in root.iter("BehaviorTree") if element.get("ID") == tree_id
    )
    return [
        (element.tag, element.get("code"))
        for element in subtree.iter()
        if element.get("code") is not None
    ]


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
