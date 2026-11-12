"""Issue 323: the site is scanned before the first task is asked for.

The binding (324) picks among the blocks the world model holds. Asking first and discovering
only when a slot's block is unknown hands it one block at a time, which is not a choice. What
the survey then does per pose is in test_survey_site.cpp; the order is pinned here.
"""

from pathlib import Path
from xml.etree import ElementTree

TREE_DIR = Path(__file__).resolve().parents[1] / "behavior_trees"


def _order(xml):
    """Node IDs/tags of one tree file in document order."""
    root = ElementTree.parse(TREE_DIR / xml).getroot()
    return [node.get("ID") or node.tag for node in root.iter()]


def test_the_survey_runs_before_the_first_ask():
    order = _order("wall_assembly.xml")
    assert order.index("SubTreeSurveySite") < order.index("GetNextAssemblyTask")


def test_the_survey_runs_once_and_not_per_block():
    """Inside the loop it would re-drive both poses before every pick."""
    root = ElementTree.parse(TREE_DIR / "wall_assembly.xml").getroot()
    loop = next(e for e in root.iter() if e.get("name") == "assembly_loop")
    assert [e.get("ID") for e in loop.iter()].count("SubTreeSurveySite") == 0


def test_every_survey_pose_is_moved_to_before_it_is_discovered_from():
    order = _order("subtree_survey_site.xml")
    assert order.index("IterateSurveyPoses") < order.index("SubTreeMoveAbove")
    assert order.index("SubTreeMoveAbove") < order.index("RunPoseEstimation")
    root = ElementTree.parse(TREE_DIR / "subtree_survey_site.xml").getroot()
    discovery = next(root.iter("RunPoseEstimation"))
    assert discovery.get("mode") == "SCENE_DISCOVERY"
    # The count per pose is what the all-empty verdict is made of.
    assert discovery.get("block_count") == next(root.iter("IterateSurveyPoses")).get(
        "blocks"
    )
