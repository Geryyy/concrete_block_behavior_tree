"""Issue 303: nothing swallows a missing measurement any more.

Lab day 2026-11-09: no lidar pose came back for 32 s, every `REFINE_*` sat in a
`ForceSuccess`, and the run went on with FK -- the grip 6 cm off centre, the grasp offset
read back from FK, and block_0's FK pose written as a measurement and used as the stacking
target. The behavioural half of this is in test_tree_load.cpp; what is pinned here is that no
new wrapper can reintroduce the pass-through.
"""

from pathlib import Path
from xml.etree import ElementTree

TREE_DIR = Path(__file__).resolve().parents[1] / "behavior_trees"
# The operator drives this one step by step and decides himself what an unmeasured pass means.
OPERATOR_TREE = "operator_guided_single_block.xml"


def _parents(root):
    return {child: parent for parent in root.iter() for child in parent}


def _mandatory(root):
    """The nodes whose failure has to reach the subtree's root."""
    for node in root.iter():
        name = node.get("ID") or node.tag
        if name == "CaptureBlockGraspOffset":
            yield node
        elif name == "RunPoseEstimation" and node.get("mode", "").startswith("REFINE_"):
            yield node


def test_no_refine_or_capture_is_forced_to_succeed():
    for xml in sorted(TREE_DIR.glob("*.xml")):
        if xml.name == OPERATOR_TREE:
            continue
        root = ElementTree.parse(xml).getroot()
        parents = _parents(root)
        for node in _mandatory(root):
            ancestors = []
            while node in parents:
                node = parents[node]
                ancestors.append(node.get("ID") or node.tag)
            assert "ForceSuccess" not in ancestors, (xml.name, ancestors)


def test_the_scene_discovery_re_ask_keeps_its_wrapper():
    """It only fetches the next task, and the next task is gated anyway."""
    root = ElementTree.parse(TREE_DIR / "wall_assembly.xml").getroot()
    parents = _parents(root)
    discovery = [
        node
        for node in root.iter("RunPoseEstimation")
        if node.get("mode") == "SCENE_DISCOVERY"
    ]
    assert len(discovery) == 1
    assert parents[discovery[0]].tag == "ForceSuccess"


def test_the_discovery_re_ask_is_only_for_an_answer_without_a_pose():
    """A service or TF error must not reach the Scripts of the branch built for a late block.

    GetNextAssemblyTask returns FAILURE for a finished plan, for a slot whose block is not
    known yet, and for an error. On 11-09 the error path left `pickup_x` unset and the
    branch's `Script` threw. `plan_ok` separates "the server answered" from the rest.
    """
    root = ElementTree.parse(TREE_DIR / "wall_assembly.xml").getroot()
    gate = [
        node.get("code")
        for node in root.iter("ScriptCondition")
        if "plan_has_task" in (node.get("code") or "")
    ]
    assert gate == ["plan_ok && plan_has_task"], gate
    # And it is defined before the first ask, or the condition itself throws.
    setup = " ".join(node.get("code", "") for node in root.iter("Script"))
    assert "plan_ok := false" in setup
    ask = next(root.iter("GetNextAssemblyTask"))
    assert ask.get("plan_ok") == "{plan_ok}"


def test_the_placed_pose_is_written_coarse():
    """FK x the grasp offset is not a measurement; the post-place refine is what makes it one.

    The plugin's defaults are POSE_COARSE and a confidence under 1, so a tree that passes
    neither gets them. block_0's FK pose went in as POSE_PRECISE conf 1.0 and became task 2's
    stacking target.
    """
    for xml in sorted(TREE_DIR.glob("*.xml")):
        root = ElementTree.parse(xml).getroot()
        for node in root.iter():
            if node.get("ID") == "WriteBlockPoseFromGripper":
                assert node.get("pose_status") is None, xml.name
                assert node.get("confidence") is None, xml.name
