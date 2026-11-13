"""Issue 329: the 11-09 network ended goals that were physically fine.

Five crane-wide 1.5 s DDS stalls, a `switch_controller` that answered after 1.05 s on a switch
it had already done, and a 9 s `/tf` freeze. Every one of those was inside a reply budget the
tree had set too low, so the budgets are what this guards -- not a retry around the goal.
Numbers from docs/features/cbs-labday-forensics.md section 2.
"""

from pathlib import Path
from xml.etree import ElementTree

TREES = Path(__file__).resolve().parents[1] / "behavior_trees"

# 1.5 s of stall is the observed worst case; 2 s is the budget the issue asks every reply wait
# to survive.
STALL_MS = 2000
# `OptimizeTargetAngle`'s own tf2 wait, which each retry attempt blocks for before the Delay
# (epsilon_crane_behavior_tree, see the comment at the call site in subtree_move_above.xml).
TF_LOOKUP_MS = 500
TF_FREEZE_MS = 10000


def _nodes(tree, node_id):
    """Every element of `tree` that is `node_id`, written either way BT.CPP allows."""
    for element in tree.iter():
        if element.tag == node_id or element.get("ID") == node_id:
            yield element


def _all_trees():
    return sorted(TREES.glob("*.xml"))


def test_reply_waits_survive_a_two_second_stall():
    short = {}
    seen = 0
    for path in _all_trees():
        root = ElementTree.parse(path).getroot()
        for node_id in ("SwitchController", "FollowJointTrajectory"):
            for element in _nodes(root, node_id):
                seen += 1
                timeout = int(element.get("server_timeout"))
                if timeout < STALL_MS:
                    short[f"{path.name}:{node_id}"] = timeout
    assert seen, (
        "no controller switch or trajectory goal in the trees -- test is vacuous"
    )
    assert not short, (
        f"server_timeout below {STALL_MS} ms: {short}. A 1.5 s DDS stall or a slow "
        "controller_manager reply then fails a step that succeeded."
    )


def test_the_target_angle_retry_covers_a_ten_second_tf_freeze():
    seen = 0
    for path in _all_trees():
        root = ElementTree.parse(path).getroot()
        for element in root.iter():
            if element.get("name") != "wait_out_tf_stall":
                continue
            seen += 1
            attempts = int(element.get("num_attempts"))
            delay = max(int(d.get("delay_msec")) for d in _nodes(element, "Delay"))
            # Attempt n starts its lookup at (n-1)*(wait + delay), so the cover is where the
            # LAST attempt starts, not where its trailing Delay ends.
            assert (attempts - 1) * (TF_LOOKUP_MS + delay) >= TF_FREEZE_MS, path.name
    # Both call sites of OptimizeTargetAngle: the approach and the descent.
    assert seen == 2, f"wait_out_tf_stall found at {seen} sites, expected 2"
