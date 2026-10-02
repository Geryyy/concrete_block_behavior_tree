"""Static checks on the CBS A2B trees.

Whether the tree actually runs is a graph-level, human-run gate (PRD §14). What can
be pinned offline is the contract that made these files necessary: the CBS subtree
executes a move the way the timber one does -- approval gating an activate / execute
/ deactivate sequence -- differing only in which child runs the move. The
timber-backed trees, which own their controller the same way, must stay exactly as
they are.
"""

from pathlib import Path
from xml.etree import ElementTree

import yaml

PACKAGE_DIR = Path(__file__).resolve().parents[1]
TREE_DIR = PACKAGE_DIR / "behavior_trees"
CATALOG = PACKAGE_DIR / "config" / "bt_panel_catalog.yaml"

CBS_SUBTREE = TREE_DIR / "cbs_subtree_a2b_movement.xml"
CBS_MOVE_EMPTY = TREE_DIR / "cbs_move_empty_pzs100.xml"


def _nodes(tree_file):
    """Every element of a BT file, as (tag, ID attribute) pairs."""
    root = ElementTree.parse(tree_file).getroot()
    return [(element.tag, element.get("ID")) for element in root.iter()]


def _switches(tree_file):
    return [
        node for node in _nodes(tree_file) if "SwitchController" in (node[0], node[1])
    ]


def _catalog_entries():
    with CATALOG.open() as handle:
        catalog = yaml.safe_load(handle)
    return catalog["behavior_trees"]


def test_cbs_a2b_subtree_gates_activation_behind_approval():
    """
    Approval must gate the *activation*, which is what gates the motion.

    On `cbs_mpc_active` this tree sends no goal: the planner publishing
    /crane/reference is what starts the MPC driving, and the horizon reaches the
    JTC over ~/joint_trajectory, which the JTC reads only between on_activate and
    on_deactivate. So an activation outside the approval decorator -- or no
    activation at all, which is what left the horizon going to an inactive
    controller -- is a move nobody confirmed.
    """
    root = ElementTree.parse(CBS_SUBTREE).getroot()
    approval = next(
        element
        for element in root.iter("Decorator")
        if element.get("ID") == "GetUserApproval"
    )
    activations = [
        element
        for element in approval.iter("SwitchController")
        if element.get("activate_controllers") == "trajectory_controller_a2b"
    ]
    assert activations, "nothing activates the controller inside the approval gate"

    # Every failure path still has to put it back, or an abandoned move leaves an
    # active controller behind for the next horizon to drive.
    deactivations = [
        element
        for element in root.iter("SwitchController")
        if element.get("deactivate_controllers") == "trajectory_controller_a2b"
    ]
    assert len(deactivations) >= 2, "the ForceFailure branch must deactivate too"

    assert not _switches(CBS_MOVE_EMPTY), (
        "The CBS move-empty tree delegates switching to the subtree"
    )

    # BEST_EFFORT is the node's default and the reason owning the switch is safe:
    # controller_manager warns "already active" and drops that controller from the
    # request, where STRICT would fail the whole switch. Profiles that spawn the
    # controller active run no tree today, but a STRICT switch here would fault the
    # move the day one does.
    for switch in root.iter("SwitchController"):
        assert switch.get("strictness") is None, (
            "leave strictness at BEST_EFFORT; a STRICT switch fails outright when "
            "the world is already what it asks for"
        )


def test_cbs_a2b_subtree_keeps_approval_and_trajectory_following():
    nodes = _nodes(CBS_SUBTREE)

    assert ("BehaviorTree", "SubTreeCbsA2BMovement") in nodes
    assert ("Decorator", "GetUserApproval") in nodes, "operator approval gate is gone"
    assert ("Action", "FollowJointTrajectory") in nodes, "trajectory following is gone"


def test_cbs_a2b_subtree_lets_exactly_one_commander_run_a_move():
    """Issue 148: the action goal and the MPC must never both be live.

    The branch is bound at launch, not in the file: WaitForMoveComplete reads
    `a2b_mpc_drives` and skips itself when it is false, and the action carries
    the opposite guard. Losing either guard puts two commanders on the JTC.
    """
    root = ElementTree.parse(CBS_SUBTREE).getroot()

    wait = next(root.iter("WaitForMoveComplete"))
    assert wait.get("mpc_drives") == "{mpc_drives}"
    assert wait.get("trajectory") == "{trajectory}"

    # The /a2b_movement answer is the canonical eight. Gating on the passive pair
    # would hold the move against a sway value the plan never aimed, read out
    # through an uncalibrated offset; gating on q9 would hold it against a rail
    # the controller's own goal tolerance (0.00) does not check.
    checked = wait.get("joints").split(",")
    assert "theta6_tip_joint" not in checked
    assert "theta7_tilt_joint" not in checked
    assert "q9_left_rail_joint" not in checked
    # Each caller sets the tolerance; move_above asks less than the JTC's 0.05.
    assert wait.get("goal_tolerance") == "{ex_goal_tolerance}"

    guard = next(
        element
        for element in root.iter("Fallback")
        if any(child.get("ID") == "FollowJointTrajectory" for child in element)
    )
    assert guard.get("_skipIf") == "mpc_drives"

    # The plugin has to be loadable, or the tree fails to construct and the
    # panel reports the misleading "File exists but can't be loaded".
    with (PACKAGE_DIR / "config" / "bt_server_override.yaml").open() as handle:
        override = yaml.safe_load(handle)
    server_params = override["/**"]["ros__parameters"]
    assert "BT_cb_wait_for_move_complete_condition" in server_params["plugin_lib_names"]
    # Default off: every profile but cbs_mpc_active, the real-crane launch
    # included, keeps the action goal.
    assert server_params["a2b_mpc_drives"] is False


def test_cbs_move_empty_uses_the_cbs_subtree():
    root = ElementTree.parse(CBS_MOVE_EMPTY).getroot()

    assert root.get("main_tree_to_execute") == "CBS Move Empty PZS100"

    includes = [element.get("path") for element in root.iter("include")]
    assert includes == [
        "behavior_trees/cbs_subtree_a2b_movement.xml"
    ], "the CBS tree must include its own subtree, not epsilon's"

    subtrees = [element.get("ID") for element in root.iter("SubTree")]
    assert subtrees == ["SubTreeCbsA2BMovement"]

    # The planning call and the approval condition are what the harness exercises.
    nodes = _nodes(CBS_MOVE_EMPTY)
    assert ("CalcA2BMovement", None) in nodes
    assert ("Action", "CheckUserApproval") in nodes
    assert ("Decorator", "WaitGoalPose") in nodes

    # log_state_estimator is not spawned on this profile; CalcGripState would throw
    # during tree construction. Same reason as move_empty_pzs100.xml.
    assert ("CalcGripState", None) not in nodes


def test_bt_panel_catalog_offers_the_cbs_tree_and_keeps_the_timber_ones():
    entries = _catalog_entries()

    assert entries[:3] == [
        {
            "name": "Basic pick and place",
            "path": "/behavior_trees/basic_pick_and_place.xml",
        },
        {"name": "Wall assembly", "path": "/behavior_trees/wall_assembly.xml"},
        {
            "name": "Move empty (A2B to RViz goal)",
            "path": "/behavior_trees/move_empty_pzs100.xml",
        },
    ], "the existing catalog entries must stay unchanged"

    assert {
        "name": "CBS move empty (A2B to RViz goal, new stack)",
        "path": "/behavior_trees/cbs_move_empty_pzs100.xml",
    } in entries

    for entry in entries:
        assert (PACKAGE_DIR / entry["path"].lstrip("/")).is_file(), entry["path"]


def test_timber_backed_trees_still_own_their_controller():
    # Not a style preference: the timber chain spawns trajectory_controller_a2b
    # --inactive and nothing else switches it, so removing these would break it.
    assert _switches(TREE_DIR / "subtree_execute_trajectory.xml")
    assert not _switches(TREE_DIR / "move_empty_pzs100.xml"), (
        "move_empty_pzs100.xml never switched a controller itself; it delegates to "
        "epsilon's subtree_a2b_movement.xml"
    )
