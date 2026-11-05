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

# Every caller of SubTreeTouchdown, which owns the one landing descent. Its gate's inputs
# are set per caller: the blackboard is shared across the whole run.
TOUCHDOWN_CALLERS = (
    ("subtree_place_block.xml", "SubTreePlaceSimple"),
    ("subtree_place_block.xml", "SubTreePlaceFinish"),
    ("subtree_pick_and_place_block.xml", "SubTreeSetDown"),
)


def _subtree_scripts(name, tree_id):
    tree = next(
        element
        for element in ElementTree.parse(TREE_DIR / name).getroot().iter("BehaviorTree")
        if element.get("ID") == tree_id
    )
    return [node.get("code") for node in tree.iter("Script")]


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


def test_cbs_a2b_subtree_waits_for_the_load_to_settle_after_the_action():
    """Issue 220: the action arm has to leave the load still too.

    The FollowJointTrajectory goal returns on positions and a goal_time, and
    nothing on this arm damps sway, so the tree descended into a swinging block.
    The settle half is the same node with `settle_only`, placed after the goal.
    """
    root = ElementTree.parse(CBS_SUBTREE).getroot()
    waits = list(root.iter("WaitForMoveComplete"))
    settle = [node for node in waits if node.get("settle_only") == "true"]
    assert len(settle) == 1, "exactly one settle wait, and it is not a new node type"
    settle = settle[0]

    # Not on the MPC arm: the oracle before the goal already holds the settle
    # check there. Not on a landing descent either -- WaitForLoadOff ends those
    # with the block on the ground -- and that half has to be the *outer* skip,
    # because a landing descent skips the oracle and leaves mpc_drives unset,
    # which a single `mpc_drives || ex_until_landed` script would still read.
    assert settle.get("_skipIf") == "mpc_drives"
    assert settle.get("sway_bound") == "{ex_sway_bound}"
    guard = next(node for node in root.iter("Sequence") if settle in list(node))
    assert guard.get("_skipIf") == "ex_until_landed"

    # After the goal, not before: before it, the plan has not run. And after the
    # deactivate (issue 234): held, the loaded tip mode does not decay at all --
    # the position hold rings at its frequency and feeds it -- while deactivated it
    # decays with tau = 21 s. A settle gate waited out under the hold cannot pass.
    parent = next(node for node in root.iter("Sequence") if guard in list(node))
    children = list(parent)
    goal = next(node for node in children if node.tag == "Parallel")
    off = next(
        node
        for node in children
        if node.get("deactivate_controllers") == "trajectory_controller_a2b"
    )
    assert children.index(guard) > children.index(goal)
    assert children.index(guard) > children.index(off)


def test_the_short_arrival_dwell_is_asked_for_only_where_the_hook_is_empty():
    """Issue 255: the 2.7 s dwell is one period of the *carried* pendulum.

    Both halves of the subtree read it from `ex_hold_s`, and the blackboard is
    shared across the whole run, so the number that bites is the one a loaded move
    inherits: a placement left on 0.3 s sets the block down on a swing. Every short
    value is therefore paired with its carrying guard, and the placement descent
    sets the full dwell itself rather than inheriting it -- SubTreePlaceFromHover
    enters without the move_above that would otherwise have set it.
    """
    root = ElementTree.parse(CBS_SUBTREE).getroot()
    waits = list(root.iter("WaitForMoveComplete"))
    assert [node.get("hold_s") for node in waits] == ["{ex_hold_s}"] * len(waits)

    for name, flag in (
        ("subtree_move_above.xml", "mv_carries"),
        ("subtree_lift.xml", "lf_carries"),
    ):
        holds = [
            node
            for node in ElementTree.parse(TREE_DIR / name)
            .getroot()
            .iter("SetBlackboard")
            if node.get("output_key") == "ex_hold_s"
        ]
        assert [node.get("value") for node in holds] == ["0.3", "2.7"], name
        assert holds[0].get("_skipIf") == f"{flag} == 'true'", name
        assert holds[1].get("_skipIf") == f"{flag} != 'true'", name

    place = ElementTree.parse(TREE_DIR / "subtree_place_block.xml").getroot()
    approach = next(
        tree
        for tree in place.iter("BehaviorTree")
        if tree.get("ID") == "SubTreePlaceApproach"
    )
    loaded = [
        node
        for node in approach.iter("SetBlackboard")
        if node.get("output_key") == "ex_hold_s"
    ]
    assert [node.get("value") for node in loaded] == ["2.7"]
    assert loaded[0].get("_skipIf") is None


def test_a_carrying_lift_asks_for_a_slower_ramp_and_gives_it_back():
    """Issue 234: the 5 s lift is an impulse on the loaded tip mode.

    1.5 m in 5 s is 1.7 periods of the 0.35 Hz carried mode and leaves 0.22 rad/s,
    which the settle gate's 30 s cannot absorb at the mode's 21 s decay. The
    override has to be carrying-only and it has to be restored: a leaked value
    slows every move after the lift, and the place descent reads `slow_down` too.

    Issue 254: and it has to *multiply* the run's pace rather than assign over it,
    or a run launched at slow_down 3 comes back to full speed on its slowest leg.
    """
    root = ElementTree.parse(TREE_DIR / "subtree_lift.xml").getroot()
    scripts = [node.get("code") for node in root.iter("Script")]

    assert "lf_sd_saved := slow_down" in scripts
    override = next(node for node in root.iter("Script") if "* 3.0" in node.get("code"))
    assert override.get("code") == "slow_down := lf_sd_saved * 3.0"
    assert override.get("_skipIf") == "lf_carries != 'true'"
    restore = next(
        node
        for node in root.iter("Script")
        if node.get("code") == "slow_down := lf_sd_saved"
    )
    assert restore.get("_skipIf") is None, "the restore must be unconditional"
    # SetBlackboard would create `slow_down` as a std::string, and the scripting
    # multiplication above cannot read one.
    assert not [
        node
        for node in root.iter("SetBlackboard")
        if node.get("output_key") in ("slow_down", "lf_sd_saved")
    ]


def test_the_run_pace_is_a_parameter_the_slow_legs_multiply():
    """254: fast by default, slower for a lab day, without editing a tree.

    The pace enters once, from the BT server's own `slow_down` parameter, and every
    leg that wants to be slower than the run scales it. An assignment anywhere here
    would pin that leg to one speed whatever the run was launched at.
    """
    for name in ("basic_pick_and_place.xml", "wall_assembly.xml"):
        root = ElementTree.parse(TREE_DIR / name).getroot()
        assert [node.get("slow_down") for node in root.iter("GetSlowDown")] == [
            "{slow_down}"
        ], name

    touchdown = next(
        tree
        for tree in ElementTree.parse(TREE_DIR / "subtree_place_block.xml")
        .getroot()
        .iter("BehaviorTree")
        if tree.get("ID") == "SubTreeTouchdown"
    )
    codes = [node.get("code") for node in touchdown.iter("Script")]
    assert "sd_saved := slow_down; slow_down := slow_down * td_scale" in codes
    assert "slow_down := sd_saved" in codes
    # Every caller sets the scale: the blackboard is shared, and the simple path's 8.0
    # would otherwise pace a 0.25 m stroke after it (issue 290).
    for name, tree_id in TOUCHDOWN_CALLERS:
        codes = _subtree_scripts(name, tree_id)
        assert [code for code in codes if "td_scale :=" in code], tree_id


def test_cbs_move_empty_uses_the_cbs_subtree():
    root = ElementTree.parse(CBS_MOVE_EMPTY).getroot()

    assert root.get("main_tree_to_execute") == "CbsMoveEmptyPzs100"

    includes = [element.get("path") for element in root.iter("include")]
    assert includes == ["behavior_trees/cbs_subtree_a2b_movement.xml"], (
        "the CBS tree must include its own subtree, not epsilon's"
    )

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
        {"name": "Run plan (all blocks)", "path": "/behavior_trees/wall_assembly.xml"},
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


def test_no_tree_id_has_whitespace():
    """BT.CPP 4.9 refuses to load a tree whose ID has a space ('forbidden character')."""
    for xml in TREE_DIR.glob("*.xml"):
        root = ElementTree.parse(xml).getroot()
        names = [root.get("main_tree_to_execute") or ""]
        names += [tree.get("ID") for tree in root.iter("BehaviorTree")]
        assert not any(" " in name for name in names), xml.name


def _regrip_scripts():
    root = ElementTree.parse(TREE_DIR / "subtree_pick_and_place_block.xml").getroot()
    regrip = [t for t in root.iter("BehaviorTree") if t.get("ID") == "SubTreeRegrip"]
    assert len(regrip) == 1
    return [node.get("code", "") for node in regrip[0].iter("Script")]


def test_regrip_height_comes_from_regrip_z_not_the_pickup():
    """
    The regrip spot is level ground; the pickup is not necessarily.

    Both halves took their z from `pickup_z`, invisible while every pickup was a
    block on the ground. Off the truck bed (seed z 1.62 world) that is 1.32 m of
    air: the set-down hits `WaitForLoadOff`'s floor under `place_z` and fails
    holding the load, and the re-pick descends onto nothing.
    """
    code = " ".join(_regrip_scripts())
    assert "place_z := regrip_z" in code
    assert "pickup_z := regrip_z" in code
    assert "place_z := pickup_z" not in code

    # A tree that enters the regrip without defining regrip_z reads it as 0.
    for name in ("wall_assembly.xml", "basic_pick_and_place.xml"):
        root = ElementTree.parse(TREE_DIR / name).getroot()
        code = " ".join(node.get("code", "") for node in root.iter("Script"))
        code += " ".join(
            node.get("output_key", "") for node in root.iter("SetBlackboard")
        )
        assert "regrip_z" in code, name


def test_the_landing_floor_measures_with_the_slots_own_offset():
    """Issue 290: on the pick's offset the floor sat 0.11-0.19 m deeper than place_z - 0.01.

    WaitForLoadOff puts the block centre |offset| under K8. The offset the pick measured
    misses the loaded bend at the slot (1.16-1.24 vs nominal 1.05 m, lab day 2026-11-04),
    so the gate reads place_g: the angled path's own measurement where the path takes one
    (the hover's, refreshed at the waypoint, issue 291), the pick's where it does not.
    """
    landing = next(
        node
        for node in ElementTree.parse(CBS_SUBTREE).getroot().iter()
        if node.get("ID") == "WaitForLoadOff"
        and node.get("_skipIf") == "!ex_until_landed"
    )
    assert landing.get("s_log_8_z") == "{place_g}"

    # The angled path's hover capture writes it; the other two have no hover measurement
    # and say so by assigning the pick's offset.
    captures = [
        node.get("s_log_8_z")
        for node in ElementTree.parse(TREE_DIR / "subtree_place_block.xml")
        .getroot()
        .iter()
        if node.get("ID") == "CaptureBlockGraspOffset"
    ]
    assert "{place_g}" in captures
    for name, tree_id in TOUCHDOWN_CALLERS:
        if (name, tree_id) == ("subtree_place_block.xml", "SubTreePlaceFinish"):
            continue
        codes = _subtree_scripts(name, tree_id)
        assert [code for code in codes if "place_g :=" in code], tree_id


def test_landing_reference_is_taken_before_the_descent_moves():
    """WaitForLoadOff's reference window must not overlap the start of the descent."""
    root = ElementTree.parse(CBS_SUBTREE).getroot()
    parallel = next(root.iter("Parallel"))
    move, gate, watch = list(parallel)
    first = list(move)[0]
    assert first.tag == "Sleep"
    # Both gates take the same 500 ms hanging reference, so both wait it out.
    assert first.get("_skipIf") == "!ex_until_landed && !ex_watch_landed"
    assert int(first.get("msec")) > 500  # WaitForLoadOff's reference window
    assert gate.get("ID") == "WaitForLoadOff"
    assert watch.get("ID") == "WaitForLoadOff"


def test_a_watched_descent_fails_on_any_load_off():
    """Issue 289: on a loaded descent to a waypoint every load-off is early ground contact.

    The watch may never SUCCEED -- success_count is 1, so a success would end the move as
    if it had arrived and the next leg would descend onto the ground -- so
    max_above_target sits below every height the block can be at and a confirmed drop
    becomes the failure that stops the descent. And it owns no deadline worth failing on:
    the arrival wait is the move's oracle, while the landing gate's 40 s would fail every
    descent slower than that.
    """
    root = ElementTree.parse(CBS_SUBTREE).getroot()
    gates = [node for node in root.iter() if node.get("ID") == "WaitForLoadOff"]
    assert len(gates) == 2
    landing = next(g for g in gates if g.get("_skipIf") == "!ex_until_landed")
    watch = next(g for g in gates if g.get("_skipIf") == "!ex_watch_landed")

    assert float(watch.get("max_above_target")) < 0.0
    assert int(watch.get("timeout_ms")) > int(landing.get("timeout_ms"))
    # And the kinematic floor is off on the watch. It is measured from place_z while the
    # watched descents target place_z + 0.05, so at the node's 1 cm default the lab day's
    # own 0.3-0.5 m overrun trips it first -- and it trips with load_off false, i.e. as a
    # plain descent failure, which RetryUntilSuccessful answers with a second descent into
    # the ground. The touchdown keeps the floor: it is the one aiming under place_z.
    assert float(watch.get("floor_below_target")) > 0.5
    # The landing gate keeps its floor, but deeper than the 0.20 m the touchdown commands
    # under place_z and deeper than the push the 0.8 drop needs: since issue 290 it measures
    # the block with the slot's own offset, and the node's 1 cm default then fires ~1 cm past
    # contact, failing a landing that is going fine.
    landing_floor = float(landing.get("floor_below_target"))
    assert 0.20 < landing_floor < float(watch.get("floor_below_target"))
    # The same drop share: early contact may end in a release, so issue 263's reason for
    # 0.8 -- jaws that opened on 375 kg whipped the gripper off the placed block -- holds.
    assert watch.get("drop_fraction") == landing.get("drop_fraction")
    for gate in gates:
        # How the tree tells early contact from any other failure of the descent.
        assert gate.get("load_off") == "{load_off}"
        assert gate.get("target_z") == "{place_z}"


def test_the_landed_watch_is_picked_wherever_the_landing_gate_is():
    """One shared blackboard, so the footgun is a stale `true`.

    An empty lift that inherited ex_watch_landed from the place before it would be watched
    by a gate reading 880 kg leaving the hook as ground contact, and a descent whose gate
    never ran would read the previous block's load_off. So every caller that picks the mode
    sets all three keys in the one script, and never both gates at once.
    """
    seen = 0
    for xml in TREE_DIR.glob("*.xml"):
        for script in ElementTree.parse(xml).getroot().iter("Script"):
            code = script.get("code") or ""
            if "ex_until_landed :=" not in code:
                continue
            seen += 1
            assert "ex_watch_landed :=" in code, (xml.name, code)
            assert "load_off := false" in code, (xml.name, code)
            assert not (
                "ex_until_landed := true" in code and "ex_watch_landed := true" in code
            ), (xml.name, code)
    assert seen >= 8, seen
