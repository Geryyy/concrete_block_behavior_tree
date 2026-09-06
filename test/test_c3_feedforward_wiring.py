"""Static checks on the C3 feedforward wiring.

Whether the feedforward improves tracking is a sim/graph-level gate. What can be pinned
offline is the wiring, and every failure mode here is silent at runtime:

- a tree that follows a trajectory without ticking AddC3Feedforward first just runs the
  static feedforward, which is what it did before -- no error anywhere;
- a node that is registered but missing from plugin_lib_names is an unknown-node parse
  failure of the whole tree, not of the feature;
- the JTC gains are a velocity PI written through control_toolbox's position-PID names
  (p == K_I, d == K_P, i == 0), so an `i` that is not zero integrates the position error a
  second time and nothing says so.
"""

from pathlib import Path
from xml.etree import ElementTree

import yaml

PACKAGE_DIR = Path(__file__).resolve().parents[1]
TREE_DIR = PACKAGE_DIR / "behavior_trees"
BT_SERVER_CONFIG = PACKAGE_DIR / "config" / "bt_server_override.yaml"
PID_CONFIG = (
    PACKAGE_DIR
    / "config"
    / "ros2_control"
    / "crane_controller_hydraulic_a2b_jtc_pid_pzs100.ros2_control.yaml"
)

# the five axes wiki/controller_design.md 4.1 ships a gain pair for; q9 is the tool axis and
# is deliberately untuned
TUNED_JOINTS = (
    "theta1_slewing_joint",
    "theta2_boom_joint",
    "theta3_arm_joint",
    "q4_big_telescope",
    "theta8_rotator_joint",
)


def _ordered_nodes(tree_file):
    """(tag, ID) of every element, in document order."""
    root = ElementTree.parse(tree_file).getroot()
    return [(element.tag, element.get("ID")) for element in root.iter()]


def _index_of(nodes, wanted):
    for position, node in enumerate(nodes):
        if wanted in node:
            return position
    return None


def _bt_server_params():
    with BT_SERVER_CONFIG.open() as handle:
        return yaml.safe_load(handle)["/**"]["ros__parameters"]


def test_every_trajectory_follower_is_preceded_by_the_feedforward():
    # the payload-carrying callers delegate the following to SubTreeExecuteTrajectory, so the
    # feedforward has to be ticked before that SubTree element
    for tree_file in sorted(TREE_DIR.glob("*.xml")):
        nodes = _ordered_nodes(tree_file)
        follower = _index_of(nodes, "SubTreeExecuteTrajectory")
        if follower is None or ("BehaviorTree", "SubTreeExecuteTrajectory") in nodes:
            continue
        feedforward = _index_of(nodes, "AddC3Feedforward")
        assert feedforward is not None, (
            f"{tree_file.name} follows a trajectory with no C3 feedforward"
        )
        assert feedforward < follower, (
            f"{tree_file.name} ticks the feedforward too late"
        )


def test_the_cbs_subtree_feeds_forward_before_it_follows():
    nodes = _ordered_nodes(TREE_DIR / "cbs_subtree_a2b_movement.xml")
    feedforward = _index_of(nodes, "AddC3Feedforward")
    follower = _index_of(nodes, "FollowJointTrajectory")
    assert feedforward is not None and follower is not None
    assert feedforward < follower


def test_the_feedforward_plugin_is_loaded_by_the_bt_server():
    assert "BT_cb_add_c3_feedforward_action" in _bt_server_params()["plugin_lib_names"]


def test_the_feedforward_parameters_are_consistent():
    params = _bt_server_params()["c3_feedforward"]

    assert isinstance(params["enabled"], bool)
    # a time, not a sample count: the trajectory grid differs per planner, the dead time does not
    assert params["dead_time_s"] == 0.06, "the pinned common dead time is 0.06 s"
    assert len(params["joints"]) == len(params["k"]), (
        "joints and k are read as parallel lists; a length mismatch turns the feedforward off"
    )
    assert all(stiffness > 0.0 for stiffness in params["k"]), "u carries tau_dot / k"
    assert set(params["joints"]) == set(TUNED_JOINTS)

    # the feedforward branch is the one nothing downstream bounds
    assert len(params["u_min"]) == len(params["joints"])
    assert len(params["u_max"]) == len(params["joints"])
    assert all(
        lower < 0.0 < upper for lower, upper in zip(params["u_min"], params["u_max"])
    )


def test_the_pid_gains_are_a_velocity_pi():
    with PID_CONFIG.open() as handle:
        params = yaml.safe_load(handle)["trajectory_controller_a2b"]["ros__parameters"]

    assert params["effort_field_is_feedforward"] is True, (
        "without this the JTC rejects every trajectory the feedforward node touches"
    )

    for joint in TUNED_JOINTS:
        gains = params["gains"][joint]
        # p == K_I and d == K_P: both terms of the velocity PI have to be present
        assert gains["p"] > 0.0, f"{joint} has no integral gain"
        assert gains["d"] > 0.0, f"{joint} has no proportional gain"
        assert gains["i"] == 0.0, f"{joint} integrates the position error twice"
        assert gains["ff_velocity_scale"] == 1.0, (
            f"{joint} would lose the reference velocity: the effort field carries only the "
            f"correction tau_dot/k, not the whole command"
        )
        assert gains["u_clamp_min"] < 0.0 < gains["u_clamp_max"], (
            f"{joint} needs Psi's identified domain as its bound"
        )
