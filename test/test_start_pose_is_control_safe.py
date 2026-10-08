"""The Gazebo launches' default start pose must lie inside the control-safe box.

Issue 231 made `crane_planner` refuse a start outside `crane_model`'s
control-safe box, so a launch default outside it aborts the tree on its first
move -- and `CalcMovement` carries no message field, so the only thing the
operator sees is `success = False`. `initial_pose:=1` is exactly that case
(`initialization_horizontal.yaml` holds the arm at 1.5708 rad against the arm
row's 1.324864). Nothing else ties the launch default to the box.

`real_wall_assembly_pzs100.launch.py` is deliberately absent: it is a hardware
launch, human-only, and still defaults to 1.

Two things this does not check. `crane_planning`'s gate is the *intersection* of
these rows with the description's own URDF limits, so a description row that
became the binding one would refuse a start this test still passes (the
description is looser on all four axes today). And what the planner is handed is
the *measured* state, not this file: in Gazebo the boom and arm settle ~33 and
~31 mrad above their initialization values at pose 2 (measured over 28 s, then
steady to 6e-4 rad). Pose 2 keeps 0.77 rad of arm clearance under that offset,
which is why the yaml is a sound proxy here; pose 1's offset was +0.14 rad and
took `q0[2]` to 1.71.
"""

import re
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument

GAZEBO_LAUNCHES = (
    "gazebo_basic_pick_and_place_pzs100.launch.py",
    "gazebo_wall_assembly_pzs100.launch.py",
)

# crane_model's axis names -> the legacy URDF coordinate the initialization
# yamls key by, for the rows a planner actually refuses a start on.
#
# Four of the box's six rows, deliberately. q5/q6 (tip, tilt) are passive and
# the box names no row for them at all. The other two it does name are not
# gated, so asserting them would invent a refusal nothing performs:
#   rotator  theta8_rotator_joint is a URDF `continuous` joint, so
#            crane_planning's `read_limits` takes no intersection on it and
#            `relaxed_to`'s mask is `& self.bounded`.
#   tool     control_safe_limits.yaml's own note calls both tool bounds
#            unresolved -- the upper is above the rail's 0.538 m travel -- and
#            nothing boxes the tool coordinate: `position_box` slices
#            [:K_PLANNED_DOF] = 5, which stops at the rotator, and q8 enters as
#            a `pin_tool` parameter. q9_0 = 0.21 clears the lower bound by 10 mm,
#            so correcting that row (the yaml's own open item) would fail this
#            test and read as "move the start pose".
AXIS_TO_INITIALIZATION_KEY = {
    "slewing": "theta1_0",
    "boom": "theta2_0",
    "arm": "theta3_0",
    "telescope": "q4_0",
}


def _launch_defaults(name):
    path = Path(__file__).resolve().parents[1] / "launch" / name
    spec = spec_from_file_location(name.removesuffix(".py"), path)
    module = module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    context = LaunchContext()
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    return context.launch_configurations


def _initialization_values(pose_id):
    """The initialization yaml for a pose id, via the xacro that owns the mapping."""
    share = Path(get_package_share_directory("epsilon_crane_description"))
    xacro = (share / "urdf" / "inc" / "initialization.xacro").read_text(
        encoding="utf-8"
    )
    match = re.search(
        rf'initial_pose=={re.escape(str(pose_id))}\}}">\s*'
        r'<xacro:property name="initialization_filename"\s+value="([^"]+)"',
        xacro,
    )
    assert match, f"initialization.xacro maps no file to initial_pose:={pose_id}"
    return yaml.safe_load(
        (share / "config" / match.group(1)).read_text(encoding="utf-8")
    )


@pytest.mark.parametrize("launch_file", GAZEBO_LAUNCHES)
def test_default_initial_pose_is_inside_the_control_safe_box(launch_file):
    pose_id = _launch_defaults(launch_file)["initial_pose"]
    start = _initialization_values(pose_id)
    box = yaml.safe_load(
        (
            Path(get_package_share_directory("crane_model"))
            / "config"
            / "control_safe_limits.yaml"
        ).read_text(encoding="utf-8")
    )

    outside = {
        axis: (
            start[key],
            box["q_a_lower"][axis]["value"],
            box["q_a_upper"][axis]["value"],
        )
        for axis, key in AXIS_TO_INITIALIZATION_KEY.items()
        if not box["q_a_lower"][axis]["value"]
        <= start[key]
        <= box["q_a_upper"][axis]["value"]
    }
    assert not outside, (
        f"{launch_file} defaults to initial_pose:={pose_id}, which is outside "
        f"crane_model's control-safe box as {{axis: (start, lower, upper)}} {outside}. "
        "crane_planner refuses the first move from there (issue 231) and the tree "
        "reports a bare failure."
    )
