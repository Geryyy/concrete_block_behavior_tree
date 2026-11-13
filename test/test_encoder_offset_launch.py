"""Issue 318: the wall profile's FK bias reaches the sim description.

The xacro that reads `ENCODER_OFFSET` is expanded two includes down, by a launch
file that passes a fixed argument list and is in another repository, so the
environment variable is the whole path from this profile's launch argument to
the plugin. An all-zero default is what keeps every run that does not ask for a
bias unchanged, and that is the half worth guarding.
"""

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable

PROFILE = "gazebo_wall_assembly_pzs100.launch.py"


def _environment(**configurations):
    """The profile's environment after its declarations and SetEnvironmentVariable."""
    path = Path(__file__).resolve().parents[1] / "launch" / PROFILE
    spec = spec_from_file_location(PROFILE.removesuffix(".py"), path)
    module = module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)

    context = LaunchContext()
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
            if entity.name in configurations:
                context.launch_configurations[entity.name] = configurations[entity.name]
        elif isinstance(entity, SetEnvironmentVariable):
            entity.visit(context)
    return context.environment


def test_the_default_biases_no_axis():
    assert _environment()["ENCODER_OFFSET"].split() == ["0"] * 5


def test_the_launch_argument_reaches_the_xacro():
    bias = "0.00473 0.00284 0.00426 0.00710 0"
    assert _environment(encoder_offset=bias)["ENCODER_OFFSET"] == bias
