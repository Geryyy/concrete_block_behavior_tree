from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node


def _load_launch_module(name):
    launch_path = Path(__file__).resolve().parents[1] / "launch" / name
    spec = spec_from_file_location(name.removesuffix(".py"), launch_path)
    module = module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _walk_entities(entities):
    for entity in entities:
        yield entity
        sub_entities = getattr(entity, "entities", None)
        if sub_entities:
            yield from _walk_entities(sub_entities)


def test_basic_pick_and_place_launch_uses_only_cbs_grip_server():
    module = _load_launch_module("gazebo_basic_pick_and_place_pzs100.launch.py")
    ld = module.generate_launch_description()

    nodes = [entity for entity in _walk_entities(ld.entities) if isinstance(entity, Node)]
    include_actions = [
        entity for entity in _walk_entities(ld.entities) if isinstance(entity, IncludeLaunchDescription)
    ]

    assert any(
        node.node_package == "concrete_block_motion_planning" and
        node.node_executable == "grip_traj_server_simple.py"
        for node in nodes
    ), "CBS simple grip trajectory server is not launched"

    assert any(
        node.node_package == "concrete_block_assembly_planning" and
        node.node_executable == "wall_plan_server"
        for node in nodes
    ), "Commissioning stack launch must start the wall-plan server"

    assert not any(
        node.node_package == "timber_crane_motion_planning" and
        node.node_executable == "grip_traj_server"
        for node in nodes
    ), "Timber grip trajectory server must not be launched directly"

    pzs100_include = next(
        (
            action for action in include_actions
            if "pzs100_bringup.launch.py" in str(action.launch_description_source.location)
        ),
        None,
    )
    assert pzs100_include is not None, "Expected PZS100 bringup launch include"

    launch_arguments = dict(pzs100_include.launch_arguments)
    assert "start_bt_action_server" in launch_arguments
    assert str(launch_arguments["start_bt_action_server"]) == "False"
    assert "start_grip_traj_server" in launch_arguments
    assert str(launch_arguments["start_grip_traj_server"]) == "False"


def _node_parameter_names(node):
    names = []
    for param in getattr(node, "_Node__parameters"):
        if not isinstance(param, dict):
            continue
        for key in param:
            if isinstance(key, tuple):
                names.extend(getattr(part, "text", None) for part in key)
            else:
                names.append(str(key))
    return {name for name in names if name}


def test_wall_assembly_launch_exposes_grip_lift_height_override():
    module = _load_launch_module("gazebo_wall_assembly_pzs100.launch.py")
    ld = module.generate_launch_description()

    declares = [
        entity for entity in _walk_entities(ld.entities)
        if isinstance(entity, DeclareLaunchArgument)
    ]
    assert any(
        declare.name == "lift_height"
        and getattr(declare.default_value[0], "text", None) == "1.0"
        for declare in declares
    )

    grip_node = next(
        node for node in _walk_entities(ld.entities)
        if isinstance(node, Node)
        and node.node_package == "concrete_block_motion_planning"
        and node.node_executable == "grip_traj_server_simple.py"
    )
    assert "lift_height" in _node_parameter_names(grip_node)


def test_row3_truck_launch_uses_larger_grip_lift_height():
    module = _load_launch_module("gazebo_row3_middle_last_pzs100.launch.py")
    ld = module.generate_launch_description()

    wall_include = next(
        action for action in _walk_entities(ld.entities)
        if isinstance(action, IncludeLaunchDescription)
        and "gazebo_wall_assembly_pzs100.launch.py" in str(
            action.launch_description_source.location
        )
    )
    launch_arguments = dict(wall_include.launch_arguments)
    assert launch_arguments["lift_height"] == "1.2"
