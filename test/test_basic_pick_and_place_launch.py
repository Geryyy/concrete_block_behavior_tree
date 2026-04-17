from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node


def _load_launch_module():
    launch_path = Path(__file__).resolve().parents[1] / "launch" / "gazebo_basic_pick_and_place_pzs100.launch.py"
    spec = spec_from_file_location("gazebo_basic_pick_and_place_pzs100", launch_path)
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
    module = _load_launch_module()
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

    assert not any(
        node.node_package == "timber_crane_motion_planning" and
        node.node_executable == "grip_traj_server"
        for node in nodes
    ), "Timber grip trajectory server must not be launched directly"

    gazebo_include = next(
        (
            action for action in include_actions
            if "gazebo_model_bt.launch.py" in str(action.launch_description_source.location)
        ),
        None,
    )
    assert gazebo_include is not None, "Expected base Gazebo BT launch include"

    launch_arguments = dict(gazebo_include.launch_arguments)
    assert "start_grip_traj_server" in launch_arguments
    assert str(launch_arguments["start_grip_traj_server"]) == "False"
