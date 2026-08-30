from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

import yaml
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
        # TimerAction keeps its children in a private attribute, not `entities`.
        timed = getattr(entity, "_TimerAction__actions", None)
        if timed:
            yield from _walk_entities(timed)


def test_basic_pick_and_place_launch_uses_only_cbs_grip_server():
    module = _load_launch_module("gazebo_basic_pick_and_place_pzs100.launch.py")
    ld = module.generate_launch_description()

    nodes = [
        entity for entity in _walk_entities(ld.entities) if isinstance(entity, Node)
    ]
    include_actions = [
        entity
        for entity in _walk_entities(ld.entities)
        if isinstance(entity, IncludeLaunchDescription)
    ]

    assert any(
        node.node_package == "concrete_block_motion_planning"
        and node.node_executable == "grip_traj_server_simple.py"
        for node in nodes
    ), "CBS simple grip trajectory server is not launched"

    assert any(
        node.node_package == "concrete_block_assembly_planning"
        and node.node_executable == "wall_plan_server"
        for node in nodes
    ), "Commissioning stack launch must start the wall-plan server"

    # The grasp detector is started by the pzs100_bringup include asserted below,
    # not by this file.
    assert not any(
        node.node_package == "timber_crane_motion_planning"
        and node.node_executable == "grip_traj_server"
        for node in nodes
    ), "Timber grip trajectory server must not be launched directly"

    pzs100_include = next(
        (
            action
            for action in include_actions
            if "pzs100_bringup.launch.py"
            in str(action.launch_description_source.location)
        ),
        None,
    )
    assert pzs100_include is not None, "Expected PZS100 bringup launch include"

    launch_arguments = dict(pzs100_include.launch_arguments)
    assert "start_bt_action_server" in launch_arguments
    assert str(launch_arguments["start_bt_action_server"]) == "False"
    assert "start_grip_traj_server" in launch_arguments
    assert str(launch_arguments["start_grip_traj_server"]) == "False"


def test_basic_pick_and_place_launch_loads_single_block_tree():
    module = _load_launch_module("gazebo_basic_pick_and_place_pzs100.launch.py")
    ld = module.generate_launch_description()

    bt_node = next(
        node
        for node in _walk_entities(ld.entities)
        if isinstance(node, Node)
        and node.node_package == "lsrl_behavior_tree"
        and node.node_executable == "bt_action_server"
    )
    for param in getattr(bt_node, "_Node__parameters"):
        if not isinstance(param, dict):
            continue
        for key, value in param.items():
            key_text = "".join(getattr(part, "text", str(part)) for part in key)
            if key_text == "behaviortree":
                assert "basic_pick_and_place.xml" in str(value)
                return
    assert False, "bt_action_server does not set a behaviortree parameter"


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
        entity
        for entity in _walk_entities(ld.entities)
        if isinstance(entity, DeclareLaunchArgument)
    ]
    assert any(
        declare.name == "lift_height"
        and getattr(declare.default_value[0], "text", None) == "1.0"
        for declare in declares
    )

    grip_node = next(
        node
        for node in _walk_entities(ld.entities)
        if isinstance(node, Node)
        and node.node_package == "concrete_block_motion_planning"
        and node.node_executable == "grip_traj_server_simple.py"
    )
    assert "lift_height" in _node_parameter_names(grip_node)

    # The grasp detector is started by the included pzs100_bringup.launch.py.
    assert any(
        "pzs100_bringup.launch.py" in str(action.launch_description_source.location)
        for action in _walk_entities(ld.entities)
        if isinstance(action, IncludeLaunchDescription)
    ), "Wall assembly launch must include the PZS100 bringup that owns the detector"


def test_cbs_wall_assembly_launch_runs_only_the_new_stack():
    module = _load_launch_module("cbs_wall_assembly_pzs100.launch.py")
    ld = module.generate_launch_description()

    nodes = [
        entity for entity in _walk_entities(ld.entities) if isinstance(entity, Node)
    ]
    include_actions = [
        entity
        for entity in _walk_entities(ld.entities)
        if isinstance(entity, IncludeLaunchDescription)
    ]

    assert len(include_actions) == 1, (
        "CBS profile must include only the new sim profile"
    )
    sim_include = include_actions[0]
    assert "crane_bringup" in str(sim_include.launch_description_source.location)
    assert "sim.launch.py" in str(sim_include.launch_description_source.location)

    forbidden = {
        "timber_crane_motion_planning",
        "timber_crane_motion_planning_cpp",
        "collision_body_handler",
        "timber_crane_mapping",
    }
    assert not any(node.node_package in forbidden for node in nodes), (
        "CBS profile must not start any timber motion-planning node"
    )

    rviz_node = next(
        node
        for node in nodes
        if node.node_package == "rviz2" and node.node_executable == "rviz2"
    )
    remaps = [
        (
            "".join(getattr(part, "text", str(part)) for part in src),
            "".join(getattr(part, "text", str(part)) for part in dst),
        )
        for src, dst in getattr(rviz_node, "_Node__remappings")
    ]
    assert ("/robot_description", "/robot_description_full") not in remaps, (
        "RViz must read /robot_description, the model-side description whose "
        "joint origins are zeroed. /robot_description_full is the Gazebo twin, "
        "with the initial_pose preset baked into every origin; /joint_states "
        "carries that preset too, so the remap draws every joint twice-bent"
    )

    bridge = next(
        node
        for node in nodes
        if node.node_package == "crane_bringup" and node.node_executable == "sim_remote"
    )
    bridge_remaps = [
        (
            "".join(getattr(part, "text", str(part)) for part in src),
            "".join(getattr(part, "text", str(part)) for part in dst),
        )
        for src, dst in getattr(bridge, "_Node__remappings")
    ]
    assert (
        "/crane/remote_ctrl_states",
        "/gpio_controller/remote_ctrl_states",
    ) in bridge_remaps, "Deadman bridge must feed the topic the BT approval nodes read"

    declares = {
        entity.name: entity
        for entity in _walk_entities(ld.entities)
        if isinstance(entity, DeclareLaunchArgument)
    }
    assert "start_tui" in declares
    assert getattr(declares["start_tui"].default_value[0], "text", None) == "false"

    spawner = next(
        node for node in nodes if node.node_executable == "gazebo_block_spawner.py"
    )
    seed_frame = None
    for param in getattr(spawner, "_Node__parameters"):
        if not isinstance(param, dict):
            continue
        for key, value in param.items():
            key_text = "".join(getattr(part, "text", str(part)) for part in key)
            if key_text == "gazebo_seed_frame_xyz":
                seed_frame = [float(getattr(item, "text", item)) for item in value]
    assert seed_frame == [0.0, -6.0, 0.0], "World offset must carry over unchanged"


def test_row3_truck_launch_uses_larger_grip_lift_height():
    module = _load_launch_module("gazebo_row3_middle_last_pzs100.launch.py")
    ld = module.generate_launch_description()

    wall_include = next(
        action
        for action in _walk_entities(ld.entities)
        if isinstance(action, IncludeLaunchDescription)
        and "gazebo_wall_assembly_pzs100.launch.py"
        in str(action.launch_description_source.location)
    )
    launch_arguments = dict(wall_include.launch_arguments)
    assert launch_arguments["lift_height"] == "1.2"


def test_real_wall_assembly_launch_includes_world_model_without_gazebo():
    module = _load_launch_module("real_wall_assembly_pzs100.launch.py")
    ld = module.generate_launch_description()

    nodes = [
        entity for entity in _walk_entities(ld.entities) if isinstance(entity, Node)
    ]
    include_actions = [
        entity
        for entity in _walk_entities(ld.entities)
        if isinstance(entity, IncludeLaunchDescription)
    ]

    assert not any(
        "gazebo" in str(action.launch_description_source.location)
        for action in include_actions
    ), "Real launch must not include Gazebo launch files"

    perception_include = next(
        (
            action
            for action in include_actions
            if "concrete_block_detector"
            in str(action.launch_description_source.location)
            and "wall_assembly_perception.launch.py"
            in str(action.launch_description_source.location)
        ),
        None,
    )
    assert perception_include is not None, (
        "Expected concrete-block detector pipeline include"
    )

    launch_arguments = dict(perception_include.launch_arguments)
    assert "start_world_model" in launch_arguments
    assert (
        launch_arguments["start_world_model"].describe()
        == "LaunchConfig('start_world_model')"
    )
    assert "points_topic" in launch_arguments

    assert any(
        node.node_package == "rviz2" and node.node_executable == "rviz2"
        for node in nodes
    ), "Real launch should start RViz"

    assert any(
        node.node_package == "lsrl_behavior_tree"
        and node.node_executable == "bt_action_server"
        for node in nodes
    ), "Real launch should start the BT action server"

    assert any(
        node.node_package == "concrete_block_behavior_tree"
        and node.node_executable == "gripper_grasp_detector"
        for node in nodes
    ), "Real launch should start the grasp detector"


# ── Every plugin cbs.rviz names has to exist on the CBS profile ──────────────
#
# `cbs.rviz` is shared: `pzs100_bringup.launch.py` hands it to the timber-backed
# twin, and `cbs_wall_assembly_pzs100.launch.py` to the CBS-only profile. The
# timber stack has `wood_log_rviz_plugins`, `timber_crane_rviz_panel` and
# `mp_rviz_panel` on the RViz plugin path; the CBS profile does not, so a class
# from one of those loads there and fails here. A failed *Tool* is the one that
# bites: RViz builds its toolbar from this list, and the 2D Goal Pose tool
# stopped being usable when `wood_log_rviz_plugins/LogPicker` failed beside it.
RVIZ_CONFIG = Path(__file__).resolve().parents[1] / "rviz" / "cbs.rviz"
# Shipped with RViz itself, so available wherever rviz2 runs.
RVIZ_BUILTIN_NAMESPACES = ("rviz_common", "rviz_default_plugins")


def _rviz_plugin_classes():
    """Return every `Class` cbs.rviz names, displays nested in groups included."""
    document = yaml.safe_load(RVIZ_CONFIG.read_text(encoding="utf-8"))

    def walk(node):
        if isinstance(node, dict):
            if isinstance(node.get("Class"), str):
                yield node["Class"]
            for value in node.values():
                yield from walk(value)
        elif isinstance(node, list):
            for item in node:
                yield from walk(item)

    return set(walk(document))


def test_cbs_rviz_names_no_plugin_this_workspace_cannot_provide():
    """A class whose package does not resolve is a plugin RViz will fail to load."""
    from ament_index_python.packages import PackageNotFoundError, get_package_prefix

    offenders = {}
    for name in _rviz_plugin_classes():
        package = name.split("/")[0]
        if not package or package in RVIZ_BUILTIN_NAMESPACES:
            continue
        try:
            get_package_prefix(package)
        except PackageNotFoundError:
            offenders[name] = package
    assert not offenders, (
        f"cbs.rviz names {offenders}, which this workspace does not provide, so "
        "RViz logs a PluginlibFactory error and carries on without them. A "
        "failed Tool takes the toolbar with it -- that is how the 2D Goal Pose "
        "tool stopped working on the CBS profile."
    )
