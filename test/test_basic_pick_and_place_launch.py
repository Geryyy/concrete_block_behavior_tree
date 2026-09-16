from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

import yaml
from launch import LaunchContext
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


def _started_packages(ld, **configurations):
    """Node packages the profile starts with these launch configurations set."""
    context = LaunchContext()
    # Seed every declared default first: the conditions of the other optional
    # nodes (`gui`, `start_perception`, ...) read configurations too, and an
    # unset one is a SubstitutionFailure rather than a false.
    for entity in _walk_entities(ld.entities):
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    context.launch_configurations.update(configurations)
    return {
        entity.node_package
        for entity in _walk_entities(ld.entities)
        if isinstance(entity, Node)
        and (entity.condition is None or entity.condition.evaluate(context))
    }


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


def test_pzs100_bringup_never_suppresses_the_a2b_controller_spawner():
    """`mpc` gates the only trajectory_controller_a2b spawner -- it must stay False.

    The controller switch is the selected yaml, not the flag: passing mpc:=True
    left `controller:=mpc` with no A2B server behind /follow_joint_trajectory.
    """
    from launch.utilities import (
        normalize_to_list_of_substitutions,
        perform_substitutions,
    )

    module = _load_launch_module("pzs100_bringup.launch.py")
    ld = module.generate_launch_description()

    include = next(
        action
        for action in _walk_entities(ld.entities)
        if isinstance(action, IncludeLaunchDescription)
        and "gazebo_model_bt.launch.py"
        in str(action.launch_description_source.location)
    )
    launch_arguments = dict(include.launch_arguments)

    for controller, expected_yaml in (("pid", "jtc_pid"), ("mpc", "jtc_mpc")):
        context = LaunchContext()
        for entity in _walk_entities(ld.entities):
            if isinstance(entity, DeclareLaunchArgument):
                entity.execute(context)
        context.launch_configurations["controller"] = controller

        def resolve(name):
            return perform_substitutions(
                context, normalize_to_list_of_substitutions(launch_arguments[name])
            )

        assert resolve("mpc") == "False"
        assert expected_yaml in resolve("controller_a2b_config")


def _includes_launch_file(ld, name, **configurations):
    """Does the profile include this launch file with these configurations set?

    `location` on an unresolved source is its `describe()` text, not a path, so
    match the file name inside it rather than taking a basename.
    """
    context = LaunchContext()
    for entity in _walk_entities(ld.entities):
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    context.launch_configurations.update(configurations)
    return any(
        name in str(entity.launch_description_source.location)
        and (entity.condition is None or entity.condition.evaluate(context))
        for entity in _walk_entities(ld.entities)
        if isinstance(entity, IncludeLaunchDescription)
    )


def test_cbs_mpc_controller_refuses_a_planner_that_publishes_no_reference():
    """The launch-time error path no sim run reaches by accident.

    `crane_mpc` tracks `/crane/reference`, which only `planner:=cbs` publishes.
    Any other planner leaves the node started, silent and looking mistuned.
    """
    import pytest

    module = _load_launch_module("pzs100_bringup.launch.py")

    context = LaunchContext()
    context.launch_configurations.update(controller="cbs_mpc", planner="ilqr")
    with pytest.raises(RuntimeError) as excinfo:
        module.reject_cbs_mpc_without_cbs_planner(context)
    assert "cbs_mpc" in str(excinfo.value)
    assert "planner:=cbs" in str(excinfo.value)

    context.launch_configurations.update(controller="cbs_mpc_active", planner="vpsto")
    with pytest.raises(RuntimeError) as excinfo:
        module.reject_cbs_mpc_without_cbs_planner(context)
    assert "cbs_mpc_active" in str(excinfo.value)

    for controller, planner in (
        ("cbs_mpc", "cbs"),
        ("cbs_mpc_active", "cbs"),
        ("pid", "ilqr"),
        ("mpc", "ilqr"),
        ("pid", "cbs"),
    ):
        context.launch_configurations.update(controller=controller, planner=planner)
        module.reject_cbs_mpc_without_cbs_planner(context)


def test_only_cbs_mpc_starts_the_standalone_mpc_and_it_keeps_the_pid_jtc():
    """`cbs_mpc*` are the third and fourth values; `pid`/`mpc` keep their sets.

    `cbs_mpc_active` composes exactly as `cbs_mpc` -- same PID JTC, same
    standalone node -- and differs in the node's `mode` alone, which is the one
    thing that decides whether the horizon reaches the JTC (issue 146).
    """
    from launch.utilities import (
        normalize_to_list_of_substitutions,
        perform_substitutions,
    )

    module = _load_launch_module("pzs100_bringup.launch.py")
    ld = module.generate_launch_description()

    gazebo_include = next(
        action
        for action in _walk_entities(ld.entities)
        if isinstance(action, IncludeLaunchDescription)
        and "gazebo_model_bt.launch.py"
        in str(action.launch_description_source.location)
    )
    launch_arguments = dict(gazebo_include.launch_arguments)
    mpc_arguments = dict(
        next(
            action
            for action in _walk_entities(ld.entities)
            if isinstance(action, IncludeLaunchDescription)
            and "crane_mpc.launch.py" in str(action.launch_description_source.location)
        ).launch_arguments
    )

    for controller, planner, starts_mpc, expected_yaml, expected_mode in (
        ("cbs_mpc", "cbs", True, "jtc_pid", "shadow"),
        ("cbs_mpc_active", "cbs", True, "jtc_pid", "active"),
        ("pid", "ilqr", False, "jtc_pid", None),
        ("mpc", "ilqr", False, "jtc_mpc", None),
    ):
        assert (
            _includes_launch_file(
                ld, "crane_mpc.launch.py", controller=controller, planner=planner
            )
            is starts_mpc
        )

        context = LaunchContext()
        for entity in _walk_entities(ld.entities):
            if isinstance(entity, DeclareLaunchArgument):
                entity.execute(context)
        context.launch_configurations["controller"] = controller

        def resolve(value):
            return perform_substitutions(
                context, normalize_to_list_of_substitutions(value)
            )

        # The JtcMpcPlugin config is 'mpc' alone: crane_mpc must never run
        # beside a second MPC inside the JTC.
        assert expected_yaml in resolve(launch_arguments["controller_a2b_config"])
        if starts_mpc:
            assert resolve(mpc_arguments["mode"]) == expected_mode


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


def _started_executables(ld, **configurations):
    """Node executables the profile starts with these launch configurations set."""
    context = LaunchContext()
    for entity in _walk_entities(ld.entities):
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    context.launch_configurations.update(configurations)
    return {
        entity.node_executable
        for entity in _walk_entities(ld.entities)
        if isinstance(entity, Node)
        and (entity.condition is None or entity.condition.evaluate(context))
    }


def test_only_cbs_mpc_active_takes_the_action_goal_out_of_the_tree():
    """Issue 148: `a2b_mpc_drives` is the one bit that moves the commander.

    True and the CBS A2B subtree skips its FollowJointTrajectory goal, leaving
    crane_mpc alone on the JTC; false and the tree commands as it always has.
    Anything but `cbs_mpc_active` must read false, or two commanders go live.
    """
    from launch.utilities import (
        normalize_to_list_of_substitutions,
        perform_substitutions,
    )

    module = _load_launch_module("gazebo_wall_assembly_pzs100.launch.py")
    ld = module.generate_launch_description()

    bt_node = next(
        node
        for node in _walk_entities(ld.entities)
        if isinstance(node, Node)
        and node.node_package == "lsrl_behavior_tree"
        and node.node_executable == "bt_action_server"
    )
    parameter = next(
        value
        for param in getattr(bt_node, "_Node__parameters")
        if isinstance(param, dict)
        for key, value in param.items()
        if "".join(getattr(part, "text", str(part)) for part in key) == "a2b_mpc_drives"
    )

    for controller, expected in (
        ("cbs_mpc_active", "True"),
        ("cbs_mpc", "False"),
        ("mpc", "False"),
        ("pid", "False"),
    ):
        context = LaunchContext()
        for entity in _walk_entities(ld.entities):
            if isinstance(entity, DeclareLaunchArgument):
                entity.execute(context)
        context.launch_configurations["controller"] = controller
        assert (
            perform_substitutions(
                context, normalize_to_list_of_substitutions(parameter.value)
            )
            == expected
        )


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
# `cbs.rviz` is handed to the timber-backed twin by `pzs100_bringup.launch.py`.
# The timber stack has `wood_log_rviz_plugins`, `timber_crane_rviz_panel` and
# `mp_rviz_panel` on the RViz plugin path, so a class from one of those can load
# there and fail elsewhere. A failed *Tool* is the one that
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
