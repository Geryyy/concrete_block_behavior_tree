import importlib.util
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "wall_report.py"
spec = importlib.util.spec_from_file_location("wall_report", SCRIPT)
wall_report = importlib.util.module_from_spec(spec)
spec.loader.exec_module(wall_report)

BT = "[bt_action_server-34] [INFO] [0.0] [bt_action_server_execute_bt_rclcpp_node]:"


def _task(t, n, block, x):
    return (
        f"{t} {BT} GetNextAssemblyTask response | success=true has_task=true "
        f"task_id=wall_0{n} target={block} reference= pickup=(1.00,1.00,1.00) "
        f"place=(8.00,3.00,-0.85) place_world=({x:.3f},-3.000,0.300,0.0000) message=Task {n}/2"
    )


def test_a_goal_reports_true_deviations_along_and_across_the_wall(tmp_path):
    """Two blocks on one course: the second lands 3 cm short along and 4 cm off across."""
    run = tmp_path / "run"
    run.mkdir()
    (run / "launch.log").write_text(
        "\n".join(
            [
                "1.0 [INFO] [world_model_node-26]: process started with pid [77]",
                _task(9.0, 1, "block_9", -15.0).replace(
                    "success=true", "success=false"
                ),
                _task(10.0, 1, "block_0", -15.0),
                f"11.0 {BT} PlacePass: attempt 1 pass 1 within | miss (+0.020, -0.010, +0.040) m, "
                "off (+0.000, +0.000) m",
                f"12.0 {BT} WaitForLoadOff: landed | effort 1 -> 0 against 1 (m g r, r 9.00 m), "
                "centre +0.020 m from target",
                f"12.5 {BT} WriteBlockPoseFromGripper: block_id='block_0' "
                "world_pose=(-15.000, -3.000, 0.300) task_status=3 pose_status=2",
                _task(20.0, 2, "block_1", -14.08),
                "20.5 [bt_action_server-34] [1.0]: place_approach            RUNNING -> FAILURE",
                f"22.0 {BT} WaitForLoadOff: landed | effort 1 -> 0 against 1 (m g r, r 8.00 m), "
                "centre +0.010 m from target",
                f"22.5 {BT} WriteBlockPoseFromGripper: block_id='block_1' "
                "world_pose=(-14.100, -3.050, 0.310) task_status=3 pose_status=2",
                "23.0 [bt_action_server-34] [INFO] [0.0] [bt_action_server]: Goal succeeded",
                "24.0 [wall_report.py-40] [INFO] [0.0] [wall_report]: WallReport truth: "
                "gz_block_a (-15.0000, -3.0000, 0.3000, 0.0000)",
                "24.0 [wall_report.py-40] [INFO] [0.0] [wall_report]: WallReport truth: "
                "gz_block_b (-14.1100, -3.0400, 0.3000, 3.1416)",
            ]
        )
        + "\n"
    )
    (tmp_path / "world_model_node_77_1.log").write_text(
        "[INFO] [22.8] [world_model_node]: REFINE_BLOCK: 'block_1' re-measured, moved 0.012 m "
        "(observed_faces=2).\n"
    )

    [path] = wall_report.write_reports(run)
    text = path.read_text()
    assert "**succeeded**, 2/2 blocks" in text
    assert "Gazebo true pose" in text
    # block_1 true: 3 cm short of its slot along +x, 4 cm towards -y (across is +y here),
    # turned 180 deg (the same block); the written pose was 1 cm off on each axis
    assert (
        "| block_1 | wall_02 | -14.080, -3.000, 0.300 | -14.110, -3.040, 0.300 "
        "| -3.0 | -4.0 | +0.0 | +0.0 | 1.7 |" in text
    )
    assert "| block_0 → block_1 | spacing | 92.0 | 89.0 | -3.0 |" in text
    assert "| block_0 | within 2.2 | 0 | +2.0 | – |" in text
    assert "| block_1 | – | 1 | +1.0 | moved 1.2 cm |" in text


def test_a_goal_that_aborts_mid_block_still_reports(tmp_path):
    """One block, never landed, a pre-world-target log line: a report, not a crash."""
    run = tmp_path / "run"
    run.mkdir()
    task = _task(10.0, 1, "block_0", -15.0).replace(
        " place_world=(-15.000,-3.000,0.300,0.0000)", ""
    )
    (run / "launch.log").write_text(
        "\n".join(
            [
                task,
                "30.0 [bt_action_server-34] [WARN] [0.0] [bt_action_server_execute_bt_rclcpp_node]: "
                "WaitForLoadOff: no landing within 40000 ms, the jaws stay shut",
                "31.0 [bt_action_server-34] [ERROR] [0.0] [bt_action_server]: Goal failed",
            ]
        )
        + "\n"
    )
    [path] = wall_report.write_reports(run)
    text = path.read_text()
    assert "**failed**, 0/1 blocks" in text
    assert "| block_0 | – | 0 | WaitForLoadOff: no landing within 40000 ms" in text


def test_gazebo_poses_come_back_in_the_plan_frame():
    """The spawner puts world (x, y, z, yaw) at gz (-x, -y - 6, z, yaw + pi)."""
    pose = {
        "position": {"x": 15.0, "y": -3.0, "z": 0.3},
        "orientation": {"z": 1.0, "w": 0.0},
    }
    x, y, z, yaw = wall_report.gz_to_world(pose)
    assert (round(x, 6), round(y, 6), round(z, 6)) == (-15.0, -3.0, 0.3)
    assert abs(wall_report._half_turn(yaw)) < 1e-9
