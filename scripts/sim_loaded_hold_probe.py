#!/usr/bin/env python3
"""
Sim harness (issue 234): park a carried 800 kg block and hold, with no tree.

Drives the same two planners the tree drives (``/a2b_movement``,
``/grip_traj_movement``) and the same JTC action, in the pick order of
subtree_pick_block.xml, then stops sending goals: ``trajectory_controller_a2b``
stays active and holds for ``hold_s``, after which it is deactivated for
``free_s``. Both windows are recorded, so the two hypotheses separate:

  flat command, same decay held as free  => the loaded mode is free and
                                            under-damped, nothing damps it
  command ringing at the tip's period,
  decay faster held than free            => the position hold feeds the mode

No behaviour tree, so the 30 s settle gate of cbs_subtree_a2b_movement.xml does
not cut the window short, and nothing deactivates the controller at the end of a
move. `gui:=False`, so no approval producer is needed either -- this script
replaces the approval-gated subtree, it does not pass through it.

Writes `<out_dir>/{joint_states,controller_state}.csv` and prints the per-window
numbers. Sim only.
"""

import csv
import math
import os
import threading
import time

import numpy as np
import rclpy
import tf2_geometry_msgs  # noqa: F401  PoseStamped <-> tf2
from builtin_interfaces.msg import Duration as DurationMsg
from builtin_interfaces.msg import Time as TimeMsg
from concrete_block_assembly_interfaces.srv import GetNextAssemblyTask, PlanControl
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance, JointTrajectoryControllerState
from controller_manager_msgs.srv import SwitchController
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

from timber_crane_planning_interfaces.srv import CalcGripMovement, CalcMovement
from wood_log_msgs.msg import LogShape

A2B = "trajectory_controller_a2b"
PLANNING_FRAME = "K0_mounting_base"
# get_next_assembly_task.cpp:38 does this same transform; the planners read
# K0_mounting_base, the plan server answers in `world`.

# Order is the contract's canonical eight (crane_model/config/hydraulics.yaml).
# Columns are looked up by name on every message: the two stacks publish
# different /joint_states orders (CLAUDE.md).
JOINTS = [
    "theta1_slewing_joint",
    "theta2_boom_joint",
    "theta3_arm_joint",
    "q4_big_telescope",
    "theta6_tip_joint",
    "theta7_tilt_joint",
    "theta8_rotator_joint",
    "q9_left_rail_joint",
]
ACTUATED = [j for j in JOINTS if j not in ("theta6_tip_joint", "theta7_tilt_joint")]
# Goal tolerance is asked of the actuated five only, exactly as every
# FollowJointTrajectory node in the trees does: q9 is the gripper and stalls
# against the clamped block, so a tolerance on it aborts every close.
GOAL_JOINTS = [j for j in ACTUATED if j != "q9_left_rail_joint"]
TIP = "theta6_tip_joint"
TILT = "theta7_tilt_joint"

PHASE_DESCEND, PHASE_CLOSE, PHASE_OPEN, PHASE_LIFT = 1, 2, 3, 4


def _yaw(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def _spectrum(t: np.ndarray, x: np.ndarray) -> tuple[float, float]:
    """Dominant frequency [Hz] and its single-sided amplitude of x(t)."""
    if len(t) < 64:
        return float("nan"), float("nan")
    dt = (t[-1] - t[0]) / (len(t) - 1)
    grid = np.arange(t[0], t[-1], dt)
    y = np.interp(grid, t, x)
    w = np.hanning(len(y))
    spec = np.abs(np.fft.rfft((y - y.mean()) * w)) * 2.0 / w.sum()
    freq = np.fft.rfftfreq(len(y), dt)
    k = int(np.argmax(spec[1:])) + 1
    return float(freq[k]), float(spec[k])


def _envelope(t: np.ndarray, x: np.ndarray, bin_s: float = 6.0) -> tuple:
    """Peak-to-peak per bin, and the decay time constant fitted through them."""
    if len(t) < 64:
        return [], float("nan")
    edges = np.arange(t[0], t[-1] + bin_s, bin_s)
    p2p, mid = [], []
    for lo, hi in zip(edges[:-1], edges[1:]):
        m = (t >= lo) & (t < hi)
        if m.sum() < 16:
            continue
        p2p.append(float(x[m].max() - x[m].min()))
        mid.append(0.5 * (lo + hi))
    if len(p2p) < 3 or min(p2p) <= 0.0:
        return p2p, float("nan")
    slope = np.polyfit(np.array(mid) - mid[0], np.log(p2p), 1)[0]
    return p2p, float(-1.0 / slope) if slope < 0 else float("inf")


class LoadedHoldProbe(Node):
    def __init__(self):
        super().__init__("sim_loaded_hold_probe")
        self.set_parameters(
            [rclpy.parameter.Parameter("use_sim_time", value=True)]
        ) if not self.get_parameter("use_sim_time").value else None
        self.plan = self.declare_parameter("plan", "example_wall").value
        self.hold_s = float(self.declare_parameter("hold_s", 60.0).value)
        self.free_s = float(self.declare_parameter("free_s", 60.0).value)
        self.out_dir = self.declare_parameter("out_dir", "").value
        self.slow_down = float(self.declare_parameter("slow_down", 1.0).value)
        self.payload_mass = float(self.declare_parameter("payload_mass", 800.0).value)
        self.carry = bool(self.declare_parameter("carry_to_endpoint", True).value)
        # The tree's planner collision body for the carried block
        # (wall_assembly.xml: a cylinder around the upright 0.9 x 0.6 x 0.6 m).
        self.shape = LogShape(length=0.60, radius_top=0.541, radius_bottom=0.541)

        self.phase = "init"
        self.js_rows: list[list] = []
        self.cs_rows: list[list] = []
        self.marks: list[tuple[str, float, float]] = []

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.plan_control = self.create_client(
            PlanControl, "/concrete_block_wall_plan_server/plan_control"
        )
        self.next_task = self.create_client(
            GetNextAssemblyTask,
            "/concrete_block_wall_plan_server/get_next_assembly_task",
        )
        self.a2b_plan = self.create_client(CalcMovement, "/a2b_movement")
        self.grip_plan = self.create_client(CalcGripMovement, "/grip_traj_movement")
        self.switch = self.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )
        self.jtc = ActionClient(
            self, FollowJointTrajectory, f"/{A2B}/follow_joint_trajectory"
        )
        self.create_subscription(JointState, "/joint_states", self._on_js, 100)
        self.create_subscription(
            JointTrajectoryControllerState,
            "/trajectory_controllers/controller_state",
            self._on_cs,
            100,
        )

    # ---------------- recording ----------------

    def _now(self) -> tuple[float, float]:
        return self.get_clock().now().nanoseconds * 1e-9, time.monotonic()

    def _on_js(self, m: JointState):
        t_sim, t_wall = self._now()
        row = [t_sim, t_wall, self.phase]
        for name in JOINTS:
            if name in m.name:
                i = m.name.index(name)
                row += [
                    m.position[i] if i < len(m.position) else math.nan,
                    m.velocity[i] if i < len(m.velocity) else math.nan,
                    m.effort[i] if i < len(m.effort) else math.nan,
                ]
            else:
                row += [math.nan] * 3
        self.js_rows.append(row)

    def _on_cs(self, m: JointTrajectoryControllerState):
        t_sim, t_wall = self._now()
        row = [t_sim, t_wall, self.phase]
        for name in ACTUATED:
            i = m.joint_names.index(name) if name in m.joint_names else -1
            out = m.output.velocities
            err = m.error.positions
            row += [
                out[i] if 0 <= i < len(out) else math.nan,
                err[i] if 0 <= i < len(err) else math.nan,
            ]
        self.cs_rows.append(row)

    def _mark(self, phase: str):
        t_sim, t_wall = self._now()
        self.marks.append((phase, t_sim, t_wall))
        self.phase = phase
        self.get_logger().info(f"=== phase {phase} at sim {t_sim:.2f}")

    # ---------------- plumbing ----------------

    def _wait(self, future, timeout: float, label: str):
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                raise RuntimeError(f"{label}: timed out after {timeout:.0f} s")
            time.sleep(0.05)
        return future.result()

    def _call(self, client, request, timeout: float, label: str):
        if not client.wait_for_service(timeout_sec=30.0):
            raise RuntimeError(f"{label}: service not advertised")
        return self._wait(client.call_async(request), timeout, label)

    def _switch_a2b(self, active: bool):
        request = SwitchController.Request()
        request.activate_controllers = [A2B] if active else []
        request.deactivate_controllers = [] if active else [A2B]
        request.strictness = SwitchController.Request.BEST_EFFORT
        request.activate_asap = True
        request.timeout = DurationMsg(sec=5)
        answer = self._call(self.switch, request, 30.0, "switch_controller")
        self.get_logger().info(f"switch {A2B} active={active}: ok={answer.ok}")

    def _play(self, trajectory, tolerance: float, label: str):
        """Send the trajectory as a JTC goal and wait it out."""
        if not trajectory.points:
            raise RuntimeError(f"{label}: empty trajectory")
        # The planners' stamps are meaningless (issue 219); the JTC reads a zero
        # stamp as "start now", which is what follow_joint_trajectory.cpp does.
        trajectory.header.stamp = TimeMsg(sec=0, nanosec=0)
        duration = trajectory.points[-1].time_from_start
        plan_s = duration.sec + duration.nanosec * 1e-9
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        goal.goal_tolerance = [
            JointTolerance(name=j, position=tolerance) for j in GOAL_JOINTS
        ]
        if not self.jtc.wait_for_server(timeout_sec=60.0):
            raise RuntimeError(f"{label}: no {A2B} action server")
        handle = self._wait(self.jtc.send_goal_async(goal), 30.0, f"{label} accept")
        if not handle.accepted:
            raise RuntimeError(f"{label}: goal rejected")
        started = time.monotonic()
        result = self._wait(
            handle.get_result_async(), plan_s + 180.0, f"{label} result"
        )
        self.get_logger().info(
            f"{label}: plan {plan_s:.1f} s, goal held {time.monotonic() - started:.1f} s "
            f"wall, status {result.status}, error_code {result.result.error_code}"
        )
        if result.result.error_code != 0:
            raise RuntimeError(f"{label}: error_code {result.result.error_code}")

    def _to_planning(self, pose, label: str):
        if not pose.header.frame_id or pose.header.frame_id == PLANNING_FRAME:
            return pose
        for _ in range(40):
            try:
                return self.tf_buffer.transform(pose, PLANNING_FRAME)
            except Exception:  # noqa: BLE001 - tf2 raises several unrelated types
                time.sleep(0.25)
        raise RuntimeError(f"{label}: no {pose.header.frame_id} -> {PLANNING_FRAME} TF")

    def _a2b(self, pose, carries: bool, label: str):
        request = CalcMovement.Request()
        request.y_n = pose.pose.position
        request.phi_tool_n = _yaw(pose.pose.orientation)
        request.carries_log = carries
        request.log_carrying = self.shape
        request.coll_shape = self.shape
        request.m_log = self.payload_mass
        request.s_log_8.x = 0.0
        request.s_log_8.y = 0.0
        request.s_log_8.z = math.nan  # the tree's blackboard default
        request.slow_down = self.slow_down
        request.t_end = 0.0
        answer = self._call(self.a2b_plan, request, 120.0, f"{label} plan")
        if not answer.success:
            raise RuntimeError(f"{label}: planner refused")
        return answer.trajectory

    def _grip(self, pose, phase: int, label: str):
        request = CalcGripMovement.Request()
        request.y_n = pose.pose.position
        request.phi_tool_n = _yaw(pose.pose.orientation)
        request.select_phases = phase
        request.slow_down = self.slow_down
        request.xy_n = [math.nan, math.nan]
        request.coll_shape = self.shape
        answer = self._call(self.grip_plan, request, 120.0, f"{label} plan")
        if answer.success != 1:
            raise RuntimeError(f"{label}: grip planner returned {answer.success}")
        return answer.trajectory

    # ---------------- the run ----------------

    def run(self):
        for _ in range(240):
            if self.js_rows:
                break
            time.sleep(0.5)
        if not self.js_rows:
            raise RuntimeError("no /joint_states")

        request = PlanControl.Request()
        request.command = "activate"
        request.wall_plan_name = self.plan
        answer = self._call(self.plan_control, request, 120.0, "activate plan")
        if not answer.success:
            raise RuntimeError(f"activate {self.plan}: {answer.message}")
        self.get_logger().info(f"activated {self.plan}, {answer.num_goals} goals")

        task = self._call(
            self.next_task, GetNextAssemblyTask.Request(), 60.0, "next task"
        )
        if not (task.success and task.has_task):
            raise RuntimeError(f"no assembly task: {task.message}")
        pickup = self._to_planning(task.pickup_pose, "pickup")
        approach = self._to_planning(task.pickup_approach_pose, "pickup approach")
        endpoint = self._to_planning(task.approach_pose, "place approach")
        self.get_logger().info(
            f"task {task.task_id} block {task.target_block_id} "
            f"pickup {pickup.pose.position.x:.2f} {pickup.pose.position.y:.2f} "
            f"{pickup.pose.position.z:.2f} in {PLANNING_FRAME}"
        )

        self._switch_a2b(True)
        self._mark("move_above")
        self._play(self._a2b(approach, False, "move_above"), 0.05, "move_above")
        self._mark("open")
        self._play(self._grip(pickup, PHASE_OPEN, "open"), 0.20, "open")
        self._mark("descend")
        self._play(self._grip(pickup, PHASE_DESCEND, "descend"), 0.05, "descend")
        self._mark("close")
        self._play(self._grip(pickup, PHASE_CLOSE, "close"), 0.20, "close")
        self._mark("lift")
        self._play(self._grip(pickup, PHASE_LIFT, "lift"), 0.20, "lift")

        if self.carry:
            self._mark("carry")
            try:
                self._play(self._a2b(endpoint, True, "carry"), 0.05, "carry")
            except RuntimeError as error:
                # The hold is the measurement; a refused or aborted carry move
                # still leaves a loaded, ringing pose to hold.
                self.get_logger().error(f"carry move failed, holding anyway: {error}")

        self._mark("hold")
        self._sleep_sim(self.hold_s)
        if self.free_s > 0.0:
            self._switch_a2b(False)
            self._mark("free")
            self._sleep_sim(self.free_s)
        else:
            self._switch_a2b(False)
        self._mark("done")

    def _sleep_sim(self, seconds: float):
        """Wait `seconds` of /clock time, bounded by 10x that in wall time."""
        start_sim, start_wall = self._now()
        while True:
            t_sim, t_wall = self._now()
            if t_sim - start_sim >= seconds:
                return
            if t_wall - start_wall > 10.0 * seconds + 30.0:
                self.get_logger().error(
                    f"sim clock gave {t_sim - start_sim:.1f} s of {seconds:.0f} s "
                    f"in {t_wall - start_wall:.0f} s wall; giving up on the window"
                )
                return
            time.sleep(0.2)

    # ---------------- report ----------------

    def write(self):
        if not self.out_dir:
            return
        os.makedirs(self.out_dir, exist_ok=True)
        head = ["t_sim", "t_wall", "phase"]
        with open(os.path.join(self.out_dir, "joint_states.csv"), "w") as handle:
            writer = csv.writer(handle)
            writer.writerow(
                head + [f"{k}_{j}" for j in JOINTS for k in ("q", "dq", "eff")]
            )
            writer.writerows(self.js_rows)
        with open(os.path.join(self.out_dir, "controller_state.csv"), "w") as handle:
            writer = csv.writer(handle)
            writer.writerow(
                head + [f"{k}_{j}" for j in ACTUATED for k in ("u", "e_pos")]
            )
            writer.writerows(self.cs_rows)
        with open(os.path.join(self.out_dir, "marks.csv"), "w") as handle:
            writer = csv.writer(handle)
            writer.writerow(["phase", "t_sim", "t_wall"])
            writer.writerows(self.marks)

    def report(self):
        js = [r for r in self.js_rows]
        cs = [r for r in self.cs_rows]
        print("\n==== issue 234: loaded hold ====")
        for phase in ("hold", "free"):
            jw = [r for r in js if r[2] == phase]
            cw = [r for r in cs if r[2] == phase]
            if len(jw) < 64:
                print(f"\n-- {phase}: {len(jw)} joint_state samples, skipped")
                continue
            t = np.array([r[0] for r in jw])
            wall = np.array([r[1] for r in jw])
            span = t[-1] - t[0]
            rtf = span / max(wall[-1] - wall[0], 1e-9)
            print(f"\n-- {phase}: {span:.1f} s sim, RTF {rtf:.2f}, {len(jw)} samples")
            for name in (TIP, TILT):
                base = 3 + 3 * JOINTS.index(name)
                q = np.array([r[base] for r in jw])
                dq = np.array([r[base + 1] for r in jw])
                f_q, a_q = _spectrum(t, q)
                p2p, tau = _envelope(t, q)
                print(
                    f"   {name:18s} f {f_q:5.3f} Hz  amp {a_q:7.4f} rad  "
                    f"|dq| rms {np.sqrt(np.mean(dq**2)):6.4f} max {np.abs(dq).max():6.4f} rad/s"
                )
                print(
                    f"   {'':18s} p2p/{6}s "
                    + " ".join(f"{v:.4f}" for v in p2p)
                    + f"   tau {tau:.0f} s"
                )
            if len(cw) < 64:
                print(f"   no controller_state in {phase} ({len(cw)} samples)")
                continue
            tc = np.array([r[0] for r in cw])
            tip_f = _spectrum(t, np.array([r[3 + 3 * JOINTS.index(TIP)] for r in jw]))[
                0
            ]
            for i, name in enumerate(ACTUATED):
                u = np.array([r[3 + 2 * i] for r in cw])
                e = np.array([r[4 + 2 * i] for r in cw])
                f_u, a_u = _spectrum(tc, u)
                eff = np.array([r[5 + 3 * JOINTS.index(name)] for r in jw])
                print(
                    f"   {name:18s} u rms {np.sqrt(np.mean(u**2)):7.5f} "
                    f"max {np.abs(u).max():7.5f} rad/s  f {f_u:5.3f} Hz "
                    f"(tip {tip_f:5.3f})  amp {a_u:7.5f}  "
                    f"e_pos p2p {e.max() - e.min():6.4f} rad  "
                    f"eff rms {np.sqrt(np.nanmean(eff**2)):8.1f}"
                )


def main():
    rclpy.init()
    node = LoadedHoldProbe()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    failure = None
    try:
        node.run()
    except (RuntimeError, KeyboardInterrupt) as error:
        failure = error
        node.get_logger().error(f"probe aborted: {error}")
    node.write()
    node.report()
    if failure is not None:
        print(f"\nABORTED: {failure}")
    # Order matters: shut the executor down and let spin() return before the
    # context goes away, or rclpy aborts (SIGABRT) out of the spin thread and
    # the exit code says 134 instead of what the run did.
    executor.shutdown()
    spin.join(timeout=5.0)
    node.destroy_node()
    rclpy.shutdown()
    return 1 if failure is not None else 0


if __name__ == "__main__":
    raise SystemExit(main())
