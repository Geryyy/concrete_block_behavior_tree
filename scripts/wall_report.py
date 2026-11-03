#!/usr/bin/env python3
"""
Wall assembly report from a run's ROS logs, one markdown file per assembly goal.

Plan vs placed (vs the sim's true poses), placement passes, landings, planner, timing.
Written beside launch.log as wall_report_<n>.md.

    wall_report.py [LOG_DIR]     # offline, every goal (default: <ros log>/latest)
    wall_report.py --follow      # in the sim launch: one report as each goal ends, true poses
                                 # read from Gazebo
"""

import json
import math
import os
import re
import subprocess
import sys
import time
from collections import defaultdict
from pathlib import Path

ANSI = re.compile(r"\x1b\[[0-9;]*m")
LINE = re.compile(r"^([\d.]+) \[([\w.\-]+)-\d+\] (.*)$")
LEVEL = re.compile(r"^\[(INFO|WARN|ERROR|FATAL)\] \[[\d.]+\] \[([^\]]+)\]: (.*)$")
GOAL_END = re.compile(r"\[bt_action_server\]: Goal (succeeded|failed|canceled)")
STARTED = re.compile(r"\[([\w.\-]+)-\d+\]: process started with pid \[(\d+)\]")
TASK = re.compile(
    # success=false with a task is "waiting for its physical block": no task yet
    r"GetNextAssemblyTask response \| success=true has_task=true task_id=(\S*) target=(\S+) "
    r".*message=(.*)$"
)
PLACE_WORLD = re.compile(r" place_world=\(([-\d.]+),([-\d.]+),([-\d.]+),([-\d.]+)\)")
PLACED = re.compile(
    r"WriteBlockPoseFromGripper: block_id='(\S+)' world_pose=\(([-\d.]+), ([-\d.]+), ([-\d.]+)\)"
)
LANDED = re.compile(r"WaitForLoadOff: landed .*centre ([+-][\d.]+) m")
PASS = re.compile(
    r"PlacePass: attempt (\d+) pass (\d+) (\w+) \| miss \(([-+\d.]+), ([-+\d.]+), ([-+\d.]+)\)"
)
TRUTH = re.compile(
    r"WallReport truth: (\S+) \(([-\d.]+), ([-\d.]+), ([-\d.]+), ([-\d.]+)\)"
)
APPROACH_FAIL = re.compile(r"\]: place_approach\s+RUNNING -> FAILURE")
PLAN = re.compile(
    r"\[([\d.]+)\] \[crane_planner\]: .*; (\d+) SQP iterations in ([\d.]+) s"
)
REFUSED = re.compile(r"\[([\d.]+)\] \[crane_planner\]: a2b_movement refused: (.*)$")
REFINED = re.compile(
    r"\[([\d.]+)\] \[world_model_node\]: REFINE_BLOCK: '(\S+)' re-measured, moved ([\d.]+) m"
)
UNSEEN = re.compile(
    r"\[([\d.]+)\] \[world_model_node\]: REFINE_BLOCK: no detection .* of '(\S+)'"
)
#: Farthest a placed pose may be from the Gazebo block it is matched to.
TRUTH_MATCH_M = 0.3
#: After a goal ends: the follow node's truth lines, and the side logs' last flush.
SETTLE_S = 30.0


def log_root() -> Path:
    if os.environ.get("ROS_LOG_DIR"):
        return Path(os.environ["ROS_LOG_DIR"])
    return Path(os.environ.get("ROS_HOME", "~/.ros")).expanduser() / "log"


def read_launch(log_dir: Path) -> tuple:
    """Rows (wall time, process, level, logger, message), ANSI stripped; node log paths by name."""
    rows, pids = [], {}
    for raw in (log_dir / "launch.log").read_text(errors="replace").splitlines():
        raw = ANSI.sub("", raw)
        if started := STARTED.search(raw):
            pids[started[1]] = started[2]
        line = LINE.match(raw)
        if not line:
            continue
        level = LEVEL.match(line[3])
        rows.append(
            (float(line[1]), line[2], *(level.groups() if level else ("", "", line[3])))
        )
    return rows, pids


def node_log(log_dir: Path, pids: dict, process: str) -> list:
    """Read a node's own log file, beside the launch directory, by the pid launch.log gives."""
    for path in log_dir.parent.glob(f"*_{pids.get(process, 'none')}_*.log"):
        return path.read_text(errors="replace").splitlines()
    return []


def goals(rows: list) -> list:
    """(rows, result) per goal that ran an assembly task; a goal ends on the server's verdict."""
    out, start = [], 0
    for i, (_t, _proc, _level, logger, text) in enumerate(rows):
        end = re.fullmatch(r"Goal (succeeded|failed|canceled)", text)
        if end and logger == "bt_action_server":
            chunk = rows[start : i + 1]
            if any(TASK.search(row[4]) for row in chunk):
                out.append((chunk, end[1]))
            start = i + 1
    return out


def yaw_of(q) -> float:
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def analyse(chunk: list, result: str, side: dict) -> dict:
    # From the first task, not the goal's start: launch and idle time are not the assembly.
    t0 = next(row[0] for row in chunk if TASK.search(row[4]))
    t1 = chunk[-1][0]
    blocks, order, warns = {}, [], defaultdict(int)
    current = None
    for t, proc, level, _logger, text in chunk:
        if level in ("WARN", "ERROR", "FATAL") and t >= t0:
            warns[(proc, level, re.sub(r"[-+]?\d[\d.]*", "#", text)[:100])] += 1
        if m := TASK.search(text):
            current = m[2]
            if current in blocks:  # the same block again: one record, its passes kept
                continue
            order.append(current)
            world = PLACE_WORLD.search(text)
            blocks[current] = {
                "task": m[1],
                "start": t,
                "passes": [],
                "approach_fail": 0,
                "planned": tuple(float(v) for v in world.groups()) if world else None,
            }
        elif current is None:
            continue
        elif m := PASS.search(text):
            blocks[current]["passes"].append((m[3], float(m[4]), float(m[5])))
        elif APPROACH_FAIL.search(text):
            blocks[current]["approach_fail"] += 1
        elif m := LANDED.search(text):
            blocks[current]["landed"] = (t, float(m[1]))
        elif "WaitForLoadOff:" in text and level in ("WARN", "ERROR"):
            blocks[current]["landing_fault"] = text[:120]
        elif (m := PLACED.search(text)) and m[1] in blocks:
            blocks[m[1]]["placed"] = tuple(float(v) for v in m.groups()[1:])
    # By position, not name: discovered blocks get world-model ids Gazebo never saw.
    truth = side.get("truth", {})
    for b in blocks.values():
        if "placed" in b and truth:
            pose = min(truth.values(), key=lambda t: math.dist(t[:2], b["placed"][:2]))
            if math.dist(pose[:2], b["placed"][:2]) < TRUTH_MATCH_M:
                b["truth"] = pose
    for name, later in zip(order, order[1:] + [None]):
        blocks[name]["end"] = blocks[later]["start"] if later else t1

    def window(line, pattern):
        m = pattern.search(line)
        return m if m and t0 - 1.0 <= float(m[1]) <= t1 + SETTLE_S else None

    plans = [
        (int(m[2]), float(m[3]))
        for line in side["planner"]
        if (m := window(line, PLAN))
    ]
    refusals = [m[2][:140] for line in side["planner"] if (m := window(line, REFUSED))]

    # The re-check of the placed block only: the same block is refined before its pick too.
    def after(m):
        return (
            m[2] in blocks and float(m[1]) >= blocks[m[2]].get("landed", (math.inf,))[0]
        )

    refined, unseen = {}, defaultdict(int)
    for line in side["world_model"]:
        if (m := window(line, REFINED)) and after(m) and float(m[3]) > 0.0:
            refined[m[2]] = float(m[3])
        elif (m := window(line, UNSEEN)) and after(m):
            unseen[m[2]] += 1
    return {
        "t0": t0,
        "t1": t1,
        "result": result,
        "order": order,
        "blocks": blocks,
        "warns": warns,
        "plans": plans,
        "refusals": refusals,
        "refined": refined,
        "unseen": unseen,
    }


def wall_frame(blocks: list):
    """
    Return `((along, across, z) from world, axis label)`.

    The wall axis runs from the first planned slot to the farthest one; with no horizontal
    run (one block, a stack) it is world x.
    """
    first = blocks[0]["planned"]
    last = max(
        (b["planned"] for b in blocks),
        key=lambda p: math.hypot(p[0] - first[0], p[1] - first[1]),
    )
    ax, ay = last[0] - first[0], last[1] - first[1]
    norm = math.hypot(ax, ay)
    if norm < 1e-6:
        return (lambda p: p), "world x / y"
    ux, uy = ax / norm, ay / norm
    frame = lambda p: (p[0] * ux + p[1] * uy, -p[0] * uy + p[1] * ux, p[2])  # noqa: E731
    return frame, "along / across the wall axis (first slot towards the farthest)"


def render(log_dir: Path, index: int, report: dict) -> str:
    blocks, order = report["blocks"], report["order"]
    planned = [blocks[n] for n in order if blocks[n]["planned"]]
    frame, axes = wall_frame(planned) if planned else (None, "–")
    truth = any("truth" in b for b in blocks.values())
    placed = sum(1 for n in order if "placed" in blocks[n])
    out = [
        f"# Wall assembly report {index}",
        "",
        f"- Log: `{log_dir}`",
        f"- Result: **{report['result']}**, {placed}/{len(order)} blocks placed in "
        f"{(report['t1'] - report['t0']) / 60.0:.1f} min wall clock",
        "- Deviations from: "
        + (
            "Gazebo true pose"
            if truth
            else "pose written at placement (FK + grasp offset)"
        ),
        "",
        "## Deviations from plan",
        "",
        f"cm, {axes}; yaw modulo 180 deg.",
        "",
        "| Block | Task | Planned x, y, z m | Actual x, y, z m | along | across | height "
        "| yaw deg | FK vs true |",
        "|---|---|---|---|---|---|---|---|---|",
    ]
    actual = {}
    for name in order:
        b = blocks[name]
        pose = b.get("truth") or b.get("placed")
        if pose is None or frame is None or b["planned"] is None:
            out.append(
                f"| {name} | {b['task']} | {_xyz(b['planned'])} | {_xyz(pose)} "
                "| – | – | – | – | – |"
            )
            continue
        actual[name] = pose
        p, a = frame(b["planned"]), frame(pose)
        yaw = (
            f"{math.degrees(_half_turn(b['truth'][3] - b['planned'][3])):+.1f}"
            if "truth" in b
            else "–"
        )
        fk = (
            f"{100 * math.dist(b['placed'], b['truth'][:3]):.1f}"
            if "truth" in b and "placed" in b
            else "–"
        )
        out.append(
            f"| {name} | {b['task']} | {_xyz(b['planned'])} | {_xyz(pose)} "
            f"| {_cm(a[0] - p[0])} | {_cm(a[1] - p[1])} | {_cm(a[2] - p[2])} | {yaw} | {fk} |"
        )
    if len(actual) > 1:
        out += _between(blocks, actual, frame)
    out += [
        "",
        "## Placement",
        "",
        "| Block | Passes: verdict, lateral miss cm | Failed approaches | Landing centre cm "
        "| Lidar re-check |",
        "|---|---|---|---|---|",
    ]
    for name in order:
        b = blocks[name]
        passes = (
            ", ".join(f"{v} {100 * math.hypot(x, y):.1f}" for v, x, y in b["passes"])
            or "–"
        )
        landing = (
            _cm(b["landed"][1])
            if "landed" in b
            else b.get("landing_fault", "not landed")
        )
        lidar = (
            f"moved {100 * report['refined'][name]:.1f} cm"
            if name in report["refined"]
            else f"not found ({report['unseen'][name]}×)"
            if name in report["unseen"]
            else "–"
        )
        out.append(
            f"| {name} | {passes} | {b['approach_fail']} | {landing} | {lidar} |"
        )
    out += [
        "",
        "## Timing",
        "",
        "Wall clock, s.",
        "",
        "| Block | Task to landed | Cycle |",
        "|---|---|---|",
    ]
    for name in order:
        b = blocks[name]
        landed = f"{b['landed'][0] - b['start']:.0f}" if "landed" in b else "–"
        out.append(f"| {name} | {landed} | {b['end'] - b['start']:.0f} |")
    out += ["", "## Planner", ""]
    solve = sorted(s for _i, s in report["plans"])
    if solve:
        out.append(
            f"- {len(solve)} plans, {len(report['refusals'])} refused; solve median "
            f"{solve[len(solve) // 2]:.2f} s, max {solve[-1]:.2f} s, "
            f"most SQP iterations {max(i for i, _s in report['plans'])}"
        )
    else:
        out.append("- no planner log found")
    out += [f"- refused: {r}" for r in report["refusals"]]
    out += ["", "## Warnings and errors", "", "Most frequent, numbers masked as #.", ""]
    for (proc, level, text), count in sorted(
        report["warns"].items(), key=lambda kv: -kv[1]
    )[:15]:
        out.append(f"- {count}× {level} `{proc}`: {text}")
    return "\n".join(out) + "\n"


def _between(blocks: dict, actual: dict, frame) -> list:
    """Spacing within a course, the across spread of its face, and each block on the one below."""
    out = [
        "",
        "## Between blocks",
        "",
        "| Pair | Kind | Planned cm | Actual cm | Δ cm |",
        "|---|---|---|---|---|",
    ]
    courses = defaultdict(list)
    for name in actual:
        courses[round(blocks[name]["planned"][2], 1)].append(name)
    plan = lambda n, axis: frame(blocks[n]["planned"])[axis]  # noqa: E731
    real = lambda n, axis: frame(actual[n])[axis]  # noqa: E731
    levels = sorted(courses)
    for level in levels:
        row = sorted(courses[level], key=lambda n: plan(n, 0))
        for left, right in zip(row, row[1:]):
            p, a = plan(right, 0) - plan(left, 0), real(right, 0) - real(left, 0)
            out.append(
                f"| {left} → {right} | spacing | {100 * p:.1f} | {100 * a:.1f} | {_cm(a - p)} |"
            )
        spread = [real(n, 1) for n in row]
        out.append(
            f"| course z {level:.1f} m | face spread across | 0.0 "
            f"| {100 * (max(spread) - min(spread)):.1f} | |"
        )
    for below, above in zip(levels, levels[1:]):
        for name in courses[above]:
            under = min(courses[below], key=lambda n: abs(plan(n, 0) - plan(name, 0)))
            for axis, kind in ((0, "bond offset"), (1, "across offset")):
                p, a = (
                    plan(name, axis) - plan(under, axis),
                    real(name, axis) - real(under, axis),
                )
                out.append(
                    f"| {under} → {name} | {kind} | {100 * p:.1f} | {100 * a:.1f} "
                    f"| {_cm(a - p)} |"
                )
    return out


def _xyz(p) -> str:
    return "–" if p is None else f"{p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}"


def _cm(metres: float) -> str:
    return f"{100 * metres:+.1f}"


def _half_turn(angle: float) -> float:
    """Wrap to +-90 deg: a block looks the same turned 180 deg."""
    return (angle + math.pi / 2) % math.pi - math.pi / 2


def write_reports(
    log_dir: Path, truth: dict | None = None, only_last: bool = False
) -> list:
    rows, pids = read_launch(log_dir)
    side = {
        "planner": node_log(log_dir, pids, "crane_planner_node"),
        "world_model": node_log(log_dir, pids, "world_model_node"),
    }
    logged = [(t, m) for t, _p, _l, _g, text in rows if (m := TRUTH.search(text))]
    found = goals(rows)
    written = []
    for index, (chunk, result) in enumerate(found, start=1):
        if only_last and index != len(found):
            continue
        end = chunk[-1][0]
        poses = {
            m[1]: tuple(float(v) for v in m.groups()[1:])
            for t, m in logged
            if end <= t <= end + SETTLE_S
        }
        if truth and index == len(found):
            poses = truth
        path = log_dir / f"wall_report_{index}.md"
        path.write_text(
            render(log_dir, index, analyse(chunk, result, {**side, "truth": poses}))
        )
        written.append(path)
    return written


def follow() -> None:
    """Watch launch.log and write the report as each wall goal ends, with the sim's true poses."""
    log_dir = Path(os.path.realpath(log_root() / "latest"))
    launch, offset, pending, task = log_dir / "launch.log", 0, b"", False
    while True:
        time.sleep(1.0)
        try:
            with launch.open(
                "rb"
            ) as handle:  # bytes: complete lines only, exact offsets
                handle.seek(offset)
                pending += handle.read()
                offset = handle.tell()
            fresh, _, pending = pending.rpartition(b"\n")
            fresh = ANSI.sub("", fresh.decode(errors="replace"))
            task = task or "GetNextAssemblyTask response" in fresh
            if not (GOAL_END.search(fresh) and task):
                continue
            task = False
            truth = gazebo_truth()
            for name, (x, y, z, yaw) in truth.items():
                # So an offline re-run finds the truth in launch.log too.
                print(
                    f"WallReport truth: {name} ({x:.4f}, {y:.4f}, {z:.4f}, {yaw:.4f})",
                    flush=True,
                )
            time.sleep(
                6.0
            )  # the planner's and world model's own logs flush behind launch.log
            for path in write_reports(log_dir, truth, only_last=True):
                print(f"WallReport written: {path}", flush=True)
        except Exception as error:  # noqa: BLE001 -- a report must never stop the next one
            print(f"WallReport failed: {error!r}", flush=True)


#: gz = Rz(yaw) * world + offset, the launch's gazebo_seed_frame_*. Ground at gz z = 0: the
#: first course rests at 0.300 (the oracle's 0.025 slab is gone).
GZ_WORLD, GZ_OFFSET, GZ_YAW, GZ_GROUND = "default", (0.0, -6.0, 0.0), math.pi, 0.0


def gazebo_truth(prefix: str = "block_") -> dict:
    """Read the true block poses in `world` from one Gazebo pose/info message; {} without a sim."""
    try:
        line = subprocess.run(
            [
                "gz",
                "topic",
                "-e",
                "-n",
                "1",
                "-t",
                f"/world/{GZ_WORLD}/pose/info",
                "--json-output",
            ],
            capture_output=True,
            text=True,
            timeout=30.0,
        ).stdout.splitlines()[0]
    except (OSError, IndexError, subprocess.TimeoutExpired):
        print(
            "WallReport: no Gazebo poses, the report uses the placed poses", flush=True
        )
        return {}
    return {
        pose["name"]: gz_to_world(pose)
        for pose in json.loads(line)["pose"]
        if pose.get("name", "").startswith(prefix)
    }


def gz_to_world(pose: dict) -> tuple:
    p, o = pose.get("position", {}), pose.get("orientation", {})
    gx, gy = p.get("x", 0.0) - GZ_OFFSET[0], p.get("y", 0.0) - GZ_OFFSET[1]
    c, s = math.cos(-GZ_YAW), math.sin(-GZ_YAW)
    yaw = (
        yaw_of((o.get("x", 0.0), o.get("y", 0.0), o.get("z", 0.0), o.get("w", 1.0)))
        - GZ_YAW
    )
    return (
        c * gx - s * gy,
        s * gx + c * gy,
        p.get("z", 0.0) - GZ_OFFSET[2] - GZ_GROUND,
        yaw,
    )


def main() -> None:
    if "--follow" in sys.argv:
        follow()
        return
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    log_dir = Path(args[0]) if args else Path(os.path.realpath(log_root() / "latest"))
    for path in write_reports(log_dir):
        print(path)


if __name__ == "__main__":
    main()
