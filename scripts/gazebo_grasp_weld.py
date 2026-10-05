#!/usr/bin/env python3
"""
Sim only: weld the gripped block to the rotator while the jaws clamp it.

The jaws hold a block by contact friction, and Gazebo's mesh-rail contact does not: an
800 kg block slid 23 cm up the closed jaws on touchdown and fell out mid-carry (sim runs
37/38, 2026-10-02), where friction 10-100 at a 7 kN clamp would hold it. One gz
DetachableJoint per block, added at runtime to the crane model, welds the block at its
current relative pose while q9 clamps and lets go as soon as q9 opens.

gz CLI, like gazebo_block_spawner.py: no gz python bindings on jazzy.
"""

import ctypes
import json
import math
import re
import signal
import subprocess
import threading
import time

import rclpy
import rclpy.executors
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, String


def _gz(*args: str, timeout: float = 10.0) -> str:
    return subprocess.run(
        ["gz", *args], capture_output=True, text=True, timeout=timeout
    ).stdout


def _rotate(q: dict, v: list[float]) -> list[float]:
    """Rotate v by quaternion q (gz json omits zero fields; w defaults to 1)."""
    x, y, z = (q.get(k, 0.0) for k in "xyz")
    w = q.get("w", 1.0)
    # v' = v + 2w(u x v) + 2u x (u x v), u = (x, y, z)
    c1 = [y * v[2] - z * v[1], z * v[0] - x * v[2], x * v[1] - y * v[0]]
    c2 = [y * c1[2] - z * c1[1], z * c1[0] - x * c1[2], x * c1[1] - y * c1[0]]
    return [v[i] + 2 * w * c1[i] + 2 * c2[i] for i in range(3)]


def _die_with_parent():
    # A streaming `gz topic` child outlived its node and kept running.
    ctypes.CDLL("libc.so.6").prctl(1, signal.SIGTERM)  # PR_SET_PDEATHSIG


class GazeboGraspWeld(Node):
    def __init__(self):
        super().__init__("gazebo_grasp_weld")
        self.world = self.declare_parameter("world", "default").value
        self.crane = self.declare_parameter("crane_model", "crane_model").value
        self.parent_link = self.declare_parameter("parent_link", "moved_with_q8").value
        self.prefix = self.declare_parameter("block_prefix", "block_").value
        self.q9 = self.declare_parameter("q9_joint", "q9_left_rail_joint").value
        # Clamping, not touching: the sim clamps at ~7 kN, the grasp detector's 100 N fired
        # on empty jaws and lost the block.
        self.clamp_effort = self.declare_parameter("clamp_effort", 3000.0).value
        self.clamp_hold = self.declare_parameter("clamp_hold_s", 0.2).value
        self.open_margin = self.declare_parameter("open_margin", 0.01).value
        # A held block hangs under the rotator; its neighbours are >= 1 m away in xy.
        self.max_xy = self.declare_parameter("max_xy", 0.6).value

        self.crane_id = 0
        self.welds: set[str] = set()
        self.blocks = self._find_blocks()
        self.attached: str | None = None
        self.q9_at_attach = 0.0
        self.clamp_since: float | None = None
        self.next_look = 0.0
        self.armed = True
        # Commands go through one long-lived bridge and are re-sent until the joint's own
        # state confirms them: three one-shot `gz topic -p` detaches were all lost under load
        # and the lift carried a placed block away.
        verbs = ("attach", "detach")
        self.bridge = subprocess.Popen(
            ["ros2", "run", "ros_gz_bridge", "parameter_bridge"]
            + [
                f"{self._topic(b, v)}@std_msgs/msg/Empty]gz.msgs.Empty"
                for b in self.blocks
                for v in verbs
            ]
            + [
                f"{self._topic(b, 'state')}@std_msgs/msg/String[gz.msgs.StringMsg"
                for b in self.blocks
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=_die_with_parent,
        )
        self.pubs = {
            (b, v): self.create_publisher(Empty, self._topic(b, v), 10)
            for b in self.blocks
            for v in verbs
        }
        self.state: dict[str, str] = {}
        self.want: dict[str, str] = {}
        for b in self.blocks:
            self.create_subscription(
                String,
                self._topic(b, "state"),
                lambda m, b=b: self.state.update({b: m.data}),
                10,
            )
        self.create_timer(0.2, self._resend)
        self.poses: tuple[float, str] | None = None
        threading.Thread(target=self._stream_poses, daemon=True).start()
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)

    def _topic(self, block: str, verb: str) -> str:
        return f"/cbs_sim/{block}/{verb}"

    def _send(self, block: str, verb: str):
        self.want[block] = verb + "ed"
        self.pubs[(block, verb)].publish(Empty())

    def _resend(self):
        for block, wanted in self.want.items():
            if self.state.get(block) != wanted:
                self.pubs[(block, wanted[:-2])].publish(Empty())

    def _models(self) -> dict[str, int]:
        out = _gz(
            "service",
            "-s",
            f"/world/{self.world}/scene/info",
            "--reqtype",
            "gz.msgs.Empty",
            "--reptype",
            "gz.msgs.Scene",
            "--timeout",
            "5000",
            "--req",
            "",
        )
        return {
            m.group(1): int(m.group(2))
            for m in re.finditer(r'model \{\s*name: "([^"]+)"\s*id: (\d+)', out)
        }

    def _find_blocks(self) -> list[str]:
        """Wait for the crane and the spawner's blocks; remember the crane's entity id."""
        deadline = time.monotonic() + 120.0
        while True:
            models = self._models()
            blocks = sorted(n for n in models if n.startswith(self.prefix))
            if self.crane in models and blocks:
                time.sleep(2.0)  # let the spawner finish the set
                models = self._models()
                self.crane_id = models[self.crane]
                return sorted(n for n in models if n.startswith(self.prefix))
            if time.monotonic() > deadline:
                self.get_logger().error(
                    f"no {self.crane} or {self.prefix}* in gz; no welds"
                )
                return []
            time.sleep(1.0)

    def _attach(self, b: str) -> bool:
        """
        Weld b to the crane where it is.

        A DetachableJoint attaches the moment it is added, so it is added at the first grasp
        of b, not at startup: added there, every block was welded until its detach arrived
        and the crane dragged them. Later grasps use its attach topic.
        """
        if b in self.welds:
            self._send(b, "attach")
            return True
        if self.state.get(b) == "attached":
            # An add whose reply was lost: the joint is there. Adding again doubled it and the
            # weld landed mid-lift, jerking the MPC into QP failures.
            self.welds.add(b)
            self.want[b] = "attached"
            return True
        xml = (
            f"<parent_link>{self.parent_link}</parent_link><child_model>{b}</child_model>"
            f"<child_link>link</child_link><attach_topic>{self._topic(b, 'attach')}"
            f"</attach_topic><detach_topic>{self._topic(b, 'detach')}</detach_topic>"
            f"<output_topic>{self._topic(b, 'state')}</output_topic>"
        )
        req = (
            f"entity: {{id: {self.crane_id}}} plugins: {{name: "
            f'"gz::sim::systems::DetachableJoint", filename: '
            f'"gz-sim-detachable-joint-system", innerxml: {json.dumps(xml)}}}'
        )
        ok = "data: true" in _gz(
            "service",
            "-s",
            f"/world/{self.world}/entity/system/add",
            "--reqtype",
            "gz.msgs.EntityPlugin_V",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "5000",
            "--req",
            req,
        )
        if ok:
            self.welds.add(b)
            self.want[b] = "attached"  # an added joint attaches by itself
        return ok

    def _stream_poses(self):
        # The newest gz pose message: a one-shot `gz topic -e -n 1` per grasp came back
        # empty under load and two blocks were never welded.
        topic = f"/world/{self.world}/dynamic_pose/info"
        while True:
            proc = subprocess.Popen(
                ["gz", "topic", "-e", "-t", topic, "--json-output"],
                stdout=subprocess.PIPE,
                text=True,
                preexec_fn=_die_with_parent,
            )
            # Kept raw, parsed per grasp: parsing every message cost half a core.
            for line in proc.stdout:
                if self.parent_link in line:
                    self.poses = (time.monotonic(), line)
            time.sleep(1.0)

    def _block_under_jaws(self) -> str | None:
        if self.poses is None or time.monotonic() - self.poses[0] > 1.0:
            self.get_logger().warn("no fresh gz pose message to find the gripped block")
            return None
        try:
            poses = {p["name"]: p for p in json.loads(self.poses[1])["pose"]}
            crane, link = poses[self.crane], poses[self.parent_link]
        except (ValueError, KeyError):
            self.get_logger().warn(f"gz poses lack {self.crane} or {self.parent_link}")
            return None
        pos = lambda p: [p.get("position", {}).get(k, 0.0) for k in "xyz"]  # noqa: E731
        rel = _rotate(crane.get("orientation", {}), pos(link))
        tool = [a + b for a, b in zip(pos(crane), rel)]
        best, dist = None, self.max_xy
        for b in self.blocks:
            if b not in poses:
                continue
            bp = pos(poses[b])
            d = math.hypot(bp[0] - tool[0], bp[1] - tool[1])
            if d < dist and bp[2] < tool[2]:
                best, dist = b, d
        return best

    def _on_js(self, m: JointState):
        if self.q9 not in m.name or not m.effort:
            return
        i = m.name.index(self.q9)
        q, eff = m.position[i], m.effort[i]
        now = time.monotonic()
        if self.attached:
            if q > self.q9_at_attach + self.open_margin:
                self._send(self.attached, "detach")
                self.get_logger().info(f"released {self.attached} at q9 {q:.3f}")
                self.attached = None
                self.armed = False  # the jaws still squeeze as they open
            return
        if abs(eff) < self.clamp_effort:
            self.clamp_since = None
            self.armed = True
            return
        if not self.armed:
            return
        self.clamp_since = self.clamp_since or now
        if now - self.clamp_since < self.clamp_hold or now < self.next_look:
            return
        block = self._block_under_jaws()
        if block is None:
            self.get_logger().warn(
                "jaws clamped, no block under them to weld", throttle_duration_sec=5.0
            )
            self.next_look = now + 1.0
            return
        if not self._attach(block):
            self.get_logger().error(f"could not weld {block}")
            self.next_look = now + 0.3
            return
        self.attached, self.q9_at_attach = block, q
        self.get_logger().info(f"welded {block} at q9 {q:.3f}, effort {eff:.0f}")


def main():
    rclpy.init()
    node = GazeboGraspWeld()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    node.bridge.terminate()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
