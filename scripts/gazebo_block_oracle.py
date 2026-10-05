#!/usr/bin/env python3
"""
Sim only: the block detector's service, answered from Gazebo's true block poses plus noise.

Stands in for the lidar + concrete_block_detector chain so the world model's REFINE_* logic
and the BT's use of it can be tested on their own; swap in the real detector to compare.
Detections are anonymous and in the world model's `world` frame, as the detector's are.
"""

import ctypes
import json
import math
import random
import signal
import subprocess
import threading
import time

import rclpy
import rclpy.executors
from concrete_block_world_model_interfaces.msg import Block
from concrete_block_world_model_interfaces.srv import DiscoverBlocks
from rclpy.node import Node


def _qmul(a, b):
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def _die_with_parent():
    # A streaming `gz topic` child outlived its node and kept running (sim run 124).
    ctypes.CDLL("libc.so.6").prctl(1, signal.SIGTERM)  # PR_SET_PDEATHSIG


class GazeboBlockOracle(Node):
    def __init__(self):
        super().__init__("concrete_block_detector")
        self.world = self.declare_parameter("world", "default").value
        self.prefix = self.declare_parameter("block_prefix", "block_").value
        # gz = Rz(yaw) * world + offset: the spawner's gazebo_seed_frame_*.
        self.offset = self.declare_parameter(
            "gazebo_seed_frame_xyz", [0.0, -6.0, 0.0]
        ).value
        self.yaw = math.radians(
            self.declare_parameter("gazebo_seed_frame_yaw_deg", 180.0).value
        )
        # The testsite ground's surface: blocks rest 2.5 cm above their world-model height.
        self.ground = self.declare_parameter("gazebo_ground_z", 0.025).value
        self.sigma_pos = self.declare_parameter("sigma_position_m", 0.01).value
        self.sigma_ang = math.radians(
            self.declare_parameter("sigma_angle_deg", 0.5).value
        )
        self.rng = random.Random(self.declare_parameter("seed", 0).value)
        # One streaming subscription, answered from its newest message: a one-shot
        # `gz topic -e -n 1` per request took over 5 s under load and the refine timed out
        # (sim terrain run 123).
        self.latest: tuple[float, str] | None = None
        threading.Thread(target=self._stream, daemon=True).start()
        self.create_service(DiscoverBlocks, "~/discover_blocks", self._discover)

    def _stream(self):
        topic = f"/world/{self.world}/dynamic_pose/info"
        while True:
            proc = subprocess.Popen(
                ["gz", "topic", "-e", "-t", topic, "--json-output"],
                stdout=subprocess.PIPE,
                text=True,
                preexec_fn=_die_with_parent,
            )
            # Kept raw, parsed per request: parsing every message cost a quarter core.
            for line in proc.stdout:
                if f'"{self.prefix}' in line:
                    self.latest = (time.monotonic(), line)
            time.sleep(1.0)

    def _gz_poses(self) -> dict:
        if self.latest is None or time.monotonic() - self.latest[0] > 1.0:
            raise RuntimeError("no gz pose message in the last second")
        return {p["name"]: p for p in json.loads(self.latest[1])["pose"]}

    def _to_world(self, p):
        pos = p.get("position", {})
        o = p.get("orientation", {})
        gx = pos.get("x", 0.0) - self.offset[0]
        gy = pos.get("y", 0.0) - self.offset[1]
        c, s = math.cos(-self.yaw), math.sin(-self.yaw)
        xyz = (
            c * gx - s * gy,
            s * gx + c * gy,
            pos.get("z", 0.0) - self.offset[2] - self.ground,
        )
        q_gz = (o.get("x", 0.0), o.get("y", 0.0), o.get("z", 0.0), o.get("w", 1.0))
        q = _qmul((0.0, 0.0, math.sin(-self.yaw / 2), math.cos(-self.yaw / 2)), q_gz)
        return xyz, q

    def _noisy(self, xyz, q):
        xyz = [v + self.rng.gauss(0.0, self.sigma_pos) for v in xyz]
        axis = [self.rng.gauss(0.0, 1.0) for _ in range(3)]
        norm = math.sqrt(sum(a * a for a in axis)) or 1.0
        angle = self.rng.gauss(0.0, self.sigma_ang)
        dq = tuple(a / norm * math.sin(angle / 2) for a in axis) + (
            math.cos(angle / 2),
        )
        return xyz, _qmul(dq, q)

    def _discover(self, request, response):
        try:
            poses = self._gz_poses()
        except Exception as error:  # noqa: BLE001 -- a lost gz message is a failed pass
            response.success = False
            response.message = f"no gz poses: {error}"
            return response
        response.blocks.header.frame_id = "world"
        response.blocks.header.stamp = self.get_clock().now().to_msg()
        for name in sorted(n for n in poses if n.startswith(self.prefix)):
            xyz, q = self._noisy(*self._to_world(poses[name]))
            block = Block()
            block.pose_status = Block.POSE_PRECISE
            block.pose.position.x, block.pose.position.y, block.pose.position.z = xyz
            (
                block.pose.orientation.x,
                block.pose.orientation.y,
                block.pose.orientation.z,
                block.pose.orientation.w,
            ) = q
            for i in range(3):
                block.pose_covariance[7 * i] = self.sigma_pos**2
                block.pose_covariance[21 + 7 * i] = self.sigma_ang**2
            block.confidence = 1.0
            block.observed_faces = 3
            block.last_seen = response.blocks.header.stamp
            response.blocks.blocks.append(block)
        response.success = True
        # The testsite ground is flat: the plane the detector's fit would return, no cells.
        response.ground_height_m = 0.0
        response.ground_model.valid = True
        response.ground_model.plane_normal.z = 1.0
        response.ground_model.plane_source = "oracle"
        response.ground_model.cell_size = 1.0
        response.message = f"oracle: {len(response.blocks.blocks)} blocks from gz"
        return response


def main():
    rclpy.init()
    node = GazeboBlockOracle()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
