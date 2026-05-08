#!/usr/bin/env python3
"""Spawn concrete blocks in Gazebo from the world-model seed YAML.

Reads ``world_model.initial_blocks`` from the same YAML that seeds
``world_model_node`` and calls the Gazebo ``/spawn_entity`` service once
per block.

The block ``position``/``yaw_deg`` fields are expressed in the world-model
frame, usually ``K0_mounting_base``.  Gazebo's world frame can differ from the
ROS/TF ``world`` frame because the crane entity itself is spawned with an
offset and yaw.  The ``gazebo_seed_frame_*`` parameters describe the
world-model frame pose in Gazebo coordinates so both RViz markers and Gazebo
blocks stay aligned relative to the crane.
"""

import math
import os

import rclpy
from rclpy.node import Node

import yaml
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnEntity


def _load_sdf() -> str:
    """Return the concrete_block SDF XML string."""
    pkg = get_package_share_directory("concrete_block_behavior_tree")
    sdf_path = os.path.join(pkg, "models", "concrete_block", "model.sdf")
    with open(sdf_path) as f:
        return f.read()


def _parse_initial_blocks(seed_yaml_path: str) -> list[dict]:
    """Parse the ``initial_blocks`` sequence from the seed YAML file."""
    with open(seed_yaml_path) as f:
        top = yaml.safe_load(f)

    raw = (
        top
        .get("world_model_node", {})
        .get("ros__parameters", {})
        .get("world_model", {})
        .get("initial_blocks", "")
    )
    if not raw:
        return []

    blocks = yaml.safe_load(raw)
    if not isinstance(blocks, list):
        return []
    return blocks


def _rpy_deg_to_quaternion(roll_deg: float, pitch_deg: float, yaw_deg: float) -> tuple:
    """Return (x, y, z, w) quaternion from RPY angles in degrees."""
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)

    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)

    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _quaternion_multiply(q1: tuple, q2: tuple) -> tuple:
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def _quaternion_conjugate(q: tuple) -> tuple:
    x, y, z, w = q
    return (-x, -y, -z, w)


def _rotate_vector(q: tuple, v: tuple) -> tuple:
    rotated = _quaternion_multiply(
        _quaternion_multiply(q, (v[0], v[1], v[2], 0.0)),
        _quaternion_conjugate(q),
    )
    return rotated[:3]


def _as_float_list(value, *, expected_len: int, name: str) -> list[float]:
    if len(value) != expected_len:
        raise ValueError(f"{name} must contain {expected_len} values")
    return [float(v) for v in value]


class GazeboBlockSpawner(Node):

    def __init__(self):
        super().__init__("gazebo_block_spawner")

        self.declare_parameter(
            "seed_config_file",
            os.path.join(
                get_package_share_directory("concrete_block_perception"),
                "config",
                "world_model_seed_pick_place.yaml",
            ),
        )
        self.declare_parameter("gazebo_world_frame", "world")
        self.declare_parameter("use_precomputed_gazebo_pose", True)
        self.declare_parameter("seed_frame_id", "K0_mounting_base")
        self.declare_parameter("gazebo_seed_frame_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("gazebo_seed_frame_rpy_deg", [0.0, 0.0, 0.0])
        self.declare_parameter("spawn_height_offset", 0.3)

        self._seed_file = self.get_parameter("seed_config_file").value
        self._world_frame = self.get_parameter("gazebo_world_frame").value
        self._use_precomputed_gazebo_pose = (
            self.get_parameter("use_precomputed_gazebo_pose").value
        )
        self._seed_frame_id = self.get_parameter("seed_frame_id").value
        self._gazebo_seed_frame_xyz = _as_float_list(
            self.get_parameter("gazebo_seed_frame_xyz").value,
            expected_len=3,
            name="gazebo_seed_frame_xyz",
        )
        seed_frame_rpy_deg = _as_float_list(
            self.get_parameter("gazebo_seed_frame_rpy_deg").value,
            expected_len=3,
            name="gazebo_seed_frame_rpy_deg",
        )
        self._gazebo_seed_frame_quat = _rpy_deg_to_quaternion(*seed_frame_rpy_deg)
        self._spawn_height_offset = float(
            self.get_parameter("spawn_height_offset").value
        )

        self._spawn_client = self.create_client(SpawnEntity, "/spawn_entity")

    def run(self):
        """Spawn all blocks. Call from main(), NOT from a callback."""
        blocks = _parse_initial_blocks(self._seed_file)
        if not blocks:
            self.get_logger().warn(f"No initial_blocks found in {self._seed_file}")
            return

        self.get_logger().info(f"Parsed {len(blocks)} block(s) from {self._seed_file}")

        if not self._spawn_client.wait_for_service(timeout_sec=30.0):
            self.get_logger().error("/spawn_entity service not available; aborting spawn.")
            return

        sdf_xml = _load_sdf()

        for block in blocks:
            self._spawn_block(block, sdf_xml)

        self.get_logger().info("Block spawning complete.")

    def _spawn_block(self, block: dict, sdf_xml: str):
        block_id = block.get("id", "block_unknown")
        if self._use_precomputed_gazebo_pose and block.get("gazebo_pose") is not None:
            gz_pose = block.get("gazebo_pose")
            if len(gz_pose) < 6:
                self.get_logger().error(
                    f"Block '{block_id}' has invalid gazebo_pose; skipping."
                )
                return
            x, y, z, roll_deg, pitch_deg, yaw_deg = gz_pose
            qx, qy, qz, qw = _rpy_deg_to_quaternion(roll_deg, pitch_deg, yaw_deg)
        else:
            block_frame = block.get("frame_id", self._seed_frame_id)
            if block_frame != self._seed_frame_id:
                self.get_logger().error(
                    f"Block '{block_id}' frame_id '{block_frame}' does not match "
                    f"seed_frame_id '{self._seed_frame_id}'; skipping."
                )
                return

            position = block.get("position")
            if position is None or len(position) < 3:
                self.get_logger().error(
                    f"Block '{block_id}' has no valid position; skipping."
                )
                return

            bx, by, bz = float(position[0]), float(position[1]), float(position[2])
            rx, ry, rz = _rotate_vector(self._gazebo_seed_frame_quat, (bx, by, bz))
            x = self._gazebo_seed_frame_xyz[0] + rx
            y = self._gazebo_seed_frame_xyz[1] + ry
            z = self._gazebo_seed_frame_xyz[2] + rz + self._spawn_height_offset

            yaw_deg = float(block.get("yaw_deg", 0.0))
            block_quat = _rpy_deg_to_quaternion(0.0, 0.0, yaw_deg)
            qx, qy, qz, qw = _quaternion_multiply(
                self._gazebo_seed_frame_quat, block_quat
            )

        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        req = SpawnEntity.Request()
        req.name = block_id
        req.xml = sdf_xml
        req.reference_frame = ""  # Gazebo world frame
        req.initial_pose = pose

        future = self._spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() is None:
            self.get_logger().error(f"SpawnEntity call timed out for {block_id}.")
        elif not future.result().success:
            self.get_logger().error(
                f"SpawnEntity rejected {block_id}: {future.result().status_message}"
            )
        else:
            self.get_logger().info(
                f"Spawned {block_id} at world ({x:.3f}, {y:.3f}, {z:.3f})"
            )


def main():
    rclpy.init()
    node = GazeboBlockSpawner()
    # run() calls spin_until_future_complete which requires no concurrent executor.
    # Calling it here (outside rclpy.spin) avoids the re-entrant deadlock that
    # occurs when spin_until_future_complete is called from inside a timer callback.
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
