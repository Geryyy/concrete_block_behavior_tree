#!/usr/bin/env python3
"""CPU Seyond emulator for the gz sim: ray-casts ground + concrete blocks, no rendering.

Publishes what the machine's Seyond publishes -- sensor_msgs/PointCloud2 in frame
`seyond` -- so the detector runs its hardware config. Scene: the world z=0 plane and
every gz model named `block_*` as a box. All poses come from one gz pose/info sample
(via gz_pose_relay, ros_gz_bridge drops the names): the `crane_model` origin is the ROS
`world` frame and the `seyond` link pose is relative to it, so no TF is needed.
Crane, truck and terrain are not in the cloud.

FOV and ray count are measured on real captures
(~/Documents/seibersdorf_dataset/2026_07_07_static__*): az -55.3..57.6 deg,
el -55.8..14.1 deg in the `seyond` frame, ~118k points per frame.
"""

import numpy as np
import rclpy
from geometry_msgs.msg import Transform
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage


def _matrix(t: Transform) -> np.ndarray:
    x, y, z, w = t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w
    m = np.eye(4)
    m[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    m[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
    return m


class SeyondSim(Node):
    def __init__(self):
        super().__init__("seyond_sim")
        p = self.declare_parameter
        self.sensor = p("sensor_link", "seyond").value
        self.crane_model = p("crane_model", "crane_model").value
        self.block_prefix = p("block_prefix", "block_").value
        self.half = np.array(p("block_size", [0.9, 0.6, 0.6]).value) / 2.0
        self.range_min = p("range_min", 0.5).value
        self.range_max = p("range_max", 60.0).value
        self.noise = p("range_noise_std", 0.02).value
        az = np.radians(np.linspace(-55.3, 57.6, p("azimuth_samples", 560).value))
        el = np.radians(np.linspace(-55.8, 14.1, p("elevation_samples", 210).value))
        az, el = np.meshgrid(az, el)
        self.rays = (
            np.stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)], -1)
            .reshape(-1, 3)
            .astype(np.float32)
        )
        self.rng = np.random.default_rng()
        self.poses, self.stamp = {}, None
        self.create_subscription(TFMessage, "gz_poses", self._on_poses, 10)
        self.pub = self.create_publisher(PointCloud2, "points", 1)
        self.create_timer(1.0 / p("rate_hz", 5.0).value, self._tick)

    def _on_poses(self, msg: TFMessage):
        poses = {t.child_frame_id: t.transform for t in msg.transforms}
        if self.crane_model in poses and self.sensor in poses:
            self.poses, self.stamp = poses, msg.transforms[0].header.stamp

    def _tick(self):
        if self.stamp is None:
            return
        d = self.rays
        ws = _matrix(self.poses[self.sensor])  # link pose in its model = sensor in world
        world_to_sensor = np.linalg.inv(ws)
        gz_to_world = np.linalg.inv(_matrix(self.poses[self.crane_model]))

        # ground: world z = 0
        dz = d @ ws[2, :3].astype(np.float32)
        hit = np.full(len(d), np.inf, np.float32)
        down = dz < 0
        hit[down] = -ws[2, 3] / dz[down]

        # blocks: cull to the rays inside each box's bounding cone, then slab test
        radius = np.linalg.norm(self.half)
        for name, pose in self.poses.items():
            if not name.startswith(self.block_prefix):
                continue
            box_in_sensor = world_to_sensor @ gz_to_world @ _matrix(pose)
            c = box_in_sensor[:3, 3]
            dist = np.linalg.norm(c)
            if dist <= radius:
                continue
            idx = np.flatnonzero(d @ (c / dist) > np.sqrt(1.0 - (radius / dist) ** 2))
            if idx.size == 0:
                continue
            rot = box_in_sensor[:3, :3]
            o = -rot.T @ c  # sensor origin in the box frame
            db = d[idx] @ rot  # ray directions in the box frame
            with np.errstate(divide="ignore", invalid="ignore"):
                t1, t2 = (-self.half - o) / db, (self.half - o) / db
            near = np.nanmax(np.minimum(t1, t2), axis=1)
            far = np.nanmin(np.maximum(t1, t2), axis=1)
            closer = (near <= far) & (near > 0) & (near < hit[idx])
            hit[idx[closer]] = near[closer]

        ok = (hit >= self.range_min) & (hit <= self.range_max)
        r = hit[ok] + self.rng.normal(0.0, self.noise, ok.sum()).astype(np.float32)
        header = Header(stamp=self.stamp, frame_id=self.sensor)
        self.pub.publish(point_cloud2.create_cloud_xyz32(header, d[ok] * r[:, None]))


def main():
    rclpy.init()
    rclpy.spin(SeyondSim())


if __name__ == "__main__":
    main()
