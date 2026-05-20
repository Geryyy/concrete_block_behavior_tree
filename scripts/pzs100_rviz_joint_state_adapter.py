#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class Pzs100RvizJointStateAdapter(Node):
    def __init__(self):
        super().__init__("pzs100_rviz_joint_state_adapter")
        self.declare_parameter("input_topic", "/joint_states")
        self.declare_parameter("output_topic", "/joint_states_rviz")
        self.declare_parameter("opening_joint", "q9_left_rail_joint")
        self.declare_parameter("opening_state_factor", 2.0)
        self.declare_parameter("opening_offset", 0.21)
        self.declare_parameter("follower_joint", "q11_right_rail_joint")

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self._opening_joint = self.get_parameter("opening_joint").value
        self._opening_state_factor = self.get_parameter("opening_state_factor").value
        self._opening_offset = self.get_parameter("opening_offset").value
        self._follower_joint = self.get_parameter("follower_joint").value

        self._publisher = self.create_publisher(JointState, output_topic, 10)
        self._subscription = self.create_subscription(
            JointState, input_topic, self._joint_state_callback, 10
        )

    def _joint_state_callback(self, msg):
        out = JointState()
        out.header = msg.header
        out.name = list(msg.name)
        out.position = list(msg.position)
        out.velocity = list(msg.velocity)
        out.effort = list(msg.effort)

        try:
            opening_index = out.name.index(self._opening_joint)
        except ValueError:
            self._publisher.publish(out)
            return

        # Invert the EPSCOPE actuator read():
        #   reported = (sim_position + offset) * state_factor
        # so the URDF (which expects raw sim positions) needs
        #   sim_position = reported / state_factor - offset
        sim_position = (
            out.position[opening_index] / self._opening_state_factor
            - self._opening_offset
        )
        out.position[opening_index] = sim_position
        if opening_index < len(out.velocity):
            out.velocity[opening_index] /= self._opening_state_factor

        sim_velocity = (
            out.velocity[opening_index] if opening_index < len(out.velocity) else None
        )

        try:
            follower_index = out.name.index(self._follower_joint)
        except ValueError:
            out.name.append(self._follower_joint)
            out.position.append(sim_position)
            if msg.velocity and sim_velocity is not None:
                out.velocity.append(sim_velocity)
            if msg.effort:
                out.effort.append(0.0)
        else:
            out.position[follower_index] = sim_position
            if follower_index < len(out.velocity) and sim_velocity is not None:
                out.velocity[follower_index] = sim_velocity

        self._publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = Pzs100RvizJointStateAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
