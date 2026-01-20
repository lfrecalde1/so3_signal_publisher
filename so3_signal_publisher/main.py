#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from quadrotor_msgs.msg import SO3Command


class SO3SignalPublisher(Node):
    def __init__(self):
        super().__init__('so3_signal_publisher')

        # Parameters
        self.declare_parameter('topic', '/eagle4/so3_cmd_2')
        self.declare_parameter('rate_hz', 100.0)

        # Thrust around 9.8 (units depend on your stack; you asked "around 9.8")
        self.declare_parameter('thrust_mean', 9.8)
        self.declare_parameter('thrust_amp', 0.5)
        self.declare_parameter('thrust_freq_hz', 1.5)

        # Angular velocity sinusoid params (rad/s)
        self.declare_parameter('w_amp_x', 1.3)
        self.declare_parameter('w_amp_y', 1.3)
        self.declare_parameter('w_amp_z', 1.3)
        self.declare_parameter('w_freq_hz', 1.5)

        # Optional: yaw oscillation (kept OFF by default for minimal risk)
        self.declare_parameter('yaw_enable', False)
        self.declare_parameter('yaw_amp_rad', 0.35)      # ~20 deg
        self.declare_parameter('yaw_freq_hz', 0.2)

        topic = self.get_parameter('topic').value
        rate_hz = float(self.get_parameter('rate_hz').value)

        self.pub = self.create_publisher(SO3Command, topic, 10)

        self.t = 0.0
        self.dt = 1.0 / max(1e-6, rate_hz)

        self.timer = self.create_timer(self.dt, self._on_timer)

        self.get_logger().info(
            f"Publishing SO3Command on '{topic}' at {rate_hz:.1f} Hz "
            f"(thrust ~ {self.get_parameter('thrust_mean').value})."
        )

    @staticmethod
    def _quat_from_yaw(yaw: float):
        # Quaternion (w, x, y, z) for yaw rotation about +Z
        half = 0.5 * yaw
        return (math.cos(half), 0.0, 0.0, math.sin(half))

    def _on_timer(self):
        # Read params each tick (cheap enough; lets you tune live with ros2 param set)
        thrust_mean = float(self.get_parameter('thrust_mean').value)
        thrust_amp = float(self.get_parameter('thrust_amp').value)
        thrust_freq = float(self.get_parameter('thrust_freq_hz').value)

        w_amp_x = float(self.get_parameter('w_amp_x').value)
        w_amp_y = float(self.get_parameter('w_amp_y').value)
        w_amp_z = float(self.get_parameter('w_amp_z').value)
        w_freq = float(self.get_parameter('w_freq_hz').value)

        yaw_enable = bool(self.get_parameter('yaw_enable').value)
        yaw_amp = float(self.get_parameter('yaw_amp_rad').value)
        yaw_freq = float(self.get_parameter('yaw_freq_hz').value)

        # Signals
        thrust = thrust_mean + thrust_amp * math.sin(2.0 * math.pi * thrust_freq * self.t)

        wx = w_amp_x * math.sin(2.0 * math.pi * w_freq * self.t)
        wy = w_amp_y * math.sin(2.0 * math.pi * w_freq * self.t + 2.0 * math.pi / 3.0)
        wz = w_amp_z * math.sin(2.0 * math.pi * w_freq * self.t + 4.0 * math.pi / 3.0)

        # Orientation
        if yaw_enable:
            yaw = yaw_amp * math.sin(2.0 * math.pi * yaw_freq * self.t)
            qw, qx, qy, qz = self._quat_from_yaw(yaw)
        else:
            qw, qx, qy, qz = (1.0, 0.0, 0.0, 0.0)

        # Build message
        msg = SO3Command()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"

        msg.force.x = 0.0
        msg.force.y = 0.0
        msg.force.z = float(thrust)

        msg.orientation.w = float(qw)
        msg.orientation.x = float(qx)
        msg.orientation.y = float(qy)
        msg.orientation.z = float(qz)

        msg.angular_velocity.x = float(wx)
        msg.angular_velocity.y = float(wy)
        msg.angular_velocity.z = float(wz)

        # Gains: populate if your downstream expects them (safe defaults)
        msg.kx = [0.0, 0.0, 0.0]
        msg.kv = [0.0, 0.0, 0.0]
        msg.kr = [0.0, 0.0, 0.0]
        msg.kom = [0.0, 0.0, 0.0]

        # aux: leave default-constructed (do not touch unless required)
        self.pub.publish(msg)

        self.t += self.dt


def main():
    rclpy.init()
    node = SO3SignalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
