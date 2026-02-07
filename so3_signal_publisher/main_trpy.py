#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from quadrotor_msgs.msg import TRPYCommand


class TRPYSignalPublisher(Node):
    def __init__(self):
        super().__init__('trpy_signal_publisher')

        # Parameters
        self.declare_parameter('topic', '/eagle10/trpy_cmd')
        self.declare_parameter('rate_hz', 200.0)

        # Thrust signal (around 9.8)
        self.declare_parameter('thrust_mean', 0.0)
        self.declare_parameter('thrust_amp', 0.0)
        self.declare_parameter('thrust_freq_hz', 2.5)

        # Euler angles (rad): OFF by default
        self.declare_parameter('rpy_enable', False)
        self.declare_parameter('roll_amp_rad', 0.25)
        self.declare_parameter('pitch_amp_rad', 0.25)
        self.declare_parameter('yaw_amp_rad', 0.35)
        self.declare_parameter('rpy_freq_hz', 0.2)

        # Angular velocity sinusoids (rad/s)
        self.declare_parameter('w_amp_x', 10.5)
        self.declare_parameter('w_amp_y', 15.0)
        self.declare_parameter('w_amp_z', 10.0)
        self.declare_parameter('w_freq_hz', 2.5)

        # Gains (optional)
        self.declare_parameter('kr', [0.0, 0.0, 0.0])
        self.declare_parameter('kom', [0.0, 0.0, 0.0])

        topic = self.get_parameter('topic').value
        rate_hz = float(self.get_parameter('rate_hz').value)

        self.pub = self.create_publisher(TRPYCommand, topic, 10)

        self.t = 0.0
        self.dt = 1.0 / max(1e-6, rate_hz)
        self.timer = self.create_timer(self.dt, self._on_timer)

        self.get_logger().info(f"Publishing TRPYCommand on '{topic}' at {rate_hz:.1f} Hz")

    @staticmethod
    def quat_from_rpy(roll: float, pitch: float, yaw: float):
        """
        Quaternion (w,x,y,z) from intrinsic ZYX (yaw-pitch-roll).
        This is the standard robotics RPY->quat mapping.
        """
        cr = math.cos(0.5 * roll)
        sr = math.sin(0.5 * roll)
        cp = math.cos(0.5 * pitch)
        sp = math.sin(0.5 * pitch)
        cy = math.cos(0.5 * yaw)
        sy = math.sin(0.5 * yaw)

        qw = cy * cp * cr + sy * sp * sr
        qx = cy * cp * sr - sy * sp * cr
        qy = cy * sp * cr + sy * cp * sr
        qz = sy * cp * cr - cy * sp * sr
        return qw, qx, qy, qz

    def _on_timer(self):
        thrust_mean = float(self.get_parameter('thrust_mean').value)
        thrust_amp = float(self.get_parameter('thrust_amp').value)
        thrust_freq = float(self.get_parameter('thrust_freq_hz').value)

        rpy_enable = bool(self.get_parameter('rpy_enable').value)
        roll_amp = float(self.get_parameter('roll_amp_rad').value)
        pitch_amp = float(self.get_parameter('pitch_amp_rad').value)
        yaw_amp = float(self.get_parameter('yaw_amp_rad').value)
        rpy_freq = float(self.get_parameter('rpy_freq_hz').value)

        w_amp_x = float(self.get_parameter('w_amp_x').value)
        w_amp_y = float(self.get_parameter('w_amp_y').value)
        w_amp_z = float(self.get_parameter('w_amp_z').value)
        w_freq = float(self.get_parameter('w_freq_hz').value)

        # Gains (ensure length 3)
        kr = list(self.get_parameter('kr').value)
        kom = list(self.get_parameter('kom').value)
        kr = (kr + [0.0, 0.0, 0.0])[:3]
        kom = (kom + [0.0, 0.0, 0.0])[:3]

        # Signals
        thrust = thrust_mean + thrust_amp * math.sin(2.0 * math.pi * thrust_freq * self.t)

        wx = w_amp_x * math.sin(2.0 * math.pi * w_freq * self.t)
        wy = w_amp_y * math.sin(2.0 * math.pi * w_freq * self.t + 2.0 * math.pi / 3.0)
        wz = w_amp_z * math.sin(2.0 * math.pi * w_freq * self.t + 4.0 * math.pi / 3.0)

        # Euler angles
        if rpy_enable:
            roll = roll_amp * math.sin(2.0 * math.pi * rpy_freq * self.t)
            pitch = pitch_amp * math.sin(2.0 * math.pi * rpy_freq * self.t + 2.0 * math.pi / 3.0)
            yaw = yaw_amp * math.sin(2.0 * math.pi * rpy_freq * self.t + 4.0 * math.pi / 3.0)
        else:
            roll, pitch, yaw = 0.0, 0.0, 0.0

        qw, qx, qy, qz = self.quat_from_rpy(roll, pitch, yaw)

        # Build message with your exact structure
        msg = TRPYCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"

        # Exactly 4 entries
        msg.pred_input = [0.0, 0.0, 0.0, 0.0]

        msg.thrust = float(thrust)
        msg.roll = float(roll)
        msg.pitch = float(pitch)
        msg.yaw = float(yaw)

        msg.angular_velocity.x = float(wx)
        msg.angular_velocity.y = float(wy)
        msg.angular_velocity.z = float(wz)

        msg.quaternion.w = float(qw)
        msg.quaternion.x = float(qx)
        msg.quaternion.y = float(qy)
        msg.quaternion.z = float(qz)

        msg.kr = [float(kr[0]), float(kr[1]), float(kr[2])]
        msg.kom = [float(kom[0]), float(kom[1]), float(kom[2])]

        # aux left default-constructed (do not modify unless you need fields)
        self.pub.publish(msg)

        self.t += self.dt


def main():
    rclpy.init()
    node = TRPYSignalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

