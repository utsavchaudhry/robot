#!/usr/bin/env python3
"""
Publish sinusoidal joint commands to /head/joint_commands for motor handler testing.

Usage:
    ros2 run robot_bringup head_sinusoid_test
    ros2 run robot_bringup head_sinusoid_test --ros-args \
        -p yaw_amplitude:=0.5 -p pitch_amplitude:=0.3 -p frequency:=0.25

Parameters:
    yaw_amplitude   - Amplitude in radians for head_yaw   (default 1.0, max 1.5)
    pitch_amplitude - Amplitude in radians for head_pitch  (default 0.5, max 0.8)
    frequency       - Oscillation frequency in Hz           (default 0.5)
    rate            - Publish rate in Hz                    (default 50.0)
    phase_offset    - Phase offset between yaw and pitch    (default pi/2, i.e. Lissajous)
"""

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


class HeadSinusoidTest(Node):

    YAW_LIMIT = 1.5    # rad
    PITCH_LIMIT = 0.8   # rad

    def __init__(self):
        super().__init__('head_sinusoid_test')

        self.declare_parameter('yaw_amplitude', 1.0)
        self.declare_parameter('pitch_amplitude', 0.5)
        self.declare_parameter('frequency', 0.5)
        self.declare_parameter('rate', 50.0)
        self.declare_parameter('phase_offset', math.pi / 2)

        self.yaw_amp = self.get_parameter('yaw_amplitude').value
        self.pitch_amp = self.get_parameter('pitch_amplitude').value
        self.freq = self.get_parameter('frequency').value
        rate = self.get_parameter('rate').value
        self.phase_offset = self.get_parameter('phase_offset').value

        # Clamp amplitudes to hardware limits
        if self.yaw_amp > self.YAW_LIMIT:
            self.get_logger().warn(
                f'yaw_amplitude {self.yaw_amp} exceeds limit {self.YAW_LIMIT}, clamping')
            self.yaw_amp = self.YAW_LIMIT
        if self.pitch_amp > self.PITCH_LIMIT:
            self.get_logger().warn(
                f'pitch_amplitude {self.pitch_amp} exceeds limit {self.PITCH_LIMIT}, clamping')
            self.pitch_amp = self.PITCH_LIMIT

        self.pub = self.create_publisher(JointState, 'head/joint_commands', 10)
        self.start_time = time.monotonic()
        self.timer = self.create_timer(1.0 / rate, self.publish_command)

        self.get_logger().info(
            f'Publishing sinusoid: yaw_amp={self.yaw_amp:.2f} rad, '
            f'pitch_amp={self.pitch_amp:.2f} rad, freq={self.freq:.2f} Hz, '
            f'rate={rate:.0f} Hz, phase_offset={self.phase_offset:.2f} rad')

    def publish_command(self):
        t = time.monotonic() - self.start_time
        omega = 2.0 * math.pi * self.freq

        yaw = self.yaw_amp * math.sin(omega * t)
        pitch = self.pitch_amp * math.sin(omega * t + self.phase_offset)

        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['head_yaw', 'head_pitch']
        msg.position = [yaw, pitch]

        self.pub.publish(msg)

        # Log at ~1 Hz to avoid spamming
        if int(t * 1000) % 1000 < int(1000 / 50):
            self.get_logger().info(f't={t:6.1f}s  yaw={yaw:+.3f}  pitch={pitch:+.3f}')


def main(args=None):
    rclpy.init(args=args)
    node = HeadSinusoidTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Stopped.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
