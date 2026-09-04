#!/usr/bin/env python3
"""Fail hardware bringup when the peer's ROS clock is not comparable."""

import json
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


class PeerClockGuard(Node):
    def __init__(self):
        super().__init__('peer_clock_guard')
        topic = self.declare_parameter('peer_odometry_topic', '').value
        localization_topic = self.declare_parameter(
            'localization_status_topic', 'localization/promotion_status').value
        self.maximum_skew_ns = int(
            float(self.declare_parameter('maximum_skew_seconds', 1.0).value) * 1e9)
        self.startup_timeout = float(
            self.declare_parameter('startup_timeout_seconds', 30.0).value)
        self.required_samples = int(
            self.declare_parameter('required_consecutive_samples', 5).value)
        if not topic or self.maximum_skew_ns <= 0 or self.startup_timeout <= 0 or \
                self.required_samples <= 0:
            raise ValueError('Clock-guard parameters must be nonempty and positive')

        self.localized_at = None
        self.good_samples = 0
        self.verified = False
        self.failed = False
        self.create_subscription(Odometry, topic, self._odometry, qos_profile_sensor_data)
        self.create_subscription(String, localization_topic, self._localization, 10)
        self.create_timer(0.25, self._check_timeout)
        self.get_logger().info(
            f'Clock check is disarmed until local localization is active on '
            f'{localization_topic}; it will then wait up to {self.startup_timeout:.1f}s '
            f'for peer clock evidence on {topic}')

    def _fail(self, message):
        if self.failed:
            return
        self.failed = True
        self.get_logger().fatal(message)
        rclpy.shutdown()

    def _odometry(self, message):
        if self.localized_at is None:
            return
        stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        skew_ns = abs(self.get_clock().now().nanoseconds - stamp_ns)
        if skew_ns > self.maximum_skew_ns:
            self._fail(
                f'Peer clock skew is {skew_ns / 1e9:.3f}s, exceeding the '
                f'{self.maximum_skew_ns / 1e9:.3f}s limit. Synchronize both host clocks '
                'before starting SURF.')
            return
        self.good_samples += 1
        if not self.verified and self.good_samples >= self.required_samples:
            self.verified = True
            self.get_logger().info(
                f'Peer clock verified from {self.good_samples} consecutive samples; '
                f'latest apparent skew is {skew_ns / 1e6:.1f}ms')

    def _localization(self, message):
        if self.localized_at is not None:
            return
        try:
            state = json.loads(message.data).get('state')
        except (AttributeError, json.JSONDecodeError):
            return
        if state in ('ACTIVE', 'FASTLIO_GLOBAL'):
            self.localized_at = time.monotonic()
            self.get_logger().info(
                f'Local localization is {state}; starting the '
                f'{self.startup_timeout:.1f}s peer-clock deadline')

    def _check_timeout(self):
        if self.localized_at is not None and not self.verified and \
                time.monotonic() - self.localized_at >= self.startup_timeout:
            self._fail(
                f'No verifiable peer timestamps arrived within {self.startup_timeout:.1f}s; '
                'refusing unsynchronized hardware bringup after localization.')


def main(args=None):
    rclpy.init(args=args)
    node = PeerClockGuard()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
