#!/usr/bin/env python3
"""Report apparent peer odometry age without stopping hardware bringup."""

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
        self.timeout_reported = False
        self.create_subscription(Odometry, topic, self._odometry, qos_profile_sensor_data)
        self.create_subscription(String, localization_topic, self._localization, 10)
        self.create_timer(0.25, self._check_timeout)
        self.get_logger().info(
            f'Peer odometry age monitoring starts when local localization is active on '
            f'{localization_topic}; it will then wait up to {self.startup_timeout:.1f}s '
            f'for timely peer odometry on {topic}')

    def _odometry(self, message):
        if self.localized_at is None:
            return
        stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        now_ns = self.get_clock().now().nanoseconds
        age_ns = now_ns - stamp_ns
        if abs(age_ns) > self.maximum_skew_ns:
            self.good_samples = 0
            direction = 'old' if age_ns > 0 else 'ahead of local time'
            self.get_logger().warning(
                f'Peer odometry timestamp is {abs(age_ns) / 1e9:.3f}s {direction}, '
                f'exceeding the {self.maximum_skew_ns / 1e9:.3f}s limit '
                f'(local time {now_ns / 1e9:.3f}, header stamp {stamp_ns / 1e9:.3f}). '
                'Check both hosts with chronyc tracking/sources, then inspect '
                'odometry publication and bridge latency. This apparent age includes '
                'clock offset, source age, and transport delay.',
                throttle_duration_sec=5.0)
            return
        self.good_samples += 1
        if not self.verified and self.good_samples >= self.required_samples:
            self.verified = True
            self.get_logger().info(
                f'Peer timestamps within limit for {self.good_samples} consecutive samples; '
                f'latest apparent timestamp age is {age_ns / 1e6:.1f}ms')

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
                f'{self.startup_timeout:.1f}s peer-odometry monitoring window')

    def _check_timeout(self):
        if self.localized_at is not None and not self.verified and \
                not self.timeout_reported and \
                time.monotonic() - self.localized_at >= self.startup_timeout:
            self.timeout_reported = True
            self.get_logger().warning(
                f'No peer timestamps within the {self.maximum_skew_ns / 1e9:.3f}s '
                f'apparent-age limit arrived within {self.startup_timeout:.1f}s '
                'after localization. Check clock status and bridge latency.')


def main(args=None):
    rclpy.init(args=args)
    node = PeerClockGuard()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
