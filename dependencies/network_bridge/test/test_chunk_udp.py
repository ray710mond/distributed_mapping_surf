"""Exercise FIFO buffering through real ROS subscriptions and loopback UDP."""
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String


def generate_test_description():
    actions = []
    for name, local, remote, topics in [
        ('chunk_udp_sender', 39101, 39102, ['/chunks']),
        ('chunk_udp_receiver', 39102, 39101, []),
    ]:
        actions.append(launch_ros.actions.Node(
            package='network_bridge', executable='network_bridge', name=name,
            parameters=[{
                'network_interface': 'network_bridge::UdpInterface',
                'UdpInterface.local_address': '127.0.0.1',
                'UdpInterface.remote_address': '127.0.0.1',
                'UdpInterface.receive_port': local,
                'UdpInterface.send_port': remote,
                'subscribe_namespace': '/chunk_test/in',
                'publish_namespace': '/chunk_test/out',
                **({'topics': topics} if topics else {}),
                'default_rate': 100.0,
                '/chunks.queue_depth': 256,
                '/chunks.queue_bytes': 307200,
            }], output='screen'))
    return launch.LaunchDescription(actions + [launch_testing.actions.ReadyToTest()])


class ChunkUDPTest(unittest.TestCase):
    def test_burst_survives_timer_sampling(self):
        rclpy.init()
        node = rclpy.create_node('chunk_udp_test')
        received = []
        publisher = node.create_publisher(String, '/chunk_test/in/chunks', 256)
        qos = QoSProfile(depth=256, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        subscription = node.create_subscription(
            String, '/chunk_test/out/chunks', lambda m: received.append(m.data), qos)
        try:
            deadline = time.monotonic() + 10
            while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            self.assertGreater(publisher.get_subscription_count(), 0)
            # Warm up the bridge's lazily created receiving publisher.
            while 'warmup' not in received and time.monotonic() < deadline:
                publisher.publish(String(data='warmup'))
                rclpy.spin_once(node, timeout_sec=0.05)
            self.assertIn('warmup', received)
            # Publish a burst far faster than the 100 Hz bridge timer. Latest-value
            # forwarding loses most of these even on a perfect loopback network.
            for version in range(30):
                for chunk in range(6):
                    publisher.publish(String(data=f'{version}:{chunk}'))
            deadline = time.monotonic() + 10
            expected = [f'{version}:{chunk}' for version in range(30) for chunk in range(6)]
            while len([m for m in received if m != 'warmup']) < 180 and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.01)
            self.assertEqual([m for m in received if m != 'warmup'], expected)
        finally:
            node.destroy_subscription(subscription)
            node.destroy_node()
            rclpy.shutdown()
