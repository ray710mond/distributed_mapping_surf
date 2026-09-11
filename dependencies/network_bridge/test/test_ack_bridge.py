"""Verify burst ACK IDs through the deployed UDP and TCP bridge paths."""
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import rclpy
from surf_multirobot_msgs.msg import RealtimeAck, SyncAck


def bridge(name, interface, parameters):
    return launch_ros.actions.Node(
        package='network_bridge', executable='network_bridge', name=name,
        parameters=[{
            'network_interface': interface,
            'subscribe_namespace': '/ack_test/in',
            'publish_namespace': '/ack_test/out',
            'default_rate': 100.0,
            'publish_stale_data': False,
            **parameters,
        }], output='screen')


def generate_test_description():
    udp_sender = bridge('ack_udp_sender', 'network_bridge::UdpInterface', {
        'UdpInterface.local_address': '127.0.0.1',
        'UdpInterface.remote_address': '127.0.0.1',
        'UdpInterface.receive_port': 39201, 'UdpInterface.send_port': 39202,
        'topics': ['/realtime_ack'],
        '/realtime_ack.queue_depth': 256, '/realtime_ack.queue_bytes': 65536,
    })
    udp_receiver = bridge('ack_udp_receiver', 'network_bridge::UdpInterface', {
        'UdpInterface.local_address': '127.0.0.1',
        'UdpInterface.remote_address': '127.0.0.1',
        'UdpInterface.receive_port': 39202, 'UdpInterface.send_port': 39201,
    })
    tcp_server = bridge('ack_tcp_server', 'network_bridge::TcpInterface', {
        'TcpInterface.role': 'server', 'TcpInterface.port': 39203,
        'topics': ['/sync_ack'],
        '/sync_ack.queue_depth': 256, '/sync_ack.queue_bytes': 65536,
    })
    tcp_client = bridge('ack_tcp_client', 'network_bridge::TcpInterface', {
        'TcpInterface.role': 'client', 'TcpInterface.remote_address': '127.0.0.1',
        'TcpInterface.port': 39203,
    })
    return launch.LaunchDescription([
        udp_sender, udp_receiver, tcp_server,
        launch.actions.TimerAction(period=0.1, actions=[tcp_client]),
        launch_testing.actions.ReadyToTest(),
    ])


class AckBridgeTest(unittest.TestCase):
    def test_each_burst_id_crosses_bridge(self):
        rclpy.init()
        node = rclpy.create_node('ack_bridge_test')
        realtime_received = []
        sync_received = []
        realtime_publisher = node.create_publisher(
            RealtimeAck, '/ack_test/in/realtime_ack', 256)
        sync_publisher = node.create_publisher(SyncAck, '/ack_test/in/sync_ack', 256)
        realtime_subscription = node.create_subscription(
            RealtimeAck, '/ack_test/out/realtime_ack',
            lambda message: realtime_received.append(message.version), 256)
        sync_subscription = node.create_subscription(
            SyncAck, '/ack_test/out/sync_ack',
            lambda message: sync_received.append(message.version), 256)
        try:
            deadline = time.monotonic() + 10
            while (realtime_publisher.get_subscription_count() == 0 or
                   sync_publisher.get_subscription_count() == 0) and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            self.assertGreater(realtime_publisher.get_subscription_count(), 0)
            self.assertGreater(sync_publisher.get_subscription_count(), 0)

            # Warm up the lazily-created bridge output publishers.
            while (0 not in realtime_received or 0 not in sync_received) and time.monotonic() < deadline:
                realtime_publisher.publish(RealtimeAck(map_epoch=7, version=0))
                sync_publisher.publish(SyncAck(source_id='drone', map_epoch=7, version=0))
                rclpy.spin_once(node, timeout_sec=0.05)

            expected = list(range(1, 33))
            for version in expected:
                realtime_publisher.publish(RealtimeAck(map_epoch=7, version=version))
                sync_publisher.publish(
                    SyncAck(source_id='drone', map_epoch=7, version=version))
            deadline = time.monotonic() + 10
            while ((len([v for v in realtime_received if v]) < len(expected) or
                    len([v for v in sync_received if v]) < len(expected)) and
                   time.monotonic() < deadline):
                rclpy.spin_once(node, timeout_sec=0.01)
            self.assertEqual([v for v in realtime_received if v], expected)
            self.assertEqual([v for v in sync_received if v], expected)
        finally:
            node.destroy_subscription(realtime_subscription)
            node.destroy_subscription(sync_subscription)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    launch_testing.main()
