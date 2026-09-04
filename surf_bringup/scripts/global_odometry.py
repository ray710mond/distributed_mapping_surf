#!/usr/bin/env python3
"""Publish local LIO odometry as a pose in the shared map frame."""

import copy
from collections import deque

import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


def multiply(a, b):
    return (
        a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
        a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
        a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
        a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
    )


def rotate(q, v):
    rotated = multiply(multiply(q, (v[0], v[1], v[2], 0.0)),
                       (-q[0], -q[1], -q[2], q[3]))
    return rotated[:3]


class GlobalOdometry(Node):
    def __init__(self):
        super().__init__('global_odometry')
        input_topic = self.declare_parameter('input_topic', 'odom').value
        output_topic = self.declare_parameter(
            'output_topic', 'transport/global_odometry').value
        self.target_frame = self.declare_parameter('target_frame', 'map').value
        self.output_child_frame = self.declare_parameter(
            'output_child_frame', '').value
        self.timeout = Duration(seconds=max(
            0.0, self.declare_parameter('transform_timeout_seconds', 0.0).value))
        self.max_transform_wait = Duration(seconds=max(
            0.0, self.declare_parameter('max_transform_wait_seconds', 0.5).value))
        self.fallback_to_input_frame = self.declare_parameter(
            'fallback_to_input_frame', False).value
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(Odometry, output_topic, 10)
        self.subscription = self.create_subscription(
            Odometry, input_topic, self.callback, 20)
        self.pending = deque(maxlen=200)
        self.retry_timer = self.create_timer(0.01, self.drain_pending)

    def callback(self, message):
        self.pending.append((message, self.get_clock().now()))
        self.drain_pending()

    def drain_pending(self):
        while self.pending:
            message, received = self.pending[0]
            try:
                transform = self.buffer.lookup_transform(
                    self.target_frame, message.header.frame_id,
                    rclpy.time.Time.from_msg(message.header.stamp), self.timeout)
                child_transform = None
                if (self.output_child_frame and message.child_frame_id and
                        self.output_child_frame != message.child_frame_id):
                    child_transform = self.buffer.lookup_transform(
                        message.child_frame_id, self.output_child_frame,
                        rclpy.time.Time.from_msg(message.header.stamp), self.timeout)
            except TransformException as error:
                # ScanLock's map correction commonly trails FAST-LIO by one
                # update. Retain the sample until TF catches up so the output
                # remains aligned to the original odometry timestamp.
                if self.get_clock().now() - received < self.max_transform_wait:
                    return
                self.pending.popleft()
                if self.fallback_to_input_frame:
                    self.publish_fallback(message)
                    continue
                self.get_logger().warning(
                    f'Dropping odometry after waiting for transform: {error}',
                    throttle_duration_sec=5.0)
                continue

            self.pending.popleft()
            self.publish_transformed(message, transform, child_transform)

    def publish_fallback(self, message):
        """Keep local odometry flowing until a requested global TF exists."""
        result = copy.deepcopy(message)
        if self.output_child_frame:
            result.child_frame_id = self.output_child_frame
        self.publisher.publish(result)

    def publish_transformed(self, message, transform, child_transform=None):
        result = copy.deepcopy(message)
        result.header.frame_id = self.target_frame
        tq = transform.transform.rotation
        q = (tq.x, tq.y, tq.z, tq.w)
        p = message.pose.pose.position
        rp = rotate(q, (p.x, p.y, p.z))
        result.pose.pose.position.x = rp[0] + transform.transform.translation.x
        result.pose.pose.position.y = rp[1] + transform.transform.translation.y
        result.pose.pose.position.z = rp[2] + transform.transform.translation.z
        pq = message.pose.pose.orientation
        orientation = multiply(q, (pq.x, pq.y, pq.z, pq.w))
        result.pose.pose.orientation.x = orientation[0]
        result.pose.pose.orientation.y = orientation[1]
        result.pose.pose.orientation.z = orientation[2]
        result.pose.pose.orientation.w = orientation[3]

        if child_transform is not None:
            child_rotation = child_transform.transform.rotation
            child_q = (
                child_rotation.x, child_rotation.y,
                child_rotation.z, child_rotation.w)
            child_translation = child_transform.transform.translation
            child_position = rotate(orientation, (
                child_translation.x,
                child_translation.y,
                child_translation.z))
            result.pose.pose.position.x += child_position[0]
            result.pose.pose.position.y += child_position[1]
            result.pose.pose.position.z += child_position[2]
            orientation = multiply(orientation, child_q)
            result.pose.pose.orientation.x = orientation[0]
            result.pose.pose.orientation.y = orientation[1]
            result.pose.pose.orientation.z = orientation[2]
            result.pose.pose.orientation.w = orientation[3]

        if self.output_child_frame:
            result.child_frame_id = self.output_child_frame
        self.publisher.publish(result)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
