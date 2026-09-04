# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026 California Institute of Technology and Will Compton
"""Convert Livox CustomMsg scans to the PointCloud2 format GLIM consumes."""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField

from livox_ros_driver2.msg import CustomMsg


POINT_DTYPE = np.dtype([
    ('x', np.float32),
    ('y', np.float32),
    ('z', np.float32),
    ('intensity', np.float32),
    ('time', np.float32),
])


class LivoxCustomToPointCloud2(Node):
    def __init__(self):
        super().__init__('livox_custom_to_pc2')
        self.declare_parameter('input_topic', '/livox/lidar')
        self.declare_parameter('output_topic', '/livox/points')
        self.declare_parameter('frame_id', '')
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.fields = [
            PointField(name=name, offset=offset,
                       datatype=PointField.FLOAT32, count=1)
            for name, offset in (
                ('x', 0), ('y', 4), ('z', 8),
                ('intensity', 12), ('time', 16))
        ]
        self.publisher = self.create_publisher(
            PointCloud2, output_topic, qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            CustomMsg, input_topic, self._convert, qos_profile_sensor_data)
        self.get_logger().info(
            f'relaying {input_topic} -> {output_topic}')

    def _convert(self, message):
        count = message.point_num
        if count == 0:
            return
        points = message.points
        array = np.empty(count, dtype=POINT_DTYPE)
        array['x'] = [point.x for point in points]
        array['y'] = [point.y for point in points]
        array['z'] = [point.z for point in points]
        array['intensity'] = [point.reflectivity for point in points]
        array['time'] = np.fromiter(
            (point.offset_time for point in points),
            dtype=np.float32, count=count) * 1e-9

        cloud = PointCloud2()
        cloud.header = message.header
        if self.frame_id:
            cloud.header.frame_id = self.frame_id
        cloud.height = 1
        cloud.width = count
        cloud.is_dense = True
        cloud.is_bigendian = False
        cloud.fields = self.fields
        cloud.point_step = POINT_DTYPE.itemsize
        cloud.row_step = cloud.point_step * count
        cloud.data = array.tobytes()
        self.publisher.publish(cloud)


def main():
    rclpy.init()
    node = LivoxCustomToPointCloud2()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
