#!/usr/bin/env python3
"""Publish the active SLAM mapper's occupancy in the laptop RViz frame."""

import copy
import math
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from surf_multirobot_msgs.msg import VoxelDelta


class SlamMapVisualizer(Node):
    def __init__(self):
        super().__init__('slam_map_visualizer')
        source_type = self.declare_parameter('source_type', 'pointcloud').value
        input_topic = self.declare_parameter('input_topic', '').value
        output_topic = self.declare_parameter(
            'output_topic', '/slam/occupied_voxels').value
        self.output_frame = self.declare_parameter('output_frame', 'map').value
        if source_type not in ('pointcloud', 'voxel_delta'):
            raise ValueError("source_type must be 'pointcloud' or 'voxel_delta'")
        if not input_topic:
            raise ValueError('input_topic must not be empty')

        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(PointCloud2, output_topic, output_qos)
        self.voxels = {}
        self.map_epoch = None
        self.version = 0
        self.observation_times = {}
        self.retired_epochs = set()

        if source_type == 'pointcloud':
            self.subscription = self.create_subscription(
                PointCloud2, input_topic, self._cloud_callback, 10)
        else:
            delta_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=128,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.subscription = self.create_subscription(
                VoxelDelta, input_topic, self._delta_callback, delta_qos)
        self.get_logger().info(
            f'Publishing {source_type} SLAM visualization from {input_topic} '
            f'to {output_topic} in visualization frame {self.output_frame}')

    def _cloud_callback(self, message):
        output = copy.deepcopy(message)
        output.header.frame_id = self.output_frame
        self.publisher.publish(output)

    def _delta_callback(self, message):
        information = message.operating_mode == 4
        resolution = float(message.resolution)
        if not math.isfinite(resolution) or resolution <= 0:
            return
        count = len(message.x)
        if any(len(v) != count for v in (message.y, message.z, message.state,
                                         message.observation_time_ns)):
            return
        if message.map_epoch in self.retired_epochs:
            return
        if self.map_epoch != message.map_epoch:
            if self.map_epoch is not None:
                self.retired_epochs.add(self.map_epoch)
            self.voxels.clear()
            self.observation_times.clear()
            self.map_epoch = message.map_epoch
            self.version = 0
        if not information and (message.version < self.version or (
                message.version == self.version and not message.full_refresh)):
            return
        if not information and message.full_refresh:
            self.voxels.clear()
            self.observation_times.clear()

        for x, y, z, state, stamp in zip(message.x, message.y, message.z,
                                          message.state, message.observation_time_ns):
            key = (x, y, z)
            if key in self.observation_times and stamp <= self.observation_times[key]:
                continue
            self.observation_times[key] = stamp  # Retain deletion timestamps too.
            if state in (
                    VoxelDelta.STATE_OCCUPIED_STATIC,
                    VoxelDelta.STATE_OCCUPIED_DYNAMIC):
                self.voxels[key] = resolution
            elif state in (VoxelDelta.STATE_FREE, VoxelDelta.STATE_DELETE):
                self.voxels.pop(key, None)
        self.version = max(self.version, message.version)
        self._publish_voxels(message.header.stamp)

    def _publish_voxels(self, stamp):
        output = PointCloud2()
        output.header.stamp = stamp
        output.header.frame_id = self.output_frame
        output.height = 1
        output.width = len(self.voxels)
        output.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        output.is_bigendian = False
        output.point_step = 12
        output.row_step = output.point_step * output.width
        output.is_dense = True
        output.data = b''.join(
            struct.pack('<fff', x * resolution, y * resolution, z * resolution)
            for (x, y, z), resolution in self.voxels.items())
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = SlamMapVisualizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
