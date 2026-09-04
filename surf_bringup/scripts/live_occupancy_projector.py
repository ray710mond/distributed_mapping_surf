#!/usr/bin/env python3
"""Fuse a trusted reference grid and live Bonxai voxels into an expanding map."""

import math

from bonxai_msgs.srv import GetFreeVoxels, GetOccupiedVoxels
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


MAP_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL)


class LiveOccupancyProjector(Node):
    def __init__(self):
        super().__init__('live_occupancy_projector')
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.resolution = max(0.02, float(
            self.declare_parameter('resolution', 0.20).value))
        self.min_z = float(self.declare_parameter('min_z', 0.25).value)
        self.max_z = float(self.declare_parameter('max_z', 2.0).value)
        self.max_cells = max(10000, int(
            self.declare_parameter('max_cells', 16000000).value))
        period = max(0.2, float(
            self.declare_parameter('update_period_s', 1.0).value))
        reference_topic = self.declare_parameter(
            'reference_topic', '/map/reference').value
        output_topic = self.declare_parameter('output_topic', '/map').value
        self.reference = {}
        self.live_occupied = set()
        self.live_free = set()
        self.query_pending = False
        self.occupied_client = self.create_client(
            GetOccupiedVoxels, 'bonxai/get_occupied_voxels')
        self.free_client = self.create_client(
            GetFreeVoxels, 'bonxai/get_free_voxels')
        self.publisher = self.create_publisher(OccupancyGrid, output_topic, MAP_QOS)
        self.reference_subscription = self.create_subscription(
            OccupancyGrid, reference_topic, self.reference_callback, MAP_QOS)
        self.timer = self.create_timer(period, self.update)

    def cell(self, x, y):
        return (math.floor(x / self.resolution), math.floor(y / self.resolution))

    def reference_callback(self, message):
        q = message.info.origin.orientation
        yaw = math.atan2(2.0 * (q.w*q.z + q.x*q.y),
                         1.0 - 2.0 * (q.y*q.y + q.z*q.z))
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        source_resolution = message.info.resolution
        origin = message.info.origin.position
        cells = {}
        for row in range(message.info.height):
            for column in range(message.info.width):
                value = message.data[row * message.info.width + column]
                if value < 0:
                    continue
                local_x = (column + 0.5) * source_resolution
                local_y = (row + 0.5) * source_resolution
                world_x = origin.x + cos_yaw*local_x - sin_yaw*local_y
                world_y = origin.y + sin_yaw*local_x + cos_yaw*local_y
                cells[self.cell(world_x, world_y)] = int(value)
        self.reference = cells
        self.get_logger().info(
            f'Loaded {len(cells)} known cells from the trusted reference grid')
        self.publish_map()

    def update(self):
        if self.query_pending or not self.occupied_client.service_is_ready():
            return
        self.query_pending = True
        future = self.occupied_client.call_async(GetOccupiedVoxels.Request())
        future.add_done_callback(self.occupied_received)

    def projected_cells(self, grid):
        cells = set()
        for x, y, z in zip(grid.x, grid.y, grid.z):
            world_z = z * grid.resolution
            if self.min_z <= world_z <= self.max_z:
                cells.add(self.cell(
                    x * grid.resolution, y * grid.resolution))
        return cells

    def occupied_received(self, future):
        try:
            response = future.result()
            if response.success:
                self.live_occupied = self.projected_cells(response.voxel_grid)
        except Exception as error:
            self.get_logger().warning(
                f'Failed to query occupied voxels: {error}',
                throttle_duration_sec=5.0)
            self.query_pending = False
            return
        if not self.free_client.service_is_ready():
            self.query_pending = False
            self.publish_map()
            return
        future = self.free_client.call_async(GetFreeVoxels.Request())
        future.add_done_callback(self.free_received)

    def free_received(self, future):
        try:
            response = future.result()
            if response.success:
                self.live_free = self.projected_cells(response.voxel_grid)
        except Exception as error:
            self.get_logger().warning(
                f'Failed to query free voxels: {error}',
                throttle_duration_sec=5.0)
        self.query_pending = False
        self.publish_map()

    def publish_map(self):
        all_cells = set(self.reference) | self.live_free | self.live_occupied
        if not all_cells:
            return
        min_x = min(cell[0] for cell in all_cells)
        max_x = max(cell[0] for cell in all_cells)
        min_y = min(cell[1] for cell in all_cells)
        max_y = max(cell[1] for cell in all_cells)
        width, height = max_x - min_x + 1, max_y - min_y + 1
        if width * height > self.max_cells:
            self.get_logger().error(
                f'Refusing {width}x{height} grid; max_cells={self.max_cells}',
                throttle_duration_sec=5.0)
            return
        data = [-1] * (width * height)

        def assign(cell, value):
            data[(cell[1] - min_y) * width + cell[0] - min_x] = value

        # Live free space fills unknown space, but cannot erase trusted or
        # currently occupied evidence.
        for cell in self.live_free:
            assign(cell, 0)
        for cell, value in self.reference.items():
            assign(cell, value)
        for cell in self.live_occupied:
            assign(cell, 100)

        message = OccupancyGrid()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.map_frame
        message.info.map_load_time = message.header.stamp
        message.info.resolution = self.resolution
        message.info.width = width
        message.info.height = height
        message.info.origin.position.x = min_x * self.resolution
        message.info.origin.position.y = min_y * self.resolution
        message.info.origin.orientation.w = 1.0
        message.data = data
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = LiveOccupancyProjector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
