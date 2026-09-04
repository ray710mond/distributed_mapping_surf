#!/usr/bin/env python3
"""Report how much of the live FAST-LIO cloud overlaps a reference PCD."""

import json
import math
import struct
from pathlib import Path

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


_FORMATS = {
    PointField.INT8: 'b', PointField.UINT8: 'B',
    PointField.INT16: 'h', PointField.UINT16: 'H',
    PointField.INT32: 'i', PointField.UINT32: 'I',
    PointField.FLOAT32: 'f', PointField.FLOAT64: 'd',
}


def _pcd_header(stream):
    header = {}
    while True:
        line = stream.readline()
        if not line:
            raise ValueError('PCD has no DATA line')
        decoded = line.decode('ascii').strip()
        if not decoded or decoded.startswith('#'):
            continue
        key, *values = decoded.split()
        header[key.upper()] = values
        if key.upper() == 'DATA':
            return header


def load_pcd_xyz(path):
    """Load XYZ from the ASCII or uncompressed binary PCD written by PCL."""
    with Path(path).open('rb') as stream:
        header = _pcd_header(stream)
        fields = header['FIELDS']
        sizes = [int(value) for value in header['SIZE']]
        types = header['TYPE']
        counts = [int(value) for value in header.get('COUNT', ['1'] * len(fields))]
        points = int(header.get('POINTS', ['0'])[0])
        offsets = []
        offset = 0
        for size, count in zip(sizes, counts):
            offsets.append(offset)
            offset += size * count
        xyz = [fields.index(axis) for axis in ('x', 'y', 'z')]
        data_type = header['DATA'][0].lower()
        if data_type == 'ascii':
            result = []
            for line in stream:
                values = line.split()
                if values:
                    result.append(tuple(float(values[index]) for index in xyz))
            return result
        if data_type != 'binary':
            raise ValueError(f'unsupported PCD DATA encoding: {data_type}')
        payload = stream.read(points * offset)
        result = []
        for point_index in range(points):
            base = point_index * offset
            point = []
            for field_index in xyz:
                code = types[field_index]
                size = sizes[field_index]
                fmt = {('F', 4): 'f', ('F', 8): 'd', ('I', 4): 'i',
                       ('U', 4): 'I'}[(code, size)]
                point.append(struct.unpack_from('<' + fmt, payload,
                                                base + offsets[field_index])[0])
            result.append(tuple(point))
        return result


def _rotate(q, point):
    x, y, z = point
    qx, qy, qz, qw = q
    # Quaternion-vector rotation, expanded to avoid another dependency.
    tx, ty, tz = (2.0 * (qy*z - qz*y), 2.0 * (qz*x - qx*z),
                  2.0 * (qx*y - qy*x))
    return (x + qw*tx + qy*tz - qz*ty,
            y + qw*ty + qz*tx - qx*tz,
            z + qw*tz + qx*ty - qy*tx)


class LocalizationOverlapMonitor(Node):
    def __init__(self):
        super().__init__('localization_overlap_monitor')
        self.pcd = self.declare_parameter('reference_pcd', '').value
        cloud_topic = self.declare_parameter('cloud_topic', 'points').value
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.body_frame = self.declare_parameter('body_frame', 'body').value
        self.bounds_margin = float(self.declare_parameter(
            'bounds_margin', 0.0).value)
        status_topic = self.declare_parameter(
            'status_topic', 'localization/status').value
        reference_path_topic = self.declare_parameter(
            'reference_path_topic', 'localization/reference_path').value
        self.resolution = max(0.05, float(self.declare_parameter(
            'overlap_resolution', 0.5).value))
        self.sample_limit = max(100, int(self.declare_parameter(
            'sample_limit', 4000).value))
        self.anchor_threshold = float(self.declare_parameter(
            'anchor_threshold', 0.55).value)
        self.lost_threshold = float(self.declare_parameter(
            'lost_threshold', 0.20).value)
        self.required_updates = max(1, int(self.declare_parameter(
            'required_updates', 3).value))
        self.reference_voxels = set()
        self.reference_points = 0
        self.reference_bounds = None
        self.inside_bounds = False
        self.state = 'NO_REFERENCE'
        self.good_updates = 0
        self.bad_updates = 0
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(String, status_topic, 10)

        self.try_load_reference()

        self.subscription = self.create_subscription(
            PointCloud2, cloud_topic, self.cloud_callback,
            rclpy.qos.qos_profile_sensor_data)
        reference_qos = rclpy.qos.QoSProfile(
            depth=1,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
        self.reference_path_subscription = self.create_subscription(
            String, reference_path_topic, self.reference_path_callback,
            reference_qos)
        self.timer = self.create_timer(1.0, self.publish_idle_status)

    def reference_path_callback(self, message):
        if not message.data or message.data == self.pcd:
            return
        self.pcd = message.data
        self.reference_voxels.clear()
        self.reference_points = 0
        self.reference_bounds = None
        self.inside_bounds = False
        self.state = 'NO_REFERENCE'
        self.good_updates = self.bad_updates = 0
        self.try_load_reference()

    def try_load_reference(self):
        if self.reference_voxels or not self.pcd or not Path(self.pcd).is_file():
            return
        try:
            points = load_pcd_xyz(self.pcd)
            self.reference_points = len(points)
            self.reference_bounds = (
                min(point[0] for point in points),
                max(point[0] for point in points),
                min(point[1] for point in points),
                max(point[1] for point in points))
            self.reference_voxels = {self._voxel(point) for point in points
                                     if all(math.isfinite(v) for v in point)}
            self.state = 'WAITING_FOR_TRANSFORM'
            self.get_logger().info(
                f'Loaded {self.reference_points} reference points '
                f'({len(self.reference_voxels)} overlap voxels) from {self.pcd}')
        except (OSError, KeyError, ValueError, struct.error) as error:
            self.state = 'REFERENCE_ERROR'
            self.get_logger().error(
                f'Cannot load reference PCD {self.pcd}: {error}')

    def _voxel(self, point):
        return tuple(math.floor(value / self.resolution) for value in point)

    def publish(self, overlap=None, sampled=0, detail=''):
        message = String()
        message.data = json.dumps({
            'state': self.state,
            'reference_available': bool(self.reference_voxels),
            'reference_points': self.reference_points,
            'sampled_points': sampled,
            'overlap': overlap,
            'inside_reference_bounds': self.inside_bounds,
            'detail': detail,
        }, separators=(',', ':'))
        self.publisher.publish(message)

    def publish_idle_status(self):
        self.try_load_reference()
        if not self.reference_voxels:
            self.publish(detail='GLIM mapping; no fixed reference PCD')

    def cloud_callback(self, message):
        if not self.reference_voxels:
            return
        try:
            transform = self.buffer.lookup_transform(
                self.map_frame, message.header.frame_id,
                rclpy.time.Time.from_msg(message.header.stamp),
                Duration(seconds=0.05))
            body_transform = self.buffer.lookup_transform(
                self.map_frame, self.body_frame,
                rclpy.time.Time.from_msg(message.header.stamp),
                Duration(seconds=0.05))
        except TransformException as error:
            self.state = 'WAITING_FOR_TRANSFORM'
            self.publish(detail=str(error))
            return

        min_x, max_x, min_y, max_y = self.reference_bounds
        position = body_transform.transform.translation
        self.inside_bounds = (
            min_x-self.bounds_margin <= position.x <= max_x+self.bounds_margin and
            min_y-self.bounds_margin <= position.y <= max_y+self.bounds_margin)

        fields = {field.name: field for field in message.fields}
        if not all(axis in fields for axis in ('x', 'y', 'z')):
            self.state = 'CLOUD_ERROR'
            self.publish(detail='cloud has no x/y/z fields')
            return
        total = message.width * message.height
        stride = max(1, math.ceil(total / self.sample_limit))
        endian = '>' if message.is_bigendian else '<'
        sampled = matched = 0
        tf = transform.transform
        q = (tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w)
        translation = (tf.translation.x, tf.translation.y, tf.translation.z)
        neighbor = (-1, 0, 1)
        for index in range(0, total, stride):
            row, column = divmod(index, message.width)
            base = row * message.row_step + column * message.point_step
            point = []
            for axis in ('x', 'y', 'z'):
                field = fields[axis]
                point.append(struct.unpack_from(
                    endian + _FORMATS[field.datatype], message.data,
                    base + field.offset)[0])
            if not all(math.isfinite(value) for value in point):
                continue
            rotated = _rotate(q, point)
            voxel = self._voxel(tuple(rotated[i] + translation[i]
                                      for i in range(3)))
            sampled += 1
            if any((voxel[0] + dx, voxel[1] + dy, voxel[2] + dz)
                   in self.reference_voxels
                   for dx in neighbor for dy in neighbor for dz in neighbor):
                matched += 1
        overlap = matched / sampled if sampled else 0.0
        if overlap >= self.anchor_threshold:
            self.good_updates += 1
            self.bad_updates = 0
        elif overlap <= self.lost_threshold:
            self.bad_updates += 1
            self.good_updates = 0
        else:
            self.good_updates = self.bad_updates = 0
            self.state = 'TRANSITION'
        if self.good_updates >= self.required_updates:
            self.state = 'ANCHORED'
        elif self.bad_updates >= self.required_updates:
            self.state = 'DEAD_RECKONING'
        self.publish(overlap=round(overlap, 4), sampled=sampled)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationOverlapMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
