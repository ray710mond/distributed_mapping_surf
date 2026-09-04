#!/usr/bin/env python3
"""Summarize timing and motion quality in a recorded MID360 ROS bag."""

import argparse
import math
import statistics
import time
from collections import Counter

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


IMU_TOPIC = '/drone/livox/imu'
LIDAR_TOPIC = '/drone/livox/lidar'


def stamp_seconds(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def percentile(values, fraction):
    if not values:
        return float('nan')
    ordered = sorted(values)
    index = round((len(ordered) - 1) * fraction)
    return ordered[index]


def describe(label, values, unit=''):
    suffix = f' {unit}' if unit else ''
    print(
        f'{label}: n={len(values)} min={min(values):.6g}{suffix} '
        f'p50={statistics.median(values):.6g}{suffix} '
        f'p99={percentile(values, 0.99):.6g}{suffix} '
        f'max={max(values):.6g}{suffix}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag', help='Bag directory, e.g. /data/bags/mid360_motion_test')
    parser.add_argument('--events', type=int, default=20,
                        help='Number of strongest motion events to print')
    parser.add_argument('--lidar-stride', type=int, default=10,
                        help='Deserialize every Nth LiDAR scan (default: 10)')
    args = parser.parse_args()
    if args.lidar_stride < 1:
        parser.error('--lidar-stride must be at least 1')

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=args.bag, storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''))
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    missing = {IMU_TOPIC, LIDAR_TOPIC} - topic_types.keys()
    if missing:
        parser.error(f'missing required topics: {sorted(missing)}')
    message_types = {
        topic: get_message(topic_types[topic]) for topic in (IMU_TOPIC, LIDAR_TOPIC)
    }

    imu_stamps = []
    imu_bag_delays = []
    accel_norms = []
    gyro_norms = []
    motion_events = []
    lidar_stamps = []
    lidar_bag_delays = []
    lidar_counts = []
    scan_durations = []
    last_offset_lags = []
    nonmonotonic_offsets = 0
    line_counts = Counter()
    tag_counts = Counter()
    lidar_seen = 0
    messages_seen = 0
    started = time.monotonic()

    while reader.has_next():
        topic, data, bag_stamp_ns = reader.read_next()
        if topic not in message_types:
            continue
        messages_seen += 1
        if topic == LIDAR_TOPIC:
            lidar_seen += 1
            if (lidar_seen - 1) % args.lidar_stride:
                continue
        msg = deserialize_message(data, message_types[topic])
        header_stamp = stamp_seconds(msg.header)
        bag_stamp = bag_stamp_ns * 1e-9
        if topic == IMU_TOPIC:
            a = msg.linear_acceleration
            g = msg.angular_velocity
            accel = math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z)
            gyro = math.sqrt(g.x * g.x + g.y * g.y + g.z * g.z)
            imu_stamps.append(header_stamp)
            imu_bag_delays.append(bag_stamp - header_stamp)
            accel_norms.append(accel)
            gyro_norms.append(gyro)
            motion_events.append((header_stamp, accel, gyro))
        else:
            points = msg.points
            lidar_stamps.append(header_stamp)
            lidar_bag_delays.append(bag_stamp - header_stamp)
            lidar_counts.append(len(points))
            offsets = [point.offset_time for point in points]
            if offsets:
                scan_durations.append((max(offsets) - min(offsets)) * 1e-9)
                last_offset_lags.append((max(offsets) - offsets[-1]) * 1e-9)
                nonmonotonic_offsets += sum(
                    current < previous
                    for previous, current in zip(offsets, offsets[1:]))
            line_counts.update(point.line for point in points)
            tag_counts.update(point.tag for point in points)
        if messages_seen % 2000 == 0:
            print(
                f'Processed {messages_seen} input messages '
                f'({lidar_seen} LiDAR seen) in {time.monotonic() - started:.1f}s',
                flush=True)

    imu_deltas = [b - a for a, b in zip(imu_stamps, imu_stamps[1:])]
    lidar_deltas = [b - a for a, b in zip(lidar_stamps, lidar_stamps[1:])]

    print('MID360 bag diagnostics')
    print(
        f'LiDAR sampling: {len(lidar_counts)}/{lidar_seen} scans '
        f'(stride {args.lidar_stride})')
    describe('IMU period', imu_deltas, 's')
    print(f'IMU non-positive deltas: {sum(delta <= 0 for delta in imu_deltas)}')
    print(f'IMU gaps >10 ms: {sum(delta > 0.010 for delta in imu_deltas)}')
    nominal_accel = 1.0 if statistics.median(accel_norms) < 3.0 else 9.80665
    accel_unit = 'g' if nominal_accel == 1.0 else 'm/s^2'
    describe('IMU acceleration norm', accel_norms, accel_unit)
    describe('IMU angular-rate norm', gyro_norms, 'rad/s')
    describe('IMU bag-minus-header time', imu_bag_delays, 's')
    describe(f'LiDAR sampled period (stride {args.lidar_stride})', lidar_deltas, 's')
    print(f'LiDAR non-positive deltas: {sum(delta <= 0 for delta in lidar_deltas)}')
    describe('LiDAR point count', lidar_counts)
    describe('LiDAR per-point span', scan_durations, 's')
    describe('LiDAR max-offset minus last-offset', last_offset_lags, 's')
    print(
        'Scans whose last point is not latest: '
        f'{sum(lag > 0 for lag in last_offset_lags)}/{len(last_offset_lags)}')
    describe('LiDAR bag-minus-header time', lidar_bag_delays, 's')
    print(f'Nonmonotonic adjacent point offsets: {nonmonotonic_offsets}')
    print(f'Point lines: {dict(sorted(line_counts.items()))}')
    print(f'Point tags: {dict(sorted(tag_counts.items()))}')

    origin = min(imu_stamps[0], lidar_stamps[0])
    print(f'Top {args.events} IMU motion events (seconds from bag start):')
    ranked_events = sorted(
        motion_events,
        key=lambda event: max(abs(event[1] - nominal_accel), event[2]),
        reverse=True)
    for stamp, accel, gyro in ranked_events[:args.events]:
        print(
            f'  t={stamp - origin:9.3f}s accel={accel:10.4f} {accel_unit} '
            f'gyro={gyro:9.4f} rad/s')


if __name__ == '__main__':
    main()
