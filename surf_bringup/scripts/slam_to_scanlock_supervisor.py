#!/usr/bin/env python3
"""Own initial ScanLock and freeze its global correction at map exit."""

import json
import math
import signal
import subprocess
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import (
    Buffer, TransformBroadcaster, TransformException, TransformListener)


class SlamToScanLockSupervisor(Node):
    def __init__(self):
        super().__init__('slam_to_scanlock_supervisor')
        p = lambda name, default=None: self.declare_parameter(name, default).value
        self.role, self.base_pcd = p('role', ''), p('pcd_path', '')
        self.initial_reference = bool(p('initial_reference_available', False))
        if not self.initial_reference:
            raise RuntimeError(
                'ScanLock requires a validated reference PCD; mapless starts '
                'must use FAST-LIO only')
        self.executable, self.config = p('scan_lock_executable', ''), p('scan_lock_config', '')
        self.odom_frame, self.body_frame = p('odom_frame', ''), p('body_frame', '')
        self.imu_frame, self.map_frame = p('imu_frame', ''), p('map_frame', '')
        self.initialpose_topic = p('initialpose_topic', '/initialpose')
        self.initial_guess_z = float(p('initial_guess_z', 0.0))
        self.exit_overlap = float(p('handoff.exit_overlap', 0.50))
        self.exit_updates = max(1, int(p('handoff.consecutive_exit_updates', 5)))
        self.started_at = self.get_clock().now()
        self.distance, self.last_position = 0.0, None
        self.state = 'STARTING_SCAN_LOCK'
        self.detail, self.process = '', None
        self.anchor_good, self.overlap = False, 0.0
        self.inside_bounds = False
        self.ever_anchored, self.exit_count = False, 0
        self.last_good_correction, self.frozen_correction = None, None
        self.seed_pose, self.restart_not_before = None, 0.0
        self.process_started_at = self.get_clock().now()
        self.active_pcd = self.base_pcd
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, self.initialpose_topic, 10)
        transient = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.reference_publisher = self.create_publisher(
            String, 'localization/reference_path', transient)
        self.status_publisher = self.create_publisher(
            String, 'localization/promotion_status', 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 20)
        self.create_subscription(String, 'localization/status', self.status_callback, 10)
        self.create_subscription(PointCloud2, 'points', self.cloud_callback,
                                 rclpy.qos.qos_profile_sensor_data)
        self.timer = self.create_timer(2.0, self.tick)
        self.seed_timer = self.create_timer(1.0, self.seed_scan_lock)
        self.load_reference(self.active_pcd)
        self.start_scan_lock(self.active_pcd)

    def load_reference(self, path):
        self.reference_publisher.publish(String(data=str(path)))

    def destroy_node(self):
        self.stop_scan_lock()
        super().destroy_node()

    def stop_scan_lock(self):
        if self.process is not None and self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self.process.terminate()
        self.process = None

    def odom_callback(self, message):
        current = tuple(getattr(message.pose.pose.position, axis) for axis in ('x', 'y', 'z'))
        if self.last_position is not None:
            step = math.dist(current, self.last_position)
            if step < 2.0:
                self.distance += step
        self.last_position = current

    def status_callback(self, message):
        try:
            status = json.loads(message.data)
            self.overlap = float(status.get('overlap') or 0.0)
            self.inside_bounds = bool(status.get('inside_reference_bounds', False))
            self.anchor_good = (status.get('state') == 'ANCHORED' and
                                self.inside_bounds and
                                self.overlap >= self.exit_overlap)
            if self.state == 'ACTIVE' and self.anchor_good:
                correction = self.lookup_pose(self.map_frame, self.odom_frame)
                if correction is not None:
                    self.last_good_correction = correction
                    self.ever_anchored = True
                self.exit_count = 0
            elif (self.state == 'ACTIVE' and self.ever_anchored and
                  (not self.inside_bounds or self.overlap < self.exit_overlap)):
                self.exit_count += 1
                if self.exit_count >= self.exit_updates:
                    self.freeze_global_correction()
            elif self.state == 'ACTIVE':
                self.exit_count = 0
        except (TypeError, ValueError, json.JSONDecodeError):
            self.anchor_good = False

    def elapsed(self, start=None):
        return (self.get_clock().now()-(start or self.started_at)).nanoseconds/1e9

    def publish_status(self):
        self.status_publisher.publish(String(data=json.dumps({
            'state': self.state,
            'elapsed_s': round(self.elapsed(), 1), 'travel_distance_m': round(self.distance, 2),
            'anchor_overlap': round(self.overlap, 3),
            'inside_reference_bounds': self.inside_bounds,
            'handoff_exit_overlap': self.exit_overlap,
            'handoff_exit_count': self.exit_count,
            'active_pcd': self.active_pcd, 'detail': self.detail}, separators=(',', ':'))))

    def tick(self):
        if self.process is not None and self.process.poll() is not None:
            code, self.process = self.process.returncode, None
            self.state, self.detail = 'FAILED', f'ScanLock exited with code {code}'
        self.publish_status()

    def lookup_pose(self, parent, child):
        try:
            return self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time(),
                                                   Duration(seconds=0.1))
        except TransformException:
            return None

    def start_scan_lock(self, pcd):
        command = [self.executable, '--ros-args', '-r', f'__ns:=/{self.role}',
                   '-r', f'/initialpose:={self.initialpose_topic}', '--params-file', self.config,
                   '-p', f'scan_lock.pcd_file_name:={pcd}', '-p', f'frames.map_frame:={self.map_frame}',
                   '-p', f'frames.odom_frame:={self.odom_frame}',
                   '-p', f'frames.body_frame:={self.body_frame}',
                   '-p', f'frames.imu_frame:={self.imu_frame}', '-p', 'topics.lidar_topic:=points']
        command.extend(['-p', f'initial_guess.z:={self.initial_guess_z}'])
        try:
            self.process = subprocess.Popen(command, start_new_session=True)
        except OSError as error:
            self.state, self.detail = 'FAILED', f'could not launch ScanLock: {error}'
            return
        self.state, self.restart_not_before = 'STARTING_SCAN_LOCK', time.monotonic()+2.0
        self.process_started_at = self.get_clock().now()
        self.detail = 'starting ScanLock with the initial reference PCD'

    def seed_scan_lock(self):
        if self.state != 'STARTING_SCAN_LOCK':
            return
        if self.seed_pose is not None:
            transform = self.seed_pose.transform
            pose = PoseWithCovarianceStamped()
            pose.header, pose.header.frame_id = self.seed_pose.header, self.map_frame
            pose.pose.pose.position.x, pose.pose.pose.position.y = transform.translation.x, transform.translation.y
            pose.pose.pose.position.z, pose.pose.pose.orientation = transform.translation.z, transform.rotation
            pose.pose.covariance[0] = pose.pose.covariance[7] = 0.25
            pose.pose.covariance[35] = 0.1
            self.pose_publisher.publish(pose)
        correction = self.lookup_pose(self.map_frame, self.odom_frame)
        correction_is_new = correction is not None and rclpy.time.Time.from_msg(
            correction.header.stamp) >= self.process_started_at
        if time.monotonic() >= self.restart_not_before and correction_is_new:
            self.state, self.detail, self.seed_pose = 'ACTIVE', 'ScanLock reference is active', None

    def cloud_callback(self, message):
        if self.state == 'FASTLIO_GLOBAL' and self.frozen_correction is not None:
            self.publish_frozen_correction(message.header.stamp)

    def freeze_global_correction(self):
        if self.last_good_correction is None:
            self.detail = 'reference exited, but no last-good correction is available'
            return
        self.frozen_correction = self.last_good_correction
        self.stop_scan_lock()
        self.state = 'FASTLIO_GLOBAL'
        self.detail = (
            'reference overlap lost; ScanLock stopped and map-to-odom frozen')
        self.publish_frozen_correction(self.get_clock().now().to_msg())
        self.get_logger().warning(self.detail)

    def publish_frozen_correction(self, stamp):
        transform = self.frozen_correction
        transform.header.stamp = stamp
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.odom_frame
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = SlamToScanLockSupervisor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
