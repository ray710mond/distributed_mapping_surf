"""Distributed, host-local ROS instrumentation node."""

import math
import os
import socket
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from uuid import uuid4

import rclpy
from ament_index_python.packages import get_package_prefix
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from surf_multirobot_msgs.msg import (DeliveryMetrics, PipelineMetrics,
                                      RealtimeAckMetrics, SyncStatus)

from .clock import detect_clock_sync
from .network import NetworkSampler
from .storage import EventStore, validate_run_id


def _stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def _git_revision():
    try:
        return subprocess.run(['git', 'rev-parse', 'HEAD'], capture_output=True,
                              text=True, timeout=1.0, check=False).stdout.strip() or 'unavailable'
    except (OSError, subprocess.TimeoutExpired):
        return 'unavailable'


class DataTracker(Node):
    """Record only facts observable on this host; merge and analyze offline."""

    def __init__(self):
        super().__init__('data_tracker')
        self.role = self.declare_parameter('robot_role', 'unknown').value.strip()
        configured_id = self.declare_parameter('run_id', '').value.strip()
        run_id = configured_id or datetime.now(timezone.utc).strftime(
            '%Y%m%dT%H%M%SZ') + '-' + uuid4().hex[:8]
        validate_run_id(run_id)
        output = self.declare_parameter('output_root', '').value.strip()
        legacy = self.declare_parameter('output_directory', '').value.strip()
        root = Path(output or legacy).expanduser() if (output or legacy) else self._default_output()
        directory = root / run_id / 'hosts' / socket.gethostname()
        clock_method = self.declare_parameter(
            'clock_sync_method', 'unverified').value.strip()
        clock_offset = self._finite_parameter('clock_offset_ms')
        clock_uncertainty = self._finite_parameter('clock_uncertainty_ms')
        metadata = {
            'experiment_name': self.declare_parameter('experiment_name', '').value,
            'notes': self.declare_parameter('notes', '').value,
            'clock_sync_method': clock_method,
            'clock_offset_ms': clock_offset,
            'clock_uncertainty_ms': clock_uncertainty,
            'preferred_timeseries_window_s': float(
                self.declare_parameter('timeseries_window_s', 1.0).value),
            'git_revision': _git_revision(), 'pid': os.getpid(),
        }
        if clock_method in ('', 'unverified'):
            detected_clock = detect_clock_sync()
            if detected_clock:
                metadata.update(detected_clock)
        raw = bool(self.declare_parameter('record_raw_events', False).value)
        self.store = EventStore(directory, run_id, self.role, metadata, raw_events=raw)
        self.event_count = 0
        self.final_status = 'completed'
        self._record('clock', 'synchronization', self.role, {
            'clock_verified': int(
                metadata['clock_sync_method'] not in ('', 'unverified')
                and metadata['clock_uncertainty_ms'] is not None),
            'clock_offset_ms': metadata['clock_offset_ms'],
            'clock_uncertainty_ms': metadata['clock_uncertainty_ms'],
            'clock_sync_method': metadata['clock_sync_method'],
            'clock_reference': metadata.get('clock_reference', ''),
        }, event_id=f'clock:{socket.gethostname()}:{run_id}')

        pipeline_topic = self.declare_parameter(
            'pipeline_topic', '/drone/comm/pipeline_metrics').value
        delivery_topic = self.declare_parameter(
            'delivery_topic', '/humanoid/comm/delivery_metrics').value
        sync_status_topic = self.declare_parameter(
            'sync_status_topic', '/drone/comm/sync_status').value
        realtime_ack_metrics_topic = self.declare_parameter(
            'realtime_ack_metrics_topic', '/drone/comm/realtime_ack_metrics').value
        self.create_subscription(PipelineMetrics, pipeline_topic, self._pipeline, 50)
        self.create_subscription(DeliveryMetrics, delivery_topic, self._delivery, 50)
        self.create_subscription(SyncStatus, sync_status_topic, self._sync_status, 50)
        self.create_subscription(RealtimeAckMetrics, realtime_ack_metrics_topic,
                                 self._realtime_ack, 100)

        self.pose_subscriptions = []
        for specification in self.declare_parameter(
                'pose_topics', self._default_pose_topics(self.role)).value:
            identity, topic = specification.split('=', 1)
            self.pose_subscriptions.append(self.create_subscription(
                Odometry, topic, lambda msg, identity=identity: self._pose(identity, msg), 20))
        self.cloud_subscriptions = []
        for specification in self.declare_parameter(
                'cloud_topics', self._default_cloud_topics(self.role)).value:
            pipeline, stage, topic = specification.split('=', 2)
            self.cloud_subscriptions.append(self.create_subscription(
                PointCloud2, topic, lambda msg, pipeline=pipeline, stage=stage:
                self._cloud(pipeline, stage, msg), 5))

        specs = self.declare_parameter(
            'network_interfaces', self._default_interfaces(self.role)).value
        morse_rate = max(.1, float(
            self.declare_parameter('morse_stats_rate_hz', 1.0).value))
        morse_cli = self.declare_parameter('morse_cli_path', '').value.strip()
        self.network_sampler = NetworkSampler(
            dict(spec.split('=', 1) for spec in specs if '=' in spec),
            morse_stats_rate_hz=morse_rate, morse_cli=morse_cli)
        rate = max(.1, float(self.declare_parameter('network_stats_rate_hz', 2.0).value))
        if bool(self.declare_parameter('network_stats_enabled', True).value):
            self.create_timer(1.0 / rate, self._network)
        flush = max(.25, float(self.declare_parameter('flush_interval_s', 2.0).value))
        self.create_timer(flush, self._flush)
        if not configured_id:
            self.get_logger().warning(
                'No shared run_id supplied; pass the same run_id on every participating host.')
        self.get_logger().info(f'Tracker started: run_id={run_id}, output={directory}')

    def _finite_parameter(self, name):
        value = float(self.declare_parameter(name, float('nan')).value)
        return value if math.isfinite(value) else None

    @staticmethod
    def _default_output():
        prefix = Path(get_package_prefix('surf_data_tracker')).resolve()
        workspace = prefix.parent.parent
        candidates = (workspace / 'src' / 'surf_data_tracker',
                      workspace / 'surf_ws' / 'src' / 'surf_data_tracker')
        source = next((candidate for candidate in candidates if candidate.is_dir()), None)
        if source:
            return source / 'experiments'
        return Path.home() / '.local' / 'share' / 'surf' / 'experiments'

    @staticmethod
    def _default_pose_topics(role):
        if role == 'drone':
            return ['drone=/drone/transport/drone_odometry',
                    'humanoid=/drone/transport/humanoid_odometry']
        if role == 'humanoid':
            return ['humanoid=/humanoid/transport/humanoid_odometry',
                    'drone=/humanoid/transport/drone_odometry']
        return []

    @staticmethod
    def _default_cloud_topics(role):
        if role not in ('drone', 'humanoid'):
            return []
        return [f'pointcloud=communication_input=/{role}/points_body',
                f'map=occupied=/{role}/bonxai/occupied_voxels',
                f'map=static=/{role}/bonxai/static_occupied_voxels']

    @staticmethod
    def _default_interfaces(role):
        if role == 'drone':
            return ['wlx0cbf7400343c=halow_realtime', 'wlP1p1s0=wifi5_sync']
        if role == 'humanoid':
            return ['wlx0cbf740035d4=halow_realtime', 'ap0=wifi5_sync']
        return []

    def _record(self, *args, **kwargs):
        if self.store.record(*args, **kwargs):
            self.event_count += 1

    def _pipeline(self, m):
        traffic = {1: 'realtime', 2: 'sync'}.get(m.traffic_class, str(m.traffic_class))
        payload = {
            'traffic_class': m.traffic_class, 'traffic_class_name': traffic,
            'operating_mode': m.operating_mode, 'input_rate_hz': m.input_rate_hz,
            'raw_points': m.raw_points, 'valid_points': m.valid_points,
            'unique_voxels': m.unique_voxels, 'static_prior_voxels': m.static_prior_voxels,
            'temporal_suppressed_voxels': m.temporal_suppressed_voxels,
            'selected_voxels': m.selected_voxels,
            'packet_count': m.packet_count,
            'packet_budget_bytes': m.packet_budget_bytes,
            'update_budget_bytes': m.update_budget_bytes,
            'raw_serialized_bytes': m.raw_serialized_bytes,
            'raw_data_bytes': m.raw_data_bytes, 'point_step_bytes': m.point_step_bytes,
            'uncompressed_bytes': m.uncompressed_bytes, 'payload_bytes': m.payload_bytes,
            'wire_bytes': m.wire_bytes, 'codec': m.codec,
            'compression_ms': m.compression_latency_ms,
            'raw_serialization_ms': m.raw_serialization_ms,
            'wire_serialization_ms': m.wire_serialization_ms,
            'queue_wait_ms': m.queue_wait_ms,
            'transform_lookup_ms': m.transform_lookup_ms,
            'point_preprocessing_ms': m.point_preprocessing_ms,
            'occupancy_selection_ms': m.occupancy_selection_ms,
            'clearing_ms': m.clearing_ms, 'processing_ms': m.processing_latency_ms,
            'stale_input_drops': m.stale_input_drops}
        self._record('pipeline', 'pointcloud_communication', traffic, payload,
                     wall_time_ns=_stamp_ns(m.transmit_stamp),
                     event_id=f'pipeline:{m.source_id}:{m.map_epoch}:{m.version}:{m.traffic_class}')

    def _delivery(self, m):
        traffic = {1: 'realtime', 2: 'sync'}.get(m.traffic_class, str(m.traffic_class))
        payload = {'traffic_class': m.traffic_class, 'wire_bytes': m.wire_bytes,
                   'source_id': m.source_id, 'map_epoch': m.map_epoch, 'version': m.version,
                   'voxel_count': m.voxel_count, 'decode_ms': m.decode_latency_ms,
                   'chunk_index': m.chunk_index, 'chunk_count': m.chunk_count,
                   'codec_reconstruction_ms': m.codec_reconstruction_ms,
                   'receiver_processing_ms': m.receiver_processing_ms,
                   'accepted': bool(m.accepted), 'rejection_reason': m.rejection_reason,
                   'sender_to_receiver_ms': m.sender_to_receiver_ms,
                   'sensor_to_receiver_ms': m.sensor_to_receiver_ms,
                   'end_to_end_ms': m.end_to_end_latency_ms}
        self._record('delivery', 'pointcloud_communication', traffic, payload,
                     wall_time_ns=_stamp_ns(m.header.stamp),
                     event_id=(f'delivery:{m.source_id}:{m.map_epoch}:{m.version}:'
                               f'{m.traffic_class}:{m.chunk_index}'))

    def _sync_status(self, m):
        states = {
            0: 'queued', 1: 'retry', 2: 'acknowledged', 3: 'requested',
            4: 'transmitted'}
        state = states.get(m.state, str(m.state))
        stamp_ns = _stamp_ns(m.header.stamp)
        payload = {
            'source_id': m.source_id, 'map_epoch': m.map_epoch, 'version': m.version,
            'state': m.state, 'state_name': state, 'wire_bytes': m.wire_bytes,
            'packet_count': m.packet_count, 'retry_count': m.retry_count,
            'encoding_ms': m.encoding_ms,
            'estimated_transfer_ms': m.estimated_transfer_ms,
            'transfer_ms': m.transfer_ms,
            'acknowledgment_ms': m.acknowledgment_ms,
            'acknowledgment_timeout_ms': m.acknowledgment_timeout_ms,
            'next_refresh_interval_s': m.next_refresh_interval_s,
            'reason': m.reason}
        self._record('sync', 'pointcloud_communication', state, payload,
                     wall_time_ns=stamp_ns,
                     event_id=(f'sync:{m.source_id}:{m.map_epoch}:{m.version}:'
                               f'{m.state}:{m.retry_count}:{stamp_ns}'))

    def _realtime_ack(self, m):
        payload = {
            'map_epoch': m.map_epoch, 'version': m.version,
            'ack_received': int(m.acknowledged),
            'ack_lost': int(not m.acknowledged),
            'update_completion_rtt_ms': (
                m.update_completion_rtt_ms if m.acknowledged else None),
            'final_chunk_rtt_ms': m.final_chunk_rtt_ms if m.acknowledged else None,
            'estimated_one_way_ms': (
                m.final_chunk_rtt_ms / 2.0 if m.acknowledged else None),
            'timeout_ms': m.timeout_ms,
        }
        state = 'acknowledged' if m.acknowledged else 'timeout'
        self._record('latency', 'pointcloud_communication',
                     f'halow_realtime:{state}', payload,
                     wall_time_ns=_stamp_ns(m.header.stamp),
                     event_id=(f'realtime_ack:{m.map_epoch}:{m.version}:{state}'))

    def _pose(self, identity, message):
        p = message.pose.pose.position
        self._record('pose', 'localization', identity,
                     {'x_m': p.x, 'y_m': p.y, 'z_m': p.z,
                      'frame_id': message.header.frame_id,
                      'child_frame_id': message.child_frame_id},
                     wall_time_ns=_stamp_ns(message.header.stamp))

    def _cloud(self, pipeline, stage, message):
        fields = ';'.join(f'{field.name}:{field.offset}:{field.datatype}:{field.count}'
                          for field in message.fields)
        self._record('data', pipeline, stage,
                     {'element_count': message.width * message.height,
                      'data_bytes': len(message.data),
                      'point_step_bytes': message.point_step,
                      'row_step_bytes': message.row_step,
                      'height': message.height, 'width': message.width,
                      'fields_layout': fields, 'frame_id': message.header.frame_id},
                     wall_time_ns=_stamp_ns(message.header.stamp))

    def _network(self):
        for sample in self.network_sampler.sample():
            self._record('network', 'wireless', sample['link_identifier'], sample)

    def _flush(self):
        self.store.flush()

    def destroy_node(self):
        if getattr(self, 'store', None):
            self.store.close(self.final_status)
            message = f'Tracker finalized {self.event_count} events in {self.store.directory}'
            if rclpy.ok():
                self.get_logger().info(message)
            else:
                # SIGINT/SIGTERM can invalidate rosout before destroy_node().
                # A plain status line avoids publishing through a dead context.
                print(f'[data_tracker] {message}', flush=True)
            self.store = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        node.final_status = 'interrupted'
        raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
