"""Distributed, host-local ROS instrumentation node."""

import math
import hashlib
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
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from rcl_interfaces.msg import ParameterEvent
from sensor_msgs.msg import PointCloud2
from surf_multirobot_msgs.msg import (AllocationMetrics, BridgeQueueMetrics, DeliveryMetrics,
                                      LinkMetrics, PipelineMetrics,
                                      RealtimeAckMetrics)

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
            'runtime_artifacts': self._runtime_artifacts(),
            'image_reference': os.environ.get('SURF_IMAGE_REFERENCE', 'unavailable'),
            'image_digest': os.environ.get('SURF_IMAGE_DIGEST', 'unavailable'),
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

        self._configuration_clients = {}
        self._configuration_pending = set()
        self._configuration_saved = set()
        self._map_capacity_pause_reasons = {}
        self.create_subscription(ParameterEvent, '/parameter_events', self._parameter_event, 100)
        self.create_timer(2.0, self._capture_configuration)

        pipeline_topic = self.declare_parameter(
            'pipeline_topic', '/drone/comm/pipeline_metrics').value
        delivery_topic = self.declare_parameter(
            'delivery_topic', '/humanoid/comm/delivery_metrics').value
        realtime_ack_metrics_topic = self.declare_parameter(
            'realtime_ack_metrics_topic', '/drone/comm/realtime_ack_metrics').value
        self.create_subscription(PipelineMetrics, pipeline_topic, self._pipeline, 50)
        self.create_subscription(DeliveryMetrics, delivery_topic, self._delivery, 50)
        self.create_subscription(RealtimeAckMetrics, realtime_ack_metrics_topic,
                                 self._realtime_ack, 100)

        self.create_subscription(AllocationMetrics, '/drone/comm/allocation_metrics',
                                 self._allocation, 100)
        self.create_subscription(LinkMetrics, '/surf/comm/link_metrics', self._capacity, 20)
        self.create_subscription(BridgeQueueMetrics, '/surf/comm/bridge_queue_metrics',
                                 self._bridge_queue, 50)
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
            return ['wlP1p1s0=wifi5']
        if role == 'humanoid':
            return ['wlo1=wifi5']
        return []

    def _record(self, *args, **kwargs):
        if self.store.record(*args, **kwargs):
            self.event_count += 1

    @staticmethod
    def _runtime_artifacts():
        # Installed binary/config hashes still identify code when Docker omits .git.
        output = {}
        for package, executable in (('surf_drone', 'drone_scan_sender'),
                                    ('surf_humanoid', 'drone_data_receiver'),
                                    ('surf_data_tracker', 'data_tracker')):
            try:
                prefix = Path(get_package_prefix(package))
                paths = [prefix / 'lib' / package / executable]
                paths += list((prefix / 'share' / package / 'config').glob('*.yaml'))
                for path in paths:
                    if path.is_file():
                        output[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
            except (LookupError, OSError):
                continue
        # The entry point alone does not fingerprint Python implementation.
        for path in Path(__file__).resolve().parent.glob('*.py'):
            output[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
        for path in (Path('/config/robot.yaml'), Path('/config/transport.yaml')):
            if path.is_file():
                output[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
        return output

    @staticmethod
    def _tracked_node(name):
        return name.rsplit('/', 1)[-1] in (
            'drone_scan_sender', 'drone_data_receiver', 'halow_capacity_provider',
            'halow_bridge', 'halow_control_bridge')

    def _parameter_event(self, event):
        if not self._tracked_node(event.node):
            return
        values = {p.name: parameter_value_to_python(p.value)
                  for p in (*event.new_parameters, *event.changed_parameters)}
        self._record('configuration', 'runtime', event.node,
                     {'parameters': values, 'deleted': [p.name for p in event.deleted_parameters],
                      'kind': 'parameter_event'}, wall_time_ns=_stamp_ns(event.stamp))

    def _capture_configuration(self):
        for name, namespace in self.get_node_names_and_namespaces():
            node = namespace.rstrip('/') + '/' + name
            if not self._tracked_node(node) or node in self._configuration_saved or node in self._configuration_pending:
                continue
            client = self._configuration_clients.setdefault(node, None)
            if client is None:
                client = AsyncParameterClient(self, node)
                self._configuration_clients[node] = client
            if not client.services_are_ready():
                continue
            self._configuration_pending.add(node)
            def listed(future, node=node, client=client):
                try:
                    names = future.result().result.names
                    def captured(result):
                        try:
                            values = dict(zip(names, (parameter_value_to_python(v)
                                                      for v in result.result().values)))
                            self._record('configuration', 'runtime', node,
                                         {'parameters': values, 'kind': 'initial_snapshot'})
                            self._configuration_saved.add(node)
                        except Exception as error:
                            self.get_logger().warning(f'Parameter snapshot failed for {node}: {error}')
                        finally:
                            self._configuration_pending.discard(node)
                    client.get_parameters(names, callback=captured)
                except Exception as error:
                    self._configuration_pending.discard(node)
                    self.get_logger().warning(f'Parameter listing failed for {node}: {error}')
            client.list_parameters(callback=listed)

    def _allocation(self, m):
        payload = {}
        for name in m.get_fields_and_field_types():
            if name == 'header':
                continue
            value = getattr(m, name)
            if isinstance(value, (str, int, float, bool)):
                payload[name] = value
            else:
                for i, item in enumerate(value):
                    payload[f'{name}_{i}'] = item if isinstance(item, str) else float(item)
        payload['projected'] = int(any(
            not math.isfinite(requested) or abs(requested - allocated) > 1e-9
            for requested, allocated in zip(m.requested_rate, m.allocated_rate)))
        payload['saturated'] = int(m.projection_scale < 1.0)
        for i in range(2):
            payload[f'mean_priority_sent_{i}'] = (
                m.sent_priority[i] / m.selected_count[i] if m.selected_count[i] else 0.0)
            payload[f'offered_cdr_rate_{i}'] = m.wire_bytes[i] / m.actual_dt if m.actual_dt > 0 else 0.0
        allocated = sum(m.allocated_rate)
        offered = sum(payload[f'offered_cdr_rate_{i}'] for i in range(2))
        payload['allocation_realization'] = offered / allocated if allocated > 0 else None
        payload['capacity_utilization'] = (
            offered / m.usable_capacity if m.usable_capacity > 0 else None)
        self._record('allocation', 'information_debt', 'controller', payload,
                     wall_time_ns=_stamp_ns(m.header.stamp),
                     event_id=f'allocation:{m.source_id}:{m.map_epoch}:{m.step}')

        key = (m.source_id, m.map_epoch)
        previous = self._map_capacity_pause_reasons.get(key)
        outstanding = int(m.pending_count[0] + m.pending_count[1])
        reason = m.capacity_method or 'unspecified_zero_capacity'
        transition = None
        if m.usable_capacity <= 0 and outstanding and previous != reason:
            self._map_capacity_pause_reasons[key] = reason
            transition = ('paused', reason)
        elif m.usable_capacity > 0 and previous is not None:
            self._map_capacity_pause_reasons[key] = None
            transition = ('resumed', previous)
        if transition:
            state, stop_reason = transition
            self._record('transport', 'map_transmission', f'{state}:{stop_reason}', {
                'transition_count': 1,
                'state': state,
                'reason': stop_reason,
                'capacity_method': m.capacity_method,
                'usable_capacity_bytes_per_second': m.usable_capacity,
                'outstanding_updates': outstanding,
                'source_id': m.source_id,
                'map_epoch': m.map_epoch,
                'allocation_step': m.step,
            }, wall_time_ns=_stamp_ns(m.header.stamp),
                event_id=f'map-transmission:{m.source_id}:{m.map_epoch}:{m.step}')

    def _bridge_queue(self, m):
        payload = {field: getattr(m, field)
                   for field in m.get_fields_and_field_types() if field != 'header'}
        self._record('network', 'network_bridge', m.topic, payload,
                     wall_time_ns=_stamp_ns(m.header.stamp))

    def _capacity(self, m):
        self._record('capacity', 'radio', m.link_name, {
            'usable_capacity_bytes_per_second': m.usable_capacity_bytes_per_second,
            'capacity_method': m.capacity_method,
            'raw_mmrc_average_mbps': m.raw_mmrc_average_mbps,
            'experimental_capacity_factor': m.experimental_capacity_factor,
            'raw_mcs': m.raw_mcs, 'raw_rssi_dbm': m.raw_rssi_dbm,
            'achieved_interface_mbps': m.measured_throughput_mbps,
        }, wall_time_ns=_stamp_ns(m.header.stamp))

    def _pipeline(self, m):
        traffic = {1: 'delta', 2: 'backlog'}.get(m.traffic_class, str(m.traffic_class))
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
            'stale_input_drops': m.stale_input_drops,
            'free_updates': m.free_updates, 'unknown_updates': m.unknown_updates,
            'ray_cells_visited': m.ray_cells_visited, 'sampled_rays': m.sampled_rays}
        if m.traffic_class == 0:
            # Input processing is separate from later control-step transmission.
            # Unpopulated ROS numeric defaults are not measured zero-byte encodings.
            for key in ('uncompressed_bytes', 'payload_bytes', 'wire_bytes', 'codec',
                        'selected_voxels', 'packet_count', 'packet_budget_bytes',
                        'update_budget_bytes', 'compression_ms', 'wire_serialization_ms'):
                payload.pop(key, None)
            traffic = 'input'
        self._record('pipeline', 'pointcloud_communication', traffic, payload,
                     wall_time_ns=_stamp_ns(m.transmit_stamp),
                     event_id=f'pipeline:{m.source_id}:{m.map_epoch}:{m.version}:{m.traffic_class}')

    def _delivery(self, m):
        traffic = {1: 'delta', 2: 'backlog'}.get(m.traffic_class, str(m.traffic_class))
        payload = {'protocol': 'information', 'traffic_class': m.traffic_class, 'wire_bytes': m.wire_bytes,
                   'source_id': m.source_id, 'map_epoch': m.map_epoch, 'version': m.version,
                   'voxel_count': m.voxel_count, 'decode_ms': m.decode_latency_ms,
                   'accepted_voxel_count': m.accepted_voxel_count,
                   'oldest_accepted_observation_age_s': m.oldest_accepted_observation_age_s,
                   'newest_accepted_observation_age_s': m.newest_accepted_observation_age_s,
                   'mean_accepted_observation_age_s': m.mean_accepted_observation_age_s,
                   'chunk_index': m.chunk_index, 'chunk_count': m.chunk_count,
                   'codec_reconstruction_ms': m.codec_reconstruction_ms,
                   'receiver_processing_ms': m.receiver_processing_ms,
                   'stale_voxels_rejected': m.stale_voxels_rejected,
                   'applied_temporal_regressions': m.applied_temporal_regressions,
                   'attempted_temporal_regressions': m.attempted_temporal_regressions,
                   'accepted': bool(m.accepted), 'rejection_reason': m.rejection_reason,
                   'sender_to_receiver_ms': m.sender_to_receiver_ms,
                   'sensor_to_receiver_ms': m.sensor_to_receiver_ms,
                   'end_to_end_ms': m.end_to_end_latency_ms}
        self._record('delivery', 'pointcloud_communication', traffic, payload,
                     wall_time_ns=_stamp_ns(m.header.stamp),
                     event_id=(f'delivery:{m.source_id}:{m.map_epoch}:{m.version}:'
                               f'{m.traffic_class}:{m.chunk_index}'))

    def _realtime_ack(self, m):
        payload = {
            'map_epoch': m.map_epoch, 'version': m.version,
            'ack_received': int(m.acknowledged),
            'ack_lost': int(not m.acknowledged),
            'ack_success_fraction': int(m.acknowledged),
            'attempted_wire_bytes': m.wire_bytes,
            'acknowledged_wire_bytes': m.wire_bytes if m.acknowledged else 0,
            'update_completion_rtt_ms': (
                m.update_completion_rtt_ms if m.acknowledged else None),
            'final_chunk_rtt_ms': m.final_chunk_rtt_ms if m.acknowledged else None,
            'estimated_one_way_ms': (
                m.final_chunk_rtt_ms / 2.0 if m.acknowledged else None),
            'timeout_ms': m.timeout_ms,
            'ack_lock_wait_ms': m.ack_lock_wait_ms if m.acknowledged else None,
        }
        state = 'acknowledged' if m.acknowledged else 'timeout'
        self._record('latency', 'pointcloud_communication',
                     f'halow:{"delta" if m.traffic_class == 1 else "backlog"}:{state}', payload,
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
