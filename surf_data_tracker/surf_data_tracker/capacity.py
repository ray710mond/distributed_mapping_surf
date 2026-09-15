"""Guarded live capacity estimate outside LQR. MMRC is not application goodput."""
import math


def usable_capacity(telemetry, *, measured_application_bps=None, mmrc_factor=0.1):
    """Use fresh Morse rate-control telemetry with reserved link headroom.

    Never use achieved throughput or the S1G nl80211 compatibility rate as capacity.
    Missing/stale measurements stop map publication rather than granting credit.
    """
    if measured_application_bps is not None and math.isfinite(measured_application_bps):
        return max(0.0, measured_application_bps), 'direct_application_measurement'
    rate = telemetry.get('s1g_average_throughput_mbps')
    probability = telemetry.get('s1g_success_probability_pct')
    if (telemetry.get('morse_sample_valid') and rate is not None and
            math.isfinite(rate) and rate > 0 and math.isfinite(mmrc_factor) and
            0 < mmrc_factor <= 1 and probability is not None and
            math.isfinite(probability) and probability > 0):
        return rate * 125000 * mmrc_factor * min(1.0, probability / 100), 'guarded_mmrc_estimate'
    return 0.0, 'telemetry_unavailable'


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from surf_multirobot_msgs.msg import LinkMetrics
    from .network import NetworkSampler

    class CapacityProvider(Node):
        def __init__(self):
            super().__init__('halow_capacity_provider')
            interface = self.declare_parameter('interface', 'wlx0cbf7400343c').value
            self.factor = self.declare_parameter('experimental_mmrc_factor', 0.1).value
            if not math.isfinite(self.factor) or not 0 < self.factor <= 1:
                raise ValueError('experimental_mmrc_factor must be in (0, 1]')
            self.sampler = NetworkSampler({interface: 'halow'})
            self.publisher = self.create_publisher(LinkMetrics, '/surf/comm/link_metrics', 10)
            self.create_timer(1.0, self.sample)

        def sample(self):
            raw = self.sampler.sample()[0]
            capacity, method = usable_capacity(raw, mmrc_factor=self.factor)
            m = LinkMetrics()
            m.header.stamp = self.get_clock().now().to_msg()
            m.link_name = 'halow'
            m.usable_capacity_valid = method != 'telemetry_unavailable'
            m.usable_capacity_bytes_per_second = capacity
            m.capacity_method = method
            m.raw_mmrc_average_mbps = float(raw.get('s1g_average_throughput_mbps', float('nan')))
            m.experimental_capacity_factor = float(self.factor)
            m.raw_mcs = float(raw.get('s1g_mcs', float('nan')))
            m.raw_rssi_dbm = float(raw.get('rssi_dbm', float('nan')))
            m.measured_throughput_mbps = float(raw.get('tx_interface_mbps', float('nan')))
            self.publisher.publish(m)

    rclpy.init(args=args)
    node = CapacityProvider()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
