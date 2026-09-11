"""Capacity calibration outside LQR. MMRC rates are driver statistics, not goodput."""
import math


def usable_capacity(telemetry, *, measured_application_bps=None, mmrc_factor=0.0,
                    configured_bps=0.0):
    """Prefer a directly supplied application measurement, then calibrated MMRC.

    Never use interface achieved throughput or the S1G nl80211 compatibility rate.
    mmrc_factor=0 disables uncalibrated conversion; configured fallback is explicit.
    """
    if measured_application_bps is not None and math.isfinite(measured_application_bps):
        return max(0.0, measured_application_bps), 'direct_application_measurement'
    rate = telemetry.get('s1g_average_throughput_mbps')
    if rate is not None and math.isfinite(rate) and 0 < mmrc_factor <= 1:
        return max(0.0, rate * 125000 * mmrc_factor), 'experimental_mmrc_conversion'
    return max(0.0, configured_bps), 'development_configured'


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from surf_multirobot_msgs.msg import LinkMetrics
    from .network import NetworkSampler

    class CapacityProvider(Node):
        def __init__(self):
            super().__init__('halow_capacity_provider')
            interface = self.declare_parameter('interface', 'wlx0cbf7400343c').value
            self.factor = self.declare_parameter('experimental_mmrc_factor', 0.0).value
            self.fallback = self.declare_parameter('development_bytes_per_second', 10000.0).value
            self.sampler = NetworkSampler({interface: 'halow'})
            self.publisher = self.create_publisher(LinkMetrics, '/surf/comm/link_metrics', 10)
            self.create_timer(1.0, self.sample)

        def sample(self):
            raw = self.sampler.sample()[0]
            capacity, method = usable_capacity(raw, mmrc_factor=self.factor,
                                               configured_bps=self.fallback)
            m = LinkMetrics()
            m.header.stamp = self.get_clock().now().to_msg()
            m.link_name = 'halow'
            m.usable_capacity_valid = True
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
