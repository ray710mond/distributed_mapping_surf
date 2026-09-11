std_msgs/Header header
string link_name
float32 configured_bandwidth_mbps
float32 configured_latency_ms
float32 configured_jitter_ms
float32 configured_loss_percent
float32 measured_throughput_mbps
float32 mean_delivery_latency_ms
uint32 queue_depth
uint64 received_packets
uint64 delivered_packets
uint64 dropped_loss
uint64 dropped_stale
uint64 transmitted_bytes

# Externally derived application capacity, NOT achieved throughput or generic iw rate.
bool usable_capacity_valid
float64 usable_capacity_bytes_per_second
string capacity_method

# Raw driver rate-control statistic, NOT PHY rate or application goodput (NaN unavailable).
float64 raw_mmrc_average_mbps
float64 experimental_capacity_factor
float64 raw_mcs
float64 raw_rssi_dbm
