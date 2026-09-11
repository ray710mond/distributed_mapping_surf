# State/input ordering: DELTA, BACKLOG. Matrices are row-major at nominal_dt.
# Debt events are cumulative; difference successive rows for identification.
std_msgs/Header header
string source_id
uint64 map_epoch
uint64 step
float64 nominal_dt
float64 actual_dt
float64 computation_ms
float64[2] debt
float64[2] previous_debt
uint64[2] pending_count
uint64[2] inflight_count
float64[2] generated_debt
float64[2] acknowledged_debt
float64[2] superseded_debt
float64[2] reclassified_debt
float64[4] a
float64[4] b
float64[4] q
float64[4] r
float64[4] k
uint64 configuration_revision
uint64 rejected_matrix_updates
float64 matrix_update_time
float64[2] requested_rate
float64[2] allocated_rate
float64 usable_capacity
string capacity_method
float64 projection_scale
float64[2] target_bytes
uint64[2] selected_count
float64[2] sent_priority
uint64[2] uncompressed_bytes
uint64[2] compressed_bytes
# Serialized CDR bytes offered to network_bridge; excludes bridge/IP/radio overhead.
uint64[2] wire_bytes
uint64 superseded_count
uint64 stale_observations

float64[2] disturbance
float64[2] timed_out_debt

float64 controller_ms
float64 priority_ms
float64 encoding_ms
float64[2] mean_priority_remaining
float64[2] max_priority_remaining
float64[2] max_priority_sent
float64 oldest_pending_age

# Debt removed because the peer-owned mask excludes this coordinate.
float64[2] excluded_debt

# Observation ages use the sensor/ROS clock; pending age above uses steady time.
# Age/lag maxima are zero when the corresponding sample count is zero.
float64 oldest_observation_age_s
float64 newest_observation_age_s
float64 max_ack_observation_lag_s
uint64 ack_lag_samples
uint64 stale_pending_discarded
string[2] codec
