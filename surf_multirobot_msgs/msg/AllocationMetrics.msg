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
# Includes resolved records retained for recovery.
uint64 retained_count
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
# Flat index class*4+bucket/state. Age buckets are <10, 10-30, 30-60, >=60 s;
# state slots correspond to VoxelDelta states 1..4.
uint64[8] pending_age_bucket_count
uint64[8] selected_state_count
uint64[2] old_pending_selected_count
float64[2] mean_selected_pending_age_s
float64[2] max_selected_pending_age_s
uint64[2] uncompressed_bytes
uint64[2] compressed_bytes
# Serialized CDR bytes offered to network_bridge; excludes bridge/IP/radio overhead.
uint64[2] wire_bytes
uint64 superseded_count
uint64 stale_observations

float64[2] disturbance
float64[2] timed_out_debt

float64 controller_ms
float64 debt_advance_ms
float64 debt_score_ms
float64 metrics_scan_ms
float64 debt_cycle_ms
float64[2] selection_ms
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

# Bounded scheduler: debt and pending/in-flight counts are global; age buckets,
# age extrema, ACK-lag sample counts and priority maxima describe this sample.
uint64 scored_entries
bool sampled_age_metrics
uint64 recovery_entries_processed
uint64 recovery_entries_remaining
uint64[2] selected_unknown_count
# Block keys are floor(voxel coordinate / 16). Delivery completion means all
# currently known records in a block were ACKed, NOT all block cells observed,
# nor that downstream map integration/planner validation has completed.
bool focus_region_valid
int32[3] focus_region
uint64 focus_region_revision
uint64 focus_region_known
uint64 focus_region_pending
uint64 completed_region_deliveries

# Raw LQR requested_rate is retained above. Before capacity projection, streams
# without eligible sampled candidates have their serviceable request set to zero.
float64[2] serviceable_requested_rate
uint64[2] eligible_candidate_count

# BACKLOG records are ordered by proximity-graph component; a packet may combine
# small component groups to use its byte budget. UNKNOWN records break
# connectivity. Completion means no eligible record was
# found within the bounded traversal; it is a scheduler event, not map coverage.
bool focus_cluster_valid
int32[3] focus_cluster_seed
uint64 focus_cluster_visited
uint64 focus_cluster_candidates
uint64 selected_cluster_count
bool focus_cluster_truncated
uint64 completed_cluster_deliveries
