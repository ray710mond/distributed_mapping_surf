# Host-local telemetry; this message is never bridged over either radio.
std_msgs/Header header
uint64 map_epoch
uint64 version
bool acknowledged
# First map chunk sent through receipt of the ACK.
float32 update_completion_rtt_ms
# Final map chunk sent through receipt of the ACK.
float32 final_chunk_rtt_ms
float32 timeout_ms

# 1 DELTA, 2 BACKLOG (legacy type name retained).
uint8 traffic_class
# Serialized packet bytes associated with this attempt.
uint64 wire_bytes

# Sender callback entry -> state mutex acquisition, excludes DDS/executor wait.
float32 ack_lock_wait_ms
