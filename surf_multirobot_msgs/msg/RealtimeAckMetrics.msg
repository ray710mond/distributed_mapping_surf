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
