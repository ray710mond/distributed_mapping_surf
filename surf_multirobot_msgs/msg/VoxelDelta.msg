# Uncompressed, versioned changes to a sparse voxel map.
# Deprecated legacy snapshot/adaptive constants; retained for old wire readers.
uint8 MODE_FULL=0
uint8 MODE_VOXEL_DELTAS=1
uint8 MODE_DYNAMIC_ONLY=2
uint8 MODE_METADATA_ONLY=3
# Incremental DELTA/BACKLOG protocol with independent per-voxel temporal validation.
uint8 MODE_INFORMATION=4

uint8 STATE_OCCUPIED_STATIC=1
uint8 STATE_OCCUPIED_DYNAMIC=2
uint8 STATE_FREE=3
# Legacy scan-observed clearing of a static cell (also free evidence).
uint8 STATE_DELETE=4
# Withdraw this source's observation; no free-space evidence. Used for expiry.
uint8 STATE_UNKNOWN=5

std_msgs/Header header
string source_id
uint64 map_epoch
uint64 version
uint64 base_version
uint8 operating_mode
# Compatibility field; normal information packets always set false.
bool full_refresh
# Zero-based chunk metadata. A value of 0/1 is an unchunked update.
uint32 chunk_index
uint32 chunk_count
float32 resolution
geometry_msgs/Point sensor_origin
int32[] x
int32[] y
int32[] z
uint8[] state
# Observation time for each voxel record, in nanoseconds in the header clock domain.
uint64[] observation_time_ns
# Optional measured-ray metadata per record. Flag 1 is a sampled FREE prefix;
# flag 2 is an occupied hit with its own scan origin (the endpoint is the voxel
# center). The receiver reconstructs free prefixes locally. Empty arrays are
# legacy voxel updates.
uint8[] ray_flags
geometry_msgs/Point[] ray_origins
geometry_msgs/Point[] ray_endpoints
