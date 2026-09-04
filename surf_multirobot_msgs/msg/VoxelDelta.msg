# Uncompressed, versioned changes to a sparse voxel map.
uint8 MODE_FULL=0
uint8 MODE_VOXEL_DELTAS=1
uint8 MODE_DYNAMIC_ONLY=2
uint8 MODE_METADATA_ONLY=3

uint8 STATE_OCCUPIED_STATIC=1
uint8 STATE_OCCUPIED_DYNAMIC=2
uint8 STATE_FREE=3
uint8 STATE_DELETE=4

std_msgs/Header header
string source_id
uint64 map_epoch
uint64 version
uint64 base_version
uint8 operating_mode
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
