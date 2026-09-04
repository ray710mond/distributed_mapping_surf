# Wire representation. The payload contains deterministic binary VoxelDelta
# records and is compressed when doing so reduces the payload size.
uint8 TRAFFIC_CONTROL=0
uint8 TRAFFIC_REALTIME=1
uint8 TRAFFIC_SYNC=2

std_msgs/Header header
# Wall-clock timestamp taken immediately before serialization/publish. Unlike
# header.stamp (the originating sensor sample), this isolates network latency.
builtin_interfaces/Time transmit_stamp
string source_id
uint64 map_epoch
uint64 version
uint64 base_version
uint8 operating_mode
uint8 traffic_class
bool full_refresh
# Zero-based independently serialized chunk within this logical version.
uint32 chunk_index
uint32 chunk_count
float32 resolution
geometry_msgs/Point sensor_origin
string codec
uint32 voxel_count
uint32 uncompressed_bytes
uint8[] payload
