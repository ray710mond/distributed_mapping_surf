# Compatibility type name: receiver request to invalidate delivery knowledge and enqueue BACKLOG.
std_msgs/Header header
# Empty source_id with epoch zero requests recovery from connected senders.
string source_id
uint64 map_epoch
uint64 last_version
string reason
