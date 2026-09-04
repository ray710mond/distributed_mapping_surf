cmake_minimum_required(VERSION 3.8)
project(surf_multirobot_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CompressedVoxelDelta.msg"
  "msg/DeliveryMetrics.msg"
  "msg/LinkMetrics.msg"
  "msg/PipelineMetrics.msg"
  "msg/RealtimeAck.msg"
  "msg/RealtimeAckMetrics.msg"
  "msg/SyncAck.msg"
  "msg/SyncRequest.msg"
  "msg/SyncStatus.msg"
  "msg/VoxelDelta.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
