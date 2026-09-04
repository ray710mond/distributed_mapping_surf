#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "surf_multirobot_msgs/msg/compressed_voxel_delta.hpp"
#include "surf_multirobot_msgs/msg/voxel_delta.hpp"

namespace surf::comms
{

struct CodecResult
{
  bool ok{false};
  std::string error;
};

CodecResult encode_delta(
  const surf_multirobot_msgs::msg::VoxelDelta & delta,
  surf_multirobot_msgs::msg::CompressedVoxelDelta & output,
  int compression_level = 1);

CodecResult decode_delta(
  const surf_multirobot_msgs::msg::CompressedVoxelDelta & input,
  surf_multirobot_msgs::msg::VoxelDelta & delta,
  std::size_t maximum_uncompressed_bytes = 64U * 1024U * 1024U);

}  // namespace surf::comms
