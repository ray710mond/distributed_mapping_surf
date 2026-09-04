#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>

#include "surf_multirobot_msgs/msg/voxel_delta.hpp"
#include "surf_multirobot_comms/voxel_codec.hpp"

namespace
{

TEST(VoxelCodec, RoundTripsEveryOperationAndSortsDeterministically)
{
  surf_multirobot_msgs::msg::VoxelDelta input;
  input.source_id = "drone";
  input.map_epoch = 42U;
  input.version = 7U;
  input.base_version = 6U;
  input.chunk_index = 2U;
  input.chunk_count = 4U;
  input.resolution = 0.2F;
  input.x = {100, -3, 0, 100};
  input.y = {-200, 4, 0, -200};
  input.z = {5, -6, 0, 5};
  input.state = {
    surf_multirobot_msgs::msg::VoxelDelta::STATE_OCCUPIED_STATIC,
    surf_multirobot_msgs::msg::VoxelDelta::STATE_OCCUPIED_DYNAMIC,
    surf_multirobot_msgs::msg::VoxelDelta::STATE_FREE,
    surf_multirobot_msgs::msg::VoxelDelta::STATE_DELETE};
  input.observation_time_ns = {1000000000U, 2000000000U, 3000000000U, 4000000000U};

  surf_multirobot_msgs::msg::CompressedVoxelDelta wire;
  const auto encoded = surf::comms::encode_delta(input, wire, 1);
  ASSERT_TRUE(encoded.ok) << encoded.error;
  EXPECT_EQ(wire.codec, "raw-svd2");
  EXPECT_LT(wire.payload.size(), 150U);

  surf_multirobot_msgs::msg::VoxelDelta output;
  const auto decoded = surf::comms::decode_delta(wire, output);
  ASSERT_TRUE(decoded.ok) << decoded.error;
  EXPECT_EQ(wire.chunk_index, 2U);
  EXPECT_EQ(wire.chunk_count, 4U);
  EXPECT_EQ(output.chunk_index, 2U);
  EXPECT_EQ(output.chunk_count, 4U);
  EXPECT_EQ(output.source_id, input.source_id);
  EXPECT_EQ(output.map_epoch, input.map_epoch);
  EXPECT_EQ(output.version, input.version);
  ASSERT_EQ(output.x.size(), input.x.size());
  EXPECT_EQ(output.observation_time_ns.size(), input.observation_time_ns.size());
  EXPECT_TRUE(std::is_permutation(
    output.observation_time_ns.begin(), output.observation_time_ns.end(),
    input.observation_time_ns.begin(), input.observation_time_ns.end()));
  for (std::size_t index = 1; index < output.x.size(); ++index) {
    EXPECT_TRUE(output.x[index - 1] < output.x[index] ||
      (output.x[index - 1] == output.x[index] && output.y[index - 1] <= output.y[index]));
  }
}

TEST(VoxelCodec, RejectsMismatchedArraysAndCorruption)
{
  surf_multirobot_msgs::msg::VoxelDelta bad;
  bad.x.push_back(1);
  surf_multirobot_msgs::msg::CompressedVoxelDelta wire;
  EXPECT_FALSE(surf::comms::encode_delta(bad, wire).ok);

  surf_multirobot_msgs::msg::VoxelDelta valid;
  valid.x = {1};
  valid.y = {2};
  valid.z = {3};
  valid.state = {surf_multirobot_msgs::msg::VoxelDelta::STATE_FREE};
  valid.observation_time_ns = {123U};
  ASSERT_TRUE(surf::comms::encode_delta(valid, wire).ok);
  wire.payload.at(0) ^= 0x5aU;
  surf_multirobot_msgs::msg::VoxelDelta output;
  EXPECT_FALSE(surf::comms::decode_delta(wire, output).ok);
}

TEST(VoxelCodec, UsesZstdWhenItReducesPayloadSize)
{
  surf_multirobot_msgs::msg::VoxelDelta input;
  for (int32_t x = 0; x < 1000; ++x) {
    input.x.push_back(x);
    input.y.push_back(0);
    input.z.push_back(0);
    input.state.push_back(surf_multirobot_msgs::msg::VoxelDelta::STATE_OCCUPIED_STATIC);
    input.observation_time_ns.push_back(123456789U + static_cast<uint64_t>(x));
  }

  surf_multirobot_msgs::msg::CompressedVoxelDelta wire;
  const auto encoded = surf::comms::encode_delta(input, wire, 1);
  ASSERT_TRUE(encoded.ok) << encoded.error;
  EXPECT_EQ(wire.codec, "zstd-svd2");
  EXPECT_LT(wire.payload.size(), wire.uncompressed_bytes);

  surf_multirobot_msgs::msg::VoxelDelta output;
  const auto decoded = surf::comms::decode_delta(wire, output);
  ASSERT_TRUE(decoded.ok) << decoded.error;
  EXPECT_EQ(output.x.size(), input.x.size());
}

}  // namespace
