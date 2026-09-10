#include <gtest/gtest.h>
#include "rclcpp/serialization.hpp"
#include "surf_multirobot_msgs/msg/compressed_voxel_delta.hpp"
#include "network_bridge/subscription_manager.hpp"

class ChunkQueueTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("chunk_queue_test");
  static std::shared_ptr<rclcpp::SerializedMessage> chunk(uint8_t index)
  {
    auto m = std::make_shared<rclcpp::SerializedMessage>(1200);
    auto & raw = m->get_rcl_serialized_message();
    raw.buffer_length = 1200;
    std::fill(raw.buffer, raw.buffer + 1200, index);
    return m;
  }
};

TEST_F(ChunkQueueTest, ReproducesLegacyOverwriteDuringSendTimerStall)
{
  SubscriptionManager latest(node, "/test", "");
  for (uint8_t i = 0; i < 6; ++i) {
    latest.callback(chunk(i));
  }
  bool valid = false;
  EXPECT_EQ(latest.get_data(valid)[0], 5);
  EXPECT_TRUE(valid);
  EXPECT_FALSE(latest.has_data());
}

TEST_F(ChunkQueueTest, PreservesEveryChunkAfterSendTimerStall)
{
  SubscriptionManager fifo(node, "/test", "", 1, false, 256, 307200);
  for (int batch = 0; batch < 30; ++batch) {
    for (uint8_t i = 0; i < 6; ++i) {
      fifo.callback(chunk(i));
    }
  }
  for (int batch = 0; batch < 30; ++batch) {
    for (uint8_t i = 0; i < 6; ++i) {
      ASSERT_TRUE(fifo.has_data());
      bool valid = false;
      auto & data = fifo.get_data(valid);
      ASSERT_TRUE(valid);
      EXPECT_EQ(data.size(), 1200U);
      EXPECT_EQ(data[0], i);
    }
  }
  EXPECT_FALSE(fifo.has_data());
  bool valid = true;
  fifo.get_data(valid);
  EXPECT_FALSE(valid);
}

TEST_F(ChunkQueueTest, OverflowDoesNotOverwriteAndBudgetIsReclaimed)
{
  for (bool byte_limit : {false, true}) {
    SubscriptionManager fifo(node, "/test", "", 1, true,
      byte_limit ? 100 : 2, byte_limit ? 2400 : 12000);
    fifo.callback(chunk(0)); fifo.callback(chunk(1)); fifo.callback(chunk(2));
    bool valid = false;
    EXPECT_EQ(fifo.get_data(valid)[0], 0);
    fifo.callback(chunk(3));
    EXPECT_EQ(fifo.get_data(valid)[0], 1);
    EXPECT_EQ(fifo.get_data(valid)[0], 3);
    EXPECT_FALSE(fifo.has_data());  // Stale replay must never apply to queued chunks.
  }
}

TEST_F(ChunkQueueTest, LatestSnapshotRetainsAllNewChunksAndPurgesOldBacklog)
{
  using Packet = surf_multirobot_msgs::msg::CompressedVoxelDelta;
  SubscriptionManager fifo(node, "/sync", "", 1, false, 256, 1048576, true);
  rclcpp::Serialization<Packet> serializer;
  auto send = [&](uint64_t version, uint32_t index, uint64_t epoch = 7) {
      Packet packet;
      packet.source_id = "drone"; packet.map_epoch = epoch;
      packet.full_refresh = true; packet.traffic_class = Packet::TRAFFIC_SYNC;
      packet.version = version; packet.chunk_index = index; packet.chunk_count = 3;
      auto message = std::make_shared<rclcpp::SerializedMessage>();
      serializer.serialize_message(&packet, message.get());
      fifo.callback(message);
    };
  send(159, 0); send(159, 1); send(159, 2);  // Disconnected: no send ticks.
  send(270, 2); send(159, 0); send(270, 0); send(270, 1);
  for (uint32_t index : {2U, 0U, 1U}) {
    bool valid = false;
    const auto & bytes = fifo.get_data(valid);
    ASSERT_TRUE(valid);
    rclcpp::SerializedMessage message(bytes.size());
    std::copy(bytes.begin(), bytes.end(), message.get_rcl_serialized_message().buffer);
    message.get_rcl_serialized_message().buffer_length = bytes.size();
    Packet packet;
    serializer.deserialize_message(&message, &packet);
    EXPECT_EQ(packet.version, 270U);
    EXPECT_EQ(packet.chunk_index, index);
  }
  EXPECT_FALSE(fifo.has_data());
  send(159, 1);  // Watermark survives queue drain.
  EXPECT_FALSE(fifo.has_data());
  send(1, 0, 8);  // A new sender epoch can restart numbering.
  EXPECT_TRUE(fifo.has_data());
}
