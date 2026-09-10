// Run the real node through ROS, using the production codec and subscriptions.
#define SURF_NODE_TEST
#include "../src/drone_data_receiver.cpp"
#include <gtest/gtest.h>
#include <functional>
#include <thread>

using Packet = surf_multirobot_msgs::msg::CompressedVoxelDelta;
using Delta = surf_multirobot_msgs::msg::VoxelDelta;
using Ack = surf_multirobot_msgs::msg::SyncAck;
using Metrics = surf_multirobot_msgs::msg::DeliveryMetrics;
using Request = surf_multirobot_msgs::msg::SyncRequest;

class SyncRecoveryTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  std::shared_ptr<surf_humanoid::DroneDataReceiver> receiver;
  rclcpp::Node::SharedPtr probe;
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::Publisher<Packet>::SharedPtr realtime, sync;
  rclcpp::Subscription<Delta>::SharedPtr output;
  rclcpp::Subscription<Ack>::SharedPtr acknowledgments;
  rclcpp::Subscription<Metrics>::SharedPtr telemetry;
  rclcpp::Subscription<Request>::SharedPtr requests;
  std::vector<Delta> published;
  std::vector<Ack> acks;
  std::vector<Metrics> metrics;
  std::vector<Request> recovery_requests;

  bool wait(const std::function<bool()> & predicate, double seconds = 3.0)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(seconds);
    do {
      executor.spin_some();
      if (predicate()) {return true;}
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    } while (std::chrono::steady_clock::now() < deadline);
    return predicate();
  }
  void SetUp() override
  {
    receiver = std::make_shared<surf_humanoid::DroneDataReceiver>(
      rclcpp::NodeOptions().parameter_overrides({
        rclcpp::Parameter("maximum_realtime_history_bytes", int64_t(1024))}));
    probe = std::make_shared<rclcpp::Node>("sync_recovery_probe");
    realtime = probe->create_publisher<Packet>("/humanoid/transport/realtime_tx",
      surf::comms::realtime_qos());
    sync = probe->create_publisher<Packet>("/humanoid/transport/sync_tx", surf::comms::sync_qos());
    output = probe->create_subscription<Delta>("/humanoid/comm/drone_voxel_delta",
      rclcpp::QoS(128).reliable().transient_local(),
      [this](const Delta & d) {if (d.version > 0) {published.push_back(d);}});
    acknowledgments = probe->create_subscription<Ack>("/humanoid/transport/sync_ack",
      rclcpp::QoS(50).reliable().transient_local(), [this](const Ack & a) {acks.push_back(a);});
    telemetry = probe->create_subscription<Metrics>("/humanoid/comm/delivery_metrics", 100,
      [this](const Metrics & m) {metrics.push_back(m);});
    requests = probe->create_subscription<Request>("/humanoid/transport/sync_request",
      rclcpp::QoS(50).reliable().transient_local(),
      [this](const Request & r) {recovery_requests.push_back(r);});
    executor.add_node(receiver); executor.add_node(probe);
    ASSERT_TRUE(wait([this]() {return realtime->get_subscription_count() &&
        sync->get_subscription_count() && telemetry->get_publisher_count();}));
    send(10, true, 0, 1, 1, Delta::STATE_OCCUPIED_STATIC);
    ASSERT_TRUE(wait([this]() {return !acks.empty() && !published.empty();}));
  }
  void TearDown() override
  {
    executor.remove_node(receiver); executor.remove_node(probe);
    receiver.reset(); probe.reset();
  }
  void send(uint64_t version, bool full, uint32_t index = 0, uint32_t count = 1,
    int x = 1, uint8_t state = Delta::STATE_OCCUPIED_STATIC)
  {
    Delta d;
    d.source_id = "drone"; d.map_epoch = 7; d.version = version;
    d.base_version = version - 1; d.full_refresh = full;
    d.chunk_index = index; d.chunk_count = count; d.resolution = 0.05F;
    d.header.frame_id = "map"; d.header.stamp = probe->now();
    d.x = {x}; d.y = {0}; d.z = {0}; d.state = {state};
    d.observation_time_ns = {version * 100};
    Packet packet;
    ASSERT_TRUE(surf::comms::encode_delta(d, packet).ok);
    packet.traffic_class = full ? Packet::TRAFFIC_SYNC : Packet::TRAFFIC_REALTIME;
    packet.transmit_stamp = probe->now();
    const auto before = metrics.size();
    (full ? sync : realtime)->publish(packet);
    ASSERT_TRUE(wait([this, before]() {return metrics.size() > before;}));
  }
  std::map<int, uint8_t> states(const Delta & d)
  {
    std::map<int, uint8_t> result;
    for (std::size_t i = 0; i < d.x.size(); ++i) {result[d.x[i]] = d.state[i];}
    return result;
  }
};

TEST_F(SyncRecoveryTest, DelayedSnapshotAfterRealtimeAdvancesRecoversWithoutRollback)
{
  // Model Wi-Fi loss while HaLow continues, then release a delayed full snapshot.
  send(160, false, 0, 1, 1, Delta::STATE_FREE);
  send(266, false, 0, 1, 3, Delta::STATE_DELETE);
  send(159, true, 1, 2, 2);
  send(159, true, 0, 2, 1);
  ASSERT_TRUE(wait([this]() {return acks.back().version == 159;}));
  EXPECT_TRUE(published.back().full_refresh);
  EXPECT_EQ(published.back().version, 266U);
  EXPECT_EQ(states(published.back())[1], Delta::STATE_FREE);
  EXPECT_EQ(states(published.back())[2], Delta::STATE_OCCUPIED_STATIC);
  EXPECT_EQ(states(published.back())[3], Delta::STATE_DELETE);
}

TEST_F(SyncRecoveryTest, NewSnapshotSupersedesPartialAndIgnoresLateOldChunks)
{
  send(20, true, 0, 2);
  send(25, false, 0, 1, 1, Delta::STATE_DELETE);
  send(30, true, 1, 2, 3);
  send(20, true, 1, 2, 2);  // Must not replace the version-30 assembly.
  EXPECT_EQ(metrics.back().rejection_reason, "superseded full refresh");
  send(31, false, 0, 1, 2, Delta::STATE_FREE);
  send(30, true, 0, 2, 2);
  ASSERT_TRUE(wait([this]() {return acks.back().version == 30;}));
  EXPECT_EQ(published.back().version, 31U);
  EXPECT_EQ(states(published.back())[2], Delta::STATE_FREE);
  EXPECT_EQ(states(published.back())[3], Delta::STATE_OCCUPIED_STATIC);
  const auto publications = published.size(), acknowledgments_before = acks.size();
  send(30, true, 0, 2, 2);  // Recover an ACK lost on the return path.
  ASSERT_TRUE(wait([this, acknowledgments_before]() {return acks.size() > acknowledgments_before;}));
  EXPECT_EQ(published.size(), publications);
  EXPECT_EQ(acks.back().version, 30U);
}

TEST_F(SyncRecoveryTest, EvictedHistoryRequestsFreshSnapshotInsteadOfRollingBack)
{
  for (uint64_t version = 11; version <= 25; ++version) {send(version, false);}
  send(12, true);
  EXPECT_FALSE(metrics.back().accepted);
  EXPECT_EQ(metrics.back().rejection_reason, "full refresh predates retained realtime history");
  ASSERT_TRUE(wait([this]() {return !recovery_requests.empty();}));
  EXPECT_EQ(recovery_requests.back().last_version, 25U);
  send(26, true);
  ASSERT_TRUE(wait([this]() {return acks.back().version == 26;}));
  EXPECT_EQ(published.back().version, 26U);
}
