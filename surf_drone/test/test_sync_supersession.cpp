#define SURF_NODE_TEST
#include "../src/drone_scan_sender.cpp"
#include <gtest/gtest.h>
#include <set>

TEST(SyncSupersession, NewSnapshotsContinueWithoutAckAndLateAckCannotClearReplacement)
{
  rclcpp::init(0, nullptr);
  {
    auto sender = std::make_shared<surf_drone::DroneScanSender>(
      rclcpp::NodeOptions().parameter_overrides({
        rclcpp::Parameter("sync_interval_seconds", 0.1),
        rclcpp::Parameter("transport.sync_max_interval_seconds", 0.1),
        rclcpp::Parameter("transport.sync_ack_timeout_seconds", 0.1),
        rclcpp::Parameter("transport.sync_ack_margin_seconds", 0.0),
        rclcpp::Parameter("filters.humanoid_mask.enabled", false),
        rclcpp::Parameter("adaptive.manual_mode", int64_t(1))}));
    auto probe = std::make_shared<rclcpp::Node>("sync_supersession_probe");
    using Packet = surf_multirobot_msgs::msg::CompressedVoxelDelta;
    using Ack = surf_multirobot_msgs::msg::SyncAck;
    using Status = surf_multirobot_msgs::msg::SyncStatus;
    std::vector<Packet> packets;
    std::vector<Status> statuses;
    auto cloud_pub = probe->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/drone/points", rclcpp::SensorDataQoS());
    auto ack_pub = probe->create_publisher<Ack>(
      "/drone/transport/sync_ack", rclcpp::QoS(10).reliable().transient_local());
    auto sub = probe->create_subscription<Packet>("/drone/transport/sync_tx",
      surf::comms::sync_qos(), [&](const Packet & p) {packets.push_back(p);});
    auto status_sub = probe->create_subscription<Status>("/drone/comm/sync_status",
      rclcpp::QoS(100).reliable().transient_local(),
      [&](const Status & s) {statuses.push_back(s);});
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(sender); executor.add_node(probe);
    auto spin_for = [&](double seconds) {
        auto until = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
        while (std::chrono::steady_clock::now() < until) {
          executor.spin_some();
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
      };
    spin_for(0.5);
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = "map";  // Identity map transform, no hardware/TF needed.
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(1);
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
    *x = 2.0F; *y = 0.0F; *z = 1.0F;
    for (int i = 0; i < 40; ++i) {
      cloud.header.stamp = probe->now();
      cloud_pub->publish(cloud);
      spin_for(0.05);
    }
    spin_for(0.3);  // Finish the last input; retries now refer to a fixed latest generation.
    std::set<uint64_t> versions;
    for (const auto & p : packets) {versions.insert(p.version);}
    EXPECT_GE(versions.size(), 3U);  // Old ACK gate produced only one generation.
    if (!packets.empty()) {
      Ack ack;
      ack.source_id = "drone"; ack.map_epoch = packets.back().map_epoch;
      ack.version = *versions.begin();
      ack_pub->publish(ack);
      spin_for(0.3);
      EXPECT_EQ(std::count_if(statuses.begin(), statuses.end(), [](const Status & s) {
          return s.state == Status::STATE_ACKNOWLEDGED;
        }), 0);
      ack.version = *versions.rbegin();
      ack_pub->publish(ack);
      spin_for(0.3);
      EXPECT_EQ(std::count_if(statuses.begin(), statuses.end(), [&](const Status & s) {
          return s.state == Status::STATE_ACKNOWLEDGED && s.version == ack.version;
        }), 1);
    }
    executor.remove_node(sender); executor.remove_node(probe);
  }
  rclcpp::shutdown();
}
