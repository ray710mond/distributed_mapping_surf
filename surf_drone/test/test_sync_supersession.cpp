#define SURF_NODE_TEST
#include "../src/drone_scan_sender.cpp"
#include <gtest/gtest.h>

TEST(InformationDelivery, TimeoutBacklogAndAckResolveDebtWithoutNewScans)
{
  rclcpp::init(0, nullptr);
  {
    auto sender = std::make_shared<surf_drone::DroneScanSender>(rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("filters.humanoid_mask.enabled", false),
      rclcpp::Parameter("delivery.ack_timeout_seconds", .2),
      rclcpp::Parameter("delivery.defer_seconds", .1),
      rclcpp::Parameter("capacity.development_bytes_per_second", 10000.)}));
    auto probe = std::make_shared<rclcpp::Node>("information_probe");
    using Packet = surf_multirobot_msgs::msg::CompressedVoxelDelta;
    using Ack = surf_multirobot_msgs::msg::SyncAck;
    std::vector<Packet> packets;
    std::vector<surf_multirobot_msgs::msg::AllocationMetrics> metrics;
    auto cloud_pub = probe->create_publisher<sensor_msgs::msg::PointCloud2>("/drone/points", rclcpp::SensorDataQoS());
    auto ack_pub = probe->create_publisher<Ack>("/drone/transport/sync_ack", rclcpp::QoS(10).reliable().transient_local());
    bool auto_ack = false;
    auto sub = probe->create_subscription<Packet>("/drone/transport/sync_tx", surf::comms::sync_qos(),
      [&](const Packet & p) {
        packets.push_back(p);
        if (auto_ack) {Ack a; a.source_id="drone"; a.map_epoch=p.map_epoch; a.version=p.version; ack_pub->publish(a);}
      });
    auto metric_sub = probe->create_subscription<surf_multirobot_msgs::msg::AllocationMetrics>(
      "/drone/comm/allocation_metrics", 100,
      [&](const surf_multirobot_msgs::msg::AllocationMetrics & m) {metrics.push_back(m);});
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(sender); executor.add_node(probe);
    auto spin = [&](double seconds) {
      auto until = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
      while (std::chrono::steady_clock::now() < until) {
        executor.spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
    };
    spin(.3);
    sensor_msgs::msg::PointCloud2 cloud; cloud.header.frame_id = "map"; cloud.header.stamp = probe->now();
    sensor_msgs::PointCloud2Modifier mod(cloud); mod.setPointCloud2FieldsByString(1, "xyz"); mod.resize(1);
    sensor_msgs::PointCloud2Iterator<float> x(cloud,"x"), y(cloud,"y"), z(cloud,"z");
    *x=2; *y=0; *z=1; cloud_pub->publish(cloud); spin(3.0);
    ASSERT_FALSE(packets.empty());
    EXPECT_FALSE(packets.back().full_refresh); EXPECT_EQ(packets.back().operating_mode,4);
    Ack ack; ack.source_id="drone"; ack.map_epoch=packets.back().map_epoch; ack.version=packets.back().version;
    ack_pub->publish(ack);
    auto_ack = true; spin(2.0);
    ASSERT_FALSE(metrics.empty());
    EXPECT_EQ(metrics.back().pending_count[0]+metrics.back().pending_count[1],0U);
    const auto packets_before_recovery = packets.size();
    auto recovery_pub = probe->create_publisher<surf_multirobot_msgs::msg::SyncRequest>(
      "/drone/transport/sync_request",rclcpp::QoS(10).reliable().transient_local());
    recovery_pub->publish(surf_multirobot_msgs::msg::SyncRequest{}); // No source/epoch knowledge.
    spin(2.0);
    EXPECT_GT(packets.size(),packets_before_recovery);
    EXPECT_EQ(metrics.back().pending_count[0]+metrics.back().pending_count[1],0U);
    const auto revision = metrics.back().configuration_revision;
    EXPECT_TRUE(sender->set_parameters_atomically({rclcpp::Parameter("allocation.q", std::vector<double>{2,0,0,2})}).successful);
    spin(.2); EXPECT_EQ(metrics.back().configuration_revision, revision+1);
    EXPECT_FALSE(sender->set_parameters_atomically({rclcpp::Parameter("allocation.r", std::vector<double>{-1,0,0,1})}).successful);
    EXPECT_FALSE(sender->set_parameters_atomically({rclcpp::Parameter("allocation.a", std::vector<double>{1,2})}).successful);
    spin(.2); EXPECT_EQ(metrics.back().configuration_revision, revision+1);
    double total=0, capacity_integral=0;
    for (const auto & m : metrics) {
      total += m.wire_bytes[0]+m.wire_bytes[1]; capacity_integral += m.usable_capacity*m.actual_dt;
      EXPECT_LE(m.allocated_rate[0]+m.allocated_rate[1],m.usable_capacity+1e-6);
      EXPECT_LE(total,capacity_integral+1e-6);
    }
    executor.remove_node(sender); executor.remove_node(probe);
  }
  rclcpp::shutdown();
}

TEST(InformationDelivery, ReoccupyingClearedStaticPriorCreatesNewDebt)
{
  rclcpp::init(0, nullptr);
  {
    auto sender = std::make_shared<surf_drone::DroneScanSender>(rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("filters.humanoid_mask.enabled", false),
      rclcpp::Parameter("filters.clear_min_misses", int64_t(1))}));
    auto probe = std::make_shared<rclcpp::Node>("prior_reoccupation_probe");
    using Packet = surf_multirobot_msgs::msg::CompressedVoxelDelta;
    using Delta = surf_multirobot_msgs::msg::VoxelDelta;
    auto cloud_pub = probe->create_publisher<sensor_msgs::msg::PointCloud2>("/drone/points", rclcpp::SensorDataQoS());
    auto prior_pub = probe->create_publisher<sensor_msgs::msg::PointCloud2>("/drone/bonxai/static_occupied_voxels", rclcpp::QoS(1));
    bool restored = false;
    uint64_t restoration_stamp = 0;
    auto receive = [&](const Packet & p) {
      Delta d;
      ASSERT_TRUE(surf::comms::decode_delta(p,d,1024*1024).ok);
      for (std::size_t i=0;i<d.x.size();++i) {
        if (d.x[i]==40 && d.y[i]==0 && d.z[i]==20 && d.state[i]==Delta::STATE_OCCUPIED_STATIC &&
          restoration_stamp && d.observation_time_ns[i]>=restoration_stamp) restored=true;
      }
    };
    auto delta_sub = probe->create_subscription<Packet>("/drone/transport/realtime_tx",surf::comms::realtime_qos(),receive);
    auto backlog_sub = probe->create_subscription<Packet>("/drone/transport/sync_tx",surf::comms::sync_qos(),receive);
    rclcpp::executors::SingleThreadedExecutor exec;exec.add_node(sender);exec.add_node(probe);
    auto spin = [&](double seconds) {
      auto end=std::chrono::steady_clock::now()+std::chrono::duration<double>(seconds);
      while(std::chrono::steady_clock::now()<end) {exec.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(5));}
    };
    auto cloud = [&](float px,float pz) {
      sensor_msgs::msg::PointCloud2 c;c.header.frame_id="map";c.header.stamp=probe->now();
      sensor_msgs::PointCloud2Modifier mod(c);mod.setPointCloud2FieldsByString(1,"xyz");mod.resize(1);
      sensor_msgs::PointCloud2Iterator<float> x(c,"x"),y(c,"y"),z(c,"z");*x=px;*y=0;*z=pz;return c;
    };
    spin(.3);prior_pub->publish(cloud(2,1));spin(.2);
    cloud_pub->publish(cloud(2,1));spin(.3); // Prior occupancy needs no radio update.
    cloud_pub->publish(cloud(3,1.5));spin(.3); // Clearing ray crosses prior voxel A.
    auto again=cloud(2,1);restoration_stamp=surf_drone::stamp_to_nanoseconds(again.header.stamp);
    cloud_pub->publish(again);spin(4);
    EXPECT_TRUE(restored);
    exec.remove_node(sender);exec.remove_node(probe);
  }
  rclcpp::shutdown();
}
