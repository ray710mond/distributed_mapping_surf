#define SURF_NODE_TEST
#include "../src/drone_data_receiver.cpp"
#include <gtest/gtest.h>
#include <thread>
TEST(Receiver, SpatialReorderingBacklogRetryAndEpochRejection) {
  rclcpp::init(0, nullptr);
  {
    auto receiver = std::make_shared<surf_humanoid::DroneDataReceiver>();
    auto probe = std::make_shared<rclcpp::Node>("temporal_receiver_probe");
    using Packet = surf_multirobot_msgs::msg::CompressedVoxelDelta;
    using Delta = surf_multirobot_msgs::msg::VoxelDelta;
    using Ack = surf_multirobot_msgs::msg::SyncAck;
    std::vector<surf_multirobot_msgs::msg::DeliveryMetrics> metrics;
    auto metric_sub = probe->create_subscription<surf_multirobot_msgs::msg::DeliveryMetrics>(
      "/humanoid/comm/delivery_metrics", 100,
      [&](const surf_multirobot_msgs::msg::DeliveryMetrics & m) {metrics.push_back(m);});
    std::vector<Delta> received; std::vector<Ack> acks;
    auto delta_pub = probe->create_publisher<Packet>("/humanoid/transport/realtime_tx", surf::comms::realtime_qos());
    auto backlog_pub = probe->create_publisher<Packet>("/humanoid/transport/sync_tx", surf::comms::sync_qos());
    auto output = probe->create_subscription<Delta>("/humanoid/comm/drone_voxel_delta",
      rclcpp::QoS(128).reliable().transient_local(), [&](const Delta & d) {received.push_back(d);});
    auto ack_sub = probe->create_subscription<Ack>("/humanoid/transport/sync_ack",
      rclcpp::QoS(10).reliable().transient_local(), [&](const Ack & a) {acks.push_back(a);});
    rclcpp::executors::SingleThreadedExecutor exec; exec.add_node(receiver); exec.add_node(probe);
    auto spin = [&]() {
      for (int i=0;i<40;++i) {exec.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(5));}
    };
    spin();
    auto send = [&](int x, uint64_t stamp, uint64_t version, bool backlog, uint64_t epoch=1) {
      Delta d; d.header.frame_id="map";d.header.stamp=probe->now();d.source_id="drone";
      d.map_epoch=epoch;d.version=version;d.operating_mode=4;d.chunk_count=1;d.resolution=.05;
      d.x={x};d.y={0};d.z={0};d.state={1};d.observation_time_ns={stamp};
      Packet p;ASSERT_TRUE(surf::comms::encode_delta(d,p,1).ok);p.traffic_class=backlog?2:1;p.transmit_stamp=probe->now();
      (backlog?backlog_pub:delta_pub)->publish(p);spin();
    };
    send(1,105,10,false); send(1,100,11,true); send(2,90,1,true); send(1,100,11,true);
    ASSERT_EQ(received.size(),4U);
    EXPECT_EQ(received[0].x.size(),1U);EXPECT_TRUE(received[1].x.empty());
    EXPECT_EQ(received[2].x.size(),1U);EXPECT_TRUE(received[3].x.empty());
    ASSERT_EQ(metrics.size(), 4U);
    EXPECT_GT(metrics[0].decode_latency_ms, 0);
    EXPECT_TRUE(std::isnan(metrics[0].end_to_end_latency_ms));
    EXPECT_EQ(metrics[0].accepted_voxel_count, 1U);
    EXPECT_GT(metrics[0].oldest_accepted_observation_age_s, 1000000);
    EXPECT_LT(metrics[0].sensor_to_receiver_ms, 1000);
    EXPECT_EQ(metrics[1].accepted_voxel_count, 0U);
    EXPECT_TRUE(std::isnan(metrics[1].mean_accepted_observation_age_s));
    EXPECT_EQ(acks.size(),3U); // stale retries still ACKed
    send(1,110,12,false,2);send(1,120,13,true,1);
    EXPECT_EQ(received.size(),5U); // retired epoch never reactivates
    exec.remove_node(receiver);exec.remove_node(probe);
  }
  rclcpp::shutdown();
}

TEST(Receiver, ReconstructsMeasuredFreePrefixFromOldBacklogOrigin) {
  rclcpp::init(0, nullptr);
  {
    auto receiver = std::make_shared<surf_humanoid::DroneDataReceiver>();
    auto probe = std::make_shared<rclcpp::Node>("ray_receiver_probe");
    using Delta = surf_multirobot_msgs::msg::VoxelDelta;
    using Packet = surf_multirobot_msgs::msg::CompressedVoxelDelta;
    std::vector<Delta> received;
    auto pub = probe->create_publisher<Packet>("/humanoid/transport/sync_tx", surf::comms::sync_qos());
    auto sub = probe->create_subscription<Delta>("/humanoid/comm/drone_voxel_delta",
      rclcpp::QoS(128).reliable().transient_local(), [&](const Delta & d) {received.push_back(d);});
    rclcpp::executors::SingleThreadedExecutor exec; exec.add_node(receiver); exec.add_node(probe);
    for (int i=0;i<20;++i) {exec.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(5));}
    Delta d; d.header.frame_id="map"; d.header.stamp=probe->now();
    d.source_id="drone"; d.map_epoch=77; d.version=1; d.operating_mode=4;
    d.chunk_count=1; d.resolution=.05; d.x={20};d.y={0};d.z={20};
    d.state={Delta::STATE_FREE}; d.observation_time_ns={123456789ULL};
    geometry_msgs::msg::Point origin, endpoint;
    origin.x=.025;origin.y=.025;origin.z=1.025;
    endpoint.x=2.025;endpoint.y=.025;endpoint.z=1.025;
    d.ray_flags={1};d.ray_origins={origin};d.ray_endpoints={endpoint};
    Packet p;ASSERT_TRUE(surf::comms::encode_delta(d,p,1).ok);
    p.traffic_class=2;p.transmit_stamp=probe->now();pub->publish(p);
    for (int i=0;i<40;++i) {exec.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(5));}
    ASSERT_FALSE(received.empty());
    const auto & result=received.back();
    EXPECT_GT(result.x.size(),1U);
    EXPECT_EQ(result.x.front(),15);
    EXPECT_EQ(result.x.back(),20);
    for (std::size_t i=1;i<result.x.size();++i) EXPECT_EQ(result.x[i],result.x[i-1]+1);
    exec.remove_node(receiver);exec.remove_node(probe);
  }
  rclcpp::shutdown();
}

TEST(Receiver, ReconstructsFreePrefixFromOccupiedHitOrigin) {
  rclcpp::init(0, nullptr);
  {
    auto receiver = std::make_shared<surf_humanoid::DroneDataReceiver>();
    auto probe = std::make_shared<rclcpp::Node>("occupied_origin_probe");
    using Delta = surf_multirobot_msgs::msg::VoxelDelta;
    using Packet = surf_multirobot_msgs::msg::CompressedVoxelDelta;
    std::vector<Delta> received;
    auto pub = probe->create_publisher<Packet>("/humanoid/transport/sync_tx", surf::comms::sync_qos());
    auto sub = probe->create_subscription<Delta>("/humanoid/comm/drone_voxel_delta",
      rclcpp::QoS(128).reliable().transient_local(), [&](const Delta & d) {received.push_back(d);});
    rclcpp::executors::SingleThreadedExecutor exec; exec.add_node(receiver); exec.add_node(probe);
    for (int i=0;i<20;++i) {exec.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(5));}
    Delta d; d.header.frame_id="map"; d.header.stamp=probe->now(); d.chunk_count=1;
    d.source_id="drone"; d.map_epoch=88; d.version=1; d.operating_mode=4; d.resolution=.05;
    d.sensor_origin.x=10.0; // Latest origin must not be used for this old hit.
    d.x={40}; d.y={0}; d.z={20}; d.state={Delta::STATE_OCCUPIED_STATIC};
    d.observation_time_ns={123456789ULL};
    geometry_msgs::msg::Point origin; origin.x=.025; origin.y=.025; origin.z=1.025;
    d.ray_flags={2}; d.ray_origins={origin}; d.ray_endpoints={geometry_msgs::msg::Point()};
    Packet p; ASSERT_TRUE(surf::comms::encode_delta(d,p,1).ok);
    p.traffic_class=2; p.transmit_stamp=probe->now(); pub->publish(p);
    for (int i=0;i<40;++i) {exec.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(5));}
    ASSERT_FALSE(received.empty());
    const auto & result=received.back();
    EXPECT_GT(result.x.size(),1U);
    EXPECT_EQ(result.x.front(),15);
    EXPECT_EQ(result.x.back(),40);
    EXPECT_EQ(result.state.back(),Delta::STATE_OCCUPIED_STATIC);
    for (std::size_t i=0;i+1<result.x.size();++i) EXPECT_EQ(result.state[i],Delta::STATE_FREE);
    exec.remove_node(receiver); exec.remove_node(probe);
  }
  rclcpp::shutdown();
}
