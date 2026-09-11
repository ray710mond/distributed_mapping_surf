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
    EXPECT_EQ(acks.size(),3U); // stale retries still ACKed
    send(1,110,12,false,2);send(1,120,13,true,1);
    EXPECT_EQ(received.size(),5U); // retired epoch never reactivates
    exec.remove_node(receiver);exec.remove_node(probe);
  }
  rclcpp::shutdown();
}
