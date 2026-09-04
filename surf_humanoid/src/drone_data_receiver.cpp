#include <algorithm>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"

#include "surf_multirobot_msgs/msg/compressed_voxel_delta.hpp"
#include "surf_multirobot_msgs/msg/delivery_metrics.hpp"
#include "surf_multirobot_msgs/msg/realtime_ack.hpp"
#include "surf_multirobot_msgs/msg/sync_ack.hpp"
#include "surf_multirobot_msgs/msg/sync_request.hpp"
#include "surf_multirobot_msgs/msg/voxel_delta.hpp"
#include "surf_multirobot_comms/qos_profiles.hpp"
#include "surf_multirobot_comms/voxel_codec.hpp"

namespace surf_humanoid
{

class DroneDataReceiver : public rclcpp::Node
{
public:
  DroneDataReceiver()
  : Node("drone_data_receiver")
  {
    robot_name_ = declare_parameter<std::string>("robot_name", "humanoid");
    realtime_topic_ = declare_parameter<std::string>(
      "realtime_topic", "/" + robot_name_ + "/transport/realtime_tx");
    realtime_ack_topic_ = declare_parameter<std::string>(
      "realtime_ack_topic", "/" + robot_name_ + "/transport/realtime_ack");
    sync_topic_ = declare_parameter<std::string>(
      "sync_topic", "/" + robot_name_ + "/transport/sync_tx");
    sync_ack_topic_ = declare_parameter<std::string>(
      "sync_ack_topic", "/" + robot_name_ + "/transport/sync_ack");
    sync_request_topic_ = declare_parameter<std::string>(
      "sync_request_topic", "/" + robot_name_ + "/transport/sync_request");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/" + robot_name_ + "/comm/drone_voxel_delta");
    metrics_topic_ = declare_parameter<std::string>(
      "metrics_topic", "/" + robot_name_ + "/comm/delivery_metrics");
    maximum_uncompressed_bytes_ = static_cast<std::size_t>(std::max<int64_t>(
      1024, declare_parameter<int64_t>("maximum_uncompressed_bytes", 64 * 1024 * 1024)));
    peer_odometry_topic_ = declare_parameter<std::string>(
      "peer_odometry_topic", "/humanoid/transport/drone_odometry");
    peer_box_size_x_ = std::max(
      0.0, declare_parameter<double>("peer_bounding_box.model_size_x", 0.70));
    peer_box_size_y_ = std::max(
      0.0, declare_parameter<double>("peer_bounding_box.model_size_y", 0.70));
    peer_box_size_z_ = std::max(
      0.0, declare_parameter<double>("peer_bounding_box.model_size_z", 0.40));
    peer_box_center_z_ = declare_parameter<double>(
      "peer_bounding_box.model_center_z", 0.0);
    peer_box_padding_ = std::max(
      0.0, declare_parameter<double>("peer_bounding_box.padding", 0.20));
    peer_box_max_age_ms_ = std::max(
      0.0, declare_parameter<double>("peer_bounding_box.max_age_ms", 150.0));

    // Retain more than one 10-second synchronization interval at the nominal
    // 10 Hz delta rate. Bonxai may finish subscribing after a cached sync is
    // decoded during startup, and must still receive that full refresh before
    // the subsequent real-time deltas.
    publisher_ = create_publisher<surf_multirobot_msgs::msg::VoxelDelta>(
      output_topic_, rclcpp::QoS(rclcpp::KeepLast(128)).reliable().transient_local());
    metrics_publisher_ = create_publisher<surf_multirobot_msgs::msg::DeliveryMetrics>(
      metrics_topic_, rclcpp::QoS(10));
    realtime_ack_publisher_ = create_publisher<surf_multirobot_msgs::msg::RealtimeAck>(
      realtime_ack_topic_, rclcpp::QoS(1).best_effort().durability_volatile());
    sync_ack_publisher_ = create_publisher<surf_multirobot_msgs::msg::SyncAck>(
      sync_ack_topic_, rclcpp::QoS(10).reliable().transient_local());
    sync_request_publisher_ = create_publisher<surf_multirobot_msgs::msg::SyncRequest>(
      sync_request_topic_, rclcpp::QoS(10).reliable().transient_local());
    realtime_subscription_ = create_subscription<
      surf_multirobot_msgs::msg::CompressedVoxelDelta>(
      realtime_topic_, surf::comms::realtime_qos(),
      std::bind(&DroneDataReceiver::receive, this, std::placeholders::_1));
    sync_subscription_ = create_subscription<surf_multirobot_msgs::msg::CompressedVoxelDelta>(
      sync_topic_, surf::comms::sync_qos(),
      std::bind(&DroneDataReceiver::receive, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "%s receiver: [%s, %s] -> %s",
      robot_name_.c_str(), realtime_topic_.c_str(), sync_topic_.c_str(), output_topic_.c_str());
    RCLCPP_INFO(get_logger(),
      "Peer state %s; configured box %.2f x %.2f x %.2fm + %.2fm padding (max age %.0fms)",
      peer_odometry_topic_.c_str(), peer_box_size_x_, peer_box_size_y_, peer_box_size_z_,
      peer_box_padding_, peer_box_max_age_ms_);
  }

private:
  void request_sync(
    const surf_multirobot_msgs::msg::CompressedVoxelDelta & packet,
    uint64_t last_version, const std::string & reason)
  {
    const auto request_time = std::chrono::steady_clock::now();
    if (last_sync_request_time_ != std::chrono::steady_clock::time_point{} &&
      std::chrono::duration<double>(request_time - last_sync_request_time_).count() < 1.0)
    {
      return;
    }
    last_sync_request_time_ = request_time;
    surf_multirobot_msgs::msg::SyncRequest request;
    request.header.stamp = now();
    request.source_id = packet.source_id;
    request.map_epoch = packet.map_epoch;
    request.last_version = last_version;
    request.reason = reason;
    sync_request_publisher_->publish(request);
  }

  struct SourceState
  {
    struct Assembly {
      uint64_t version{0U};
      uint32_t count{0U};
      std::vector<surf_multirobot_msgs::msg::VoxelDelta> chunks;
      std::vector<bool> received;
    } realtime_assembly, sync_assembly;
    std::map<uint64_t, surf_multirobot_msgs::msg::VoxelDelta> deferred_realtime;
    uint64_t epoch{0U};
    uint64_t last_version{0U};
    uint64_t last_full_refresh_version{0U};
    bool awaiting_full_refresh{true};
    bool initialized{false};
  };

  void publish_epoch_reset(
    const surf_multirobot_msgs::msg::CompressedVoxelDelta & packet)
  {
    surf_multirobot_msgs::msg::VoxelDelta reset;
    reset.header = packet.header;
    reset.source_id = packet.source_id;
    reset.map_epoch = packet.map_epoch;
    reset.version = 0U;
    reset.base_version = 0U;
    reset.operating_mode = packet.operating_mode;
    reset.full_refresh = true;
    reset.resolution = packet.resolution;
    reset.sensor_origin = packet.sensor_origin;
    publisher_->publish(reset);
  }

  void receive(const surf_multirobot_msgs::msg::CompressedVoxelDelta::SharedPtr packet)
  {
    const auto receive_start = std::chrono::steady_clock::now();
    surf_multirobot_msgs::msg::DeliveryMetrics metrics;
    metrics.header.stamp = now();
    metrics.sensor_stamp = packet->header.stamp;
    metrics.transmit_stamp = packet->transmit_stamp;
    metrics.source_id = packet->source_id;
    metrics.map_epoch = packet->map_epoch;
    metrics.version = packet->version;
    metrics.traffic_class = packet->traffic_class;
    metrics.voxel_count = packet->voxel_count;
    metrics.chunk_index = packet->chunk_index;
    metrics.chunk_count = packet->chunk_count == 0U ? 1U : packet->chunk_count;
    rclcpp::Serialization<surf_multirobot_msgs::msg::CompressedVoxelDelta> serializer;
    rclcpp::SerializedMessage serialized;
    serializer.serialize_message(packet.get(), &serialized);
    metrics.wire_bytes = static_cast<uint32_t>(serialized.size());
    const rclcpp::Time received(metrics.header.stamp);
    const rclcpp::Time sensed(packet->header.stamp);
    const rclcpp::Time transmitted(packet->transmit_stamp);
    metrics.sender_to_receiver_ms = static_cast<float>(
      (received - transmitted).seconds() * 1000.0);
    metrics.sensor_to_receiver_ms = static_cast<float>(
      (received - sensed).seconds() * 1000.0);
    // Retain the original field for compatibility. It historically measured
    // sensor-to-receiver time rather than network-only transport.
    metrics.transport_latency_ms = metrics.sensor_to_receiver_ms;
    auto & source = sources_[packet->source_id];
    if (!source.initialized || source.epoch != packet->map_epoch) {
      if (source.initialized) {
        RCLCPP_WARN(get_logger(), "Map epoch changed for %s; awaiting refresh from new sender",
          packet->source_id.c_str());
      }
      publish_epoch_reset(*packet);
      source = SourceState{};
      source.epoch = packet->map_epoch;
      source.awaiting_full_refresh = true;
      source.initialized = true;
    }

    if (source.awaiting_full_refresh && !packet->full_refresh) {
      request_sync(*packet, source.last_version, "awaiting initial full refresh");
      metrics.accepted = false;
      metrics.rejection_reason = "awaiting full refresh";
      metrics.decode_latency_ms = static_cast<float>(std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - receive_start).count());
      metrics.receiver_processing_ms = metrics.decode_latency_ms;
      metrics.end_to_end_latency_ms =
        metrics.sensor_to_receiver_ms + metrics.decode_latency_ms;
      metrics_publisher_->publish(metrics);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Waiting for a full refresh from %s", packet->source_id.c_str());
      return;
    }

    const uint32_t chunk_count = packet->chunk_count == 0U ? 1U : packet->chunk_count;
    if (packet->chunk_index >= chunk_count || chunk_count > 100000U) {
      RCLCPP_WARN(get_logger(), "Rejected invalid chunk metadata from %s",
        packet->source_id.c_str());
      return;
    }

    if (packet->full_refresh) {
      const bool continuing_refresh =
        source.sync_assembly.version == packet->version &&
        source.sync_assembly.count == chunk_count;
      if (!continuing_refresh &&
        (packet->version < source.last_full_refresh_version ||
        packet->version < source.last_version))
      {
        return;
      }
    } else {
      if (packet->version <= source.last_version) {
        return;
      }
      if (source.last_version > 0U && packet->base_version > source.last_version) {
        request_sync(*packet, source.last_version, "real-time version gap");
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Version gap for %s: have %lu, received base %lu/version %lu",
          packet->source_id.c_str(), source.last_version, packet->base_version, packet->version);
      }
    }

    surf_multirobot_msgs::msg::VoxelDelta delta;
    const auto codec_start = std::chrono::steady_clock::now();
    const surf::comms::CodecResult result =
      surf::comms::decode_delta(*packet, delta, maximum_uncompressed_bytes_);
    metrics.codec_reconstruction_ms = static_cast<float>(
      std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - codec_start).count());
    if (!result.ok) {
      metrics.accepted = false;
      metrics.rejection_reason = result.error;
      metrics.decode_latency_ms = static_cast<float>(std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - receive_start).count());
      metrics.receiver_processing_ms = metrics.decode_latency_ms;
      metrics.end_to_end_latency_ms =
        metrics.sensor_to_receiver_ms + metrics.decode_latency_ms;
      metrics_publisher_->publish(metrics);
      RCLCPP_WARN(get_logger(), "Rejected voxel packet from %s: %s",
        packet->source_id.c_str(), result.error.c_str());
      return;
    }

    if (chunk_count > 1U) {
      auto & assembly =
        packet->traffic_class == surf_multirobot_msgs::msg::CompressedVoxelDelta::TRAFFIC_SYNC ?
        source.sync_assembly : source.realtime_assembly;
      if (assembly.version != packet->version || assembly.count != chunk_count) {
        assembly = SourceState::Assembly{};
        assembly.version = packet->version;
        assembly.count = chunk_count;
        assembly.chunks.resize(chunk_count);
        assembly.received.assign(chunk_count, false);
      }
      if (!assembly.received[packet->chunk_index]) {
        assembly.chunks[packet->chunk_index] = std::move(delta);
        assembly.received[packet->chunk_index] = true;
      }
      if (!std::all_of(assembly.received.begin(), assembly.received.end(),
        [](bool received) {return received;}))
      {
        metrics.accepted = true;
        metrics.rejection_reason = "awaiting remaining chunks";
        metrics.decode_latency_ms = static_cast<float>(std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - receive_start).count());
        metrics_publisher_->publish(metrics);
        return;
      }
      delta = std::move(assembly.chunks.front());
      for (uint32_t index = 1U; index < chunk_count; ++index) {
        auto & part = assembly.chunks[index];
        delta.x.insert(delta.x.end(), part.x.begin(), part.x.end());
        delta.y.insert(delta.y.end(), part.y.begin(), part.y.end());
        delta.z.insert(delta.z.end(), part.z.begin(), part.z.end());
        delta.state.insert(delta.state.end(), part.state.begin(), part.state.end());
        delta.observation_time_ns.insert(delta.observation_time_ns.end(),
          part.observation_time_ns.begin(), part.observation_time_ns.end());
      }
      delta.chunk_index = 0U;
      delta.chunk_count = 1U;
      assembly = SourceState::Assembly{};
    }
    if (!packet->full_refresh && source.sync_assembly.count > 0U &&
      delta.version > source.sync_assembly.version)
    {
      // The full refresh will reset the downstream map to its snapshot
      // version. Retain newer completed deltas so they can be replayed after
      // that reset without interrupting their normal real-time publication.
      source.deferred_realtime[delta.version] = delta;
    }
    publisher_->publish(delta);
    metrics.accepted = true;
    metrics.decode_latency_ms = static_cast<float>(std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - receive_start).count());
    metrics.receiver_processing_ms = metrics.decode_latency_ms;
    metrics.end_to_end_latency_ms =
      metrics.sensor_to_receiver_ms + metrics.decode_latency_ms;
    metrics_publisher_->publish(metrics);
    source.last_version = std::max(source.last_version, packet->version);
    if (!packet->full_refresh &&
      packet->traffic_class == surf_multirobot_msgs::msg::CompressedVoxelDelta::TRAFFIC_REALTIME)
    {
      // ACK only after every chunk was decoded and the logical update was published.
      // This is measurement-only best-effort traffic; it never controls delivery.
      surf_multirobot_msgs::msg::RealtimeAck ack;
      ack.map_epoch = packet->map_epoch;
      ack.version = packet->version;
      realtime_ack_publisher_->publish(ack);
    }
    if (packet->full_refresh) {
      source.last_full_refresh_version = packet->version;
      source.awaiting_full_refresh = false;
      for (auto & [version, deferred] : source.deferred_realtime) {
        if (version > packet->version) {
          publisher_->publish(deferred);
        }
      }
      source.deferred_realtime.clear();
      surf_multirobot_msgs::msg::SyncAck ack;
      ack.header.stamp = now();
      ack.header.frame_id = packet->header.frame_id;
      ack.source_id = packet->source_id;
      ack.map_epoch = packet->map_epoch;
      ack.version = packet->version;
      sync_ack_publisher_->publish(ack);
    }
  }

  std::string robot_name_;
  std::string realtime_topic_;
  std::string realtime_ack_topic_;
  std::string sync_topic_;
  std::string sync_ack_topic_;
  std::string sync_request_topic_;
  std::string output_topic_;
  std::string metrics_topic_;
  std::string peer_odometry_topic_;
  double peer_box_size_x_{0.70};
  double peer_box_size_y_{0.70};
  double peer_box_size_z_{0.40};
  double peer_box_center_z_{0.0};
  double peer_box_padding_{0.20};
  double peer_box_max_age_ms_{150.0};
  std::size_t maximum_uncompressed_bytes_{64U * 1024U * 1024U};
  std::unordered_map<std::string, SourceState> sources_;
  std::chrono::steady_clock::time_point last_sync_request_time_{};

  rclcpp::Publisher<surf_multirobot_msgs::msg::VoxelDelta>::SharedPtr publisher_;
  rclcpp::Publisher<surf_multirobot_msgs::msg::DeliveryMetrics>::SharedPtr metrics_publisher_;
  rclcpp::Publisher<surf_multirobot_msgs::msg::RealtimeAck>::SharedPtr
    realtime_ack_publisher_;
  rclcpp::Publisher<surf_multirobot_msgs::msg::SyncAck>::SharedPtr sync_ack_publisher_;
  rclcpp::Publisher<surf_multirobot_msgs::msg::SyncRequest>::SharedPtr
    sync_request_publisher_;
  rclcpp::Subscription<surf_multirobot_msgs::msg::CompressedVoxelDelta>::SharedPtr
    realtime_subscription_;
  rclcpp::Subscription<surf_multirobot_msgs::msg::CompressedVoxelDelta>::SharedPtr
    sync_subscription_;
};

}  // namespace surf_humanoid

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<surf_humanoid::DroneDataReceiver>());
  rclcpp::shutdown();
  return 0;
}
