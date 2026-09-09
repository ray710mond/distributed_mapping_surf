#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "surf_multirobot_msgs/msg/compressed_voxel_delta.hpp"
#include "surf_multirobot_msgs/msg/link_metrics.hpp"
#include "surf_multirobot_msgs/msg/pipeline_metrics.hpp"
#include "surf_multirobot_msgs/msg/realtime_ack.hpp"
#include "surf_multirobot_msgs/msg/realtime_ack_metrics.hpp"
#include "surf_multirobot_msgs/msg/sync_ack.hpp"
#include "surf_multirobot_msgs/msg/sync_request.hpp"
#include "surf_multirobot_msgs/msg/sync_status.hpp"
#include "surf_multirobot_msgs/msg/voxel_delta.hpp"
#include "surf_drone/adaptive_mode.hpp"
#include "surf_drone/communication_state.hpp"
#include "surf_multirobot_comms/qos_profiles.hpp"
#include "surf_multirobot_comms/voxel_codec.hpp"

namespace surf_drone
{
namespace
{

struct CellState
{
  uint32_t consecutive_hits{0};
  uint32_t consecutive_misses{0};
  uint64_t last_seen_version{0};
  uint64_t last_sent_version{0};
  bool static_known{false};
  bool last_sent_static{false};
  uint64_t last_observation_time_ns{0U};
};


void append_record(
  surf_multirobot_msgs::msg::VoxelDelta & delta,
  const Coord & coord, uint8_t state, uint64_t observation_time_ns)
{
  delta.x.push_back(coord.x);
  delta.y.push_back(coord.y);
  delta.z.push_back(coord.z);
  delta.state.push_back(state);
  delta.observation_time_ns.push_back(observation_time_ns);
}

uint64_t stamp_to_nanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<uint64_t>(std::max(stamp.sec, 0)) * 1000000000ULL +
    static_cast<uint64_t>(stamp.nanosec);
}

double steady_seconds()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

}  // namespace

class DroneScanSender : public rclcpp::Node
{
public:
  DroneScanSender()
  : Node("drone_scan_sender"),
    adaptive_(load_adaptive_config())
  {
    robot_name_ = declare_parameter<std::string>("robot_name", "drone");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    input_topic_ = declare_parameter<std::string>("input_topic", "/" + robot_name_ + "/points");
    static_map_topic_ = declare_parameter<std::string>(
      "static_map_topic", "/" + robot_name_ + "/bonxai/static_occupied_voxels");
    realtime_topic_ = declare_parameter<std::string>(
      "realtime_topic", "/" + robot_name_ + "/transport/realtime_tx");
    realtime_ack_topic_ = declare_parameter<std::string>(
      "realtime_ack_topic", "/" + robot_name_ + "/transport/realtime_ack");
    realtime_ack_metrics_topic_ = declare_parameter<std::string>(
      "realtime_ack_metrics_topic", "/" + robot_name_ + "/comm/realtime_ack_metrics");
    sync_topic_ = declare_parameter<std::string>(
      "sync_topic", "/" + robot_name_ + "/transport/sync_tx");
    sync_ack_topic_ = declare_parameter<std::string>(
      "sync_ack_topic", "/" + robot_name_ + "/transport/sync_ack");
    sync_request_topic_ = declare_parameter<std::string>(
      "sync_request_topic", "/" + robot_name_ + "/transport/sync_request");
    sync_status_topic_ = declare_parameter<std::string>(
      "sync_status_topic", "/" + robot_name_ + "/comm/sync_status");
    link_metrics_topic_ = declare_parameter<std::string>(
      "link_metrics_topic", "/surf/comm/link_metrics");
    metrics_topic_ = declare_parameter<std::string>(
      "metrics_topic", "/" + robot_name_ + "/comm/pipeline_metrics");
    transform_source_ = declare_parameter<std::string>("pose_source.type", "tf");
    odometry_topic_ = declare_parameter<std::string>(
      "pose_source.odometry_topic", "/" + robot_name_ + "/odom");
    odometry_parent_frame_ = declare_parameter<std::string>(
      "pose_source.odometry_parent_frame", map_frame_);
    base_frame_ = declare_parameter<std::string>(
      "pose_source.base_frame", robot_name_ + "/body");
    pose_max_age_ms_ = std::max(
      0.0, declare_parameter<double>("pose_source.max_age_ms", 50.0));
    transform_timeout_ms_ = std::max(
      0.0, declare_parameter<double>("pose_source.tf_timeout_ms", 500.0));

    resolution_ = declare_parameter<double>("resolution", 0.05);
    min_range_ = declare_parameter<double>("filters.min_range", 0.75);
    max_range_ = declare_parameter<double>("filters.max_range", 50.0);
    min_z_ = declare_parameter<double>("filters.min_z", 0.25);
    max_z_ = declare_parameter<double>("filters.max_z", 20.0);
    self_radius_ = declare_parameter<double>("filters.self_radius", 0.70);
    humanoid_mask_enabled_ = declare_parameter<bool>("filters.humanoid_mask.enabled", true);
    humanoid_odometry_topic_ = declare_parameter<std::string>(
      "filters.humanoid_mask.odometry_topic", "/humanoid/odom");
    humanoid_mask_size_x_ = std::max(
      0.0, declare_parameter<double>("filters.humanoid_mask.model_size_x", 0.70));
    humanoid_mask_size_y_ = std::max(
      0.0, declare_parameter<double>("filters.humanoid_mask.model_size_y", 0.66));
    humanoid_mask_size_z_ = std::max(
      0.0, declare_parameter<double>("filters.humanoid_mask.model_size_z", 0.825));
    humanoid_mask_center_z_ = declare_parameter<double>(
      "filters.humanoid_mask.model_center_z", 0.4125);
    humanoid_mask_padding_ = std::max(
      0.0, declare_parameter<double>("filters.humanoid_mask.padding", 0.20));
    humanoid_mask_max_age_ms_ = std::max(
      0.0, declare_parameter<double>("filters.humanoid_mask.max_age_ms", 150.0));
    static_min_hits_ = static_cast<int>(std::max<int64_t>(
      1, declare_parameter<int64_t>("filters.static_min_hits", 20)));
    clear_min_misses_ = static_cast<int>(std::max<int64_t>(
      1, declare_parameter<int64_t>("filters.clear_min_misses", 3)));
    delta_refresh_scans_ = static_cast<int>(std::max<int64_t>(
      1, declare_parameter<int64_t>("delta_refresh_scans", 50)));
    tombstone_retention_scans_ = static_cast<int>(std::max<int64_t>(
      1, declare_parameter<int64_t>("tombstone_retention_scans", 500)));
    dynamic_retention_scans_ = static_cast<int>(std::max<int64_t>(
      1, declare_parameter<int64_t>("dynamic_retention_scans", 20)));
    maximum_ray_voxels_ = static_cast<int>(std::max<int64_t>(
      1, declare_parameter<int64_t>("maximum_ray_voxels", 1200)));
    maximum_clear_rays_ = static_cast<int>(std::max<int64_t>(
      1, declare_parameter<int64_t>("maximum_clear_rays", 256)));
    sync_interval_seconds_ = declare_parameter<double>("sync_interval_seconds", 10.0);
    compression_level_ = static_cast<int>(declare_parameter<int64_t>("compression_level", 1));
    maximum_packet_bytes_ = static_cast<std::size_t>(std::max<int64_t>(
      256, declare_parameter<int64_t>("transport.maximum_packet_bytes", 1200)));
    maximum_update_bytes_ = static_cast<std::size_t>(std::max<int64_t>(
      static_cast<int64_t>(maximum_packet_bytes_),
      declare_parameter<int64_t>("transport.maximum_update_bytes", 12000)));
    maximum_sync_packet_bytes_ = static_cast<std::size_t>(std::max<int64_t>(
      256, declare_parameter<int64_t>("transport.maximum_sync_packet_bytes", 65536)));
    maximum_sync_update_bytes_ = static_cast<std::size_t>(std::max<int64_t>(
      static_cast<int64_t>(maximum_sync_packet_bytes_),
      declare_parameter<int64_t>("transport.maximum_sync_update_bytes", 16 * 1024 * 1024)));
    chunk_publish_interval_ms_ = std::max<int64_t>(
      1, declare_parameter<int64_t>("transport.chunk_publish_interval_ms", 15));
    sync_ack_timeout_seconds_ = std::max(
      0.1, declare_parameter<double>("transport.sync_ack_timeout_seconds", 2.0));
    sync_ack_margin_seconds_ = std::max(
      0.0, declare_parameter<double>("transport.sync_ack_margin_seconds", 2.0));
    realtime_ack_timeout_seconds_ = std::max(
      0.1, declare_parameter<double>("transport.realtime_ack_timeout_seconds", 2.0));
    sync_max_interval_seconds_ = std::max(
      sync_interval_seconds_,
      declare_parameter<double>("transport.sync_max_interval_seconds", 60.0));
    sync_target_duty_cycle_ = std::clamp(
      declare_parameter<double>("transport.sync_target_duty_cycle", 0.10), 0.01, 1.0);
    next_sync_interval_seconds_.store(sync_interval_seconds_);
    adaptive_enabled_ = declare_parameter<bool>("adaptive.enabled", true);
    manual_mode_ = static_cast<int>(declare_parameter<int64_t>("adaptive.manual_mode", -1));

    std::random_device random_device;
    map_epoch_ = (static_cast<uint64_t>(random_device()) << 32U) ^ random_device();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    if (transform_source_ == "odometry") {
      odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        odometry_topic_, rclcpp::SensorDataQoS().keep_last(50),
        [this](const nav_msgs::msg::Odometry::SharedPtr message) {
          std::lock_guard<std::mutex> lock(odometry_mutex_);
          odometry_history_.push_back(message);
          while (odometry_history_.size() > 200U) {
            odometry_history_.pop_front();
          }
        });
    } else if (transform_source_ != "tf") {
      throw std::invalid_argument("pose_source.type must be 'tf' or 'odometry'");
    }

    realtime_publisher_ = create_publisher<surf_multirobot_msgs::msg::CompressedVoxelDelta>(
      realtime_topic_, surf::comms::realtime_qos());
    sync_publisher_ = create_publisher<surf_multirobot_msgs::msg::CompressedVoxelDelta>(
      sync_topic_, surf::comms::sync_qos());
    metrics_publisher_ = create_publisher<surf_multirobot_msgs::msg::PipelineMetrics>(
      metrics_topic_, rclcpp::QoS(10));
    realtime_ack_metrics_publisher_ =
      create_publisher<surf_multirobot_msgs::msg::RealtimeAckMetrics>(
      realtime_ack_metrics_topic_, rclcpp::QoS(50));
    sync_status_publisher_ = create_publisher<surf_multirobot_msgs::msg::SyncStatus>(
      sync_status_topic_, rclcpp::QoS(10).reliable().transient_local());
    realtime_publish_timer_ = create_wall_timer(
      std::chrono::milliseconds(chunk_publish_interval_ms_),
      [this]() {publish_next_queued_packet(false);});
    sync_publish_timer_ = create_wall_timer(
      std::chrono::milliseconds(chunk_publish_interval_ms_),
      [this]() {publish_next_queued_packet(true);});
    sync_retry_timer_ = create_wall_timer(
      std::chrono::milliseconds(250), [this]() {
        retry_unacknowledged_sync();
        expire_realtime_acks();
      });
    realtime_ack_subscription_ = create_subscription<surf_multirobot_msgs::msg::RealtimeAck>(
      realtime_ack_topic_, rclcpp::QoS(10).best_effort().durability_volatile(),
      std::bind(&DroneScanSender::realtime_ack_callback, this, std::placeholders::_1));
    sync_ack_subscription_ = create_subscription<surf_multirobot_msgs::msg::SyncAck>(
      sync_ack_topic_, rclcpp::QoS(10).reliable().transient_local(),
      std::bind(&DroneScanSender::sync_ack_callback, this, std::placeholders::_1));
    sync_request_subscription_ = create_subscription<surf_multirobot_msgs::msg::SyncRequest>(
      sync_request_topic_, rclcpp::QoS(10).reliable().transient_local(),
      std::bind(&DroneScanSender::sync_request_callback, this, std::placeholders::_1));

    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS().keep_last(2),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr message) {
        const auto received = std::chrono::steady_clock::now();
        {
          std::lock_guard<std::mutex> lock(queue_mutex_);
          if (last_input_time_ != std::chrono::steady_clock::time_point{}) {
            const double interval = std::chrono::duration<double>(
              received - last_input_time_).count();
            if (interval > 0.0) {
              const double instantaneous_rate = 1.0 / interval;
              input_rate_hz_ = input_rate_hz_ == 0.0 ? instantaneous_rate :
                0.2 * instantaneous_rate + 0.8 * input_rate_hz_;
            }
          }
          last_input_time_ = received;
          if (latest_cloud_) {
            ++stale_input_drops_;
          }
          latest_cloud_ = std::move(message);
          latest_cloud_received_ = received;
        }
        queue_condition_.notify_one();
      });
    static_map_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      static_map_topic_, rclcpp::QoS(1).reliable().durability_volatile(),
      std::bind(&DroneScanSender::static_map_callback, this, std::placeholders::_1));
    link_metrics_subscription_ = create_subscription<surf_multirobot_msgs::msg::LinkMetrics>(
      link_metrics_topic_, rclcpp::QoS(10),
      std::bind(&DroneScanSender::link_metrics_callback, this, std::placeholders::_1));
    if (humanoid_mask_enabled_) {
      humanoid_odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        humanoid_odometry_topic_, rclcpp::SensorDataQoS().keep_last(50),
        [this](const nav_msgs::msg::Odometry::SharedPtr message) {
          std::lock_guard<std::mutex> lock(humanoid_odometry_mutex_);
          humanoid_odometry_history_.push_back(message);
          while (humanoid_odometry_history_.size() > 200U) {
            humanoid_odometry_history_.pop_front();
          }
        });
    }

    last_sync_time_ = steady_seconds() - std::max(0.0, sync_interval_seconds_);
    worker_ = std::thread(&DroneScanSender::worker_loop, this);
    RCLCPP_INFO(get_logger(),
      "%s communication sender: %s -> [%s, %s], resolution %.2fm",
      robot_name_.c_str(), input_topic_.c_str(), realtime_topic_.c_str(), sync_topic_.c_str(),
      resolution_);
    RCLCPP_INFO(get_logger(), "Pose source: %s%s", transform_source_.c_str(),
      transform_source_ == "odometry" ? (" on " + odometry_topic_).c_str() : "");
    if (humanoid_mask_enabled_) {
      RCLCPP_INFO(get_logger(),
        "Humanoid communication mask: %.2f x %.2f x %.2fm model + %.2fm padding, state from %s",
        humanoid_mask_size_x_, humanoid_mask_size_y_, humanoid_mask_size_z_,
        humanoid_mask_padding_, humanoid_odometry_topic_.c_str());
    }
  }

  ~DroneScanSender() override
  {
    stop_.store(true);
    queue_condition_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  void enqueue_packets(
    uint8_t traffic_class,
    std::vector<surf_multirobot_msgs::msg::CompressedVoxelDelta> packets,
    uint32_t wire_bytes, float encoding_ms)
  {
    std::lock_guard<std::mutex> lock(transport_queue_mutex_);
    if (traffic_class == surf_multirobot_msgs::msg::CompressedVoxelDelta::TRAFFIC_SYNC) {
      pending_sync_packets_ = packets;
      pending_sync_epoch_ = packets.empty() ? 0U : packets.front().map_epoch;
      pending_sync_version_ = packets.empty() ? 0U : packets.front().version;
      sync_attempt_time_ = std::chrono::steady_clock::now();
      sync_first_attempt_time_ = sync_attempt_time_;
      pending_sync_wire_bytes_ = wire_bytes;
      pending_sync_encoding_ms_ = encoding_ms;
      pending_sync_estimated_transfer_ms_ = static_cast<float>(
        packets.size() * chunk_publish_interval_ms_);
      pending_sync_transfer_ms_ = 0.0F;
      pending_sync_ack_timeout_seconds_ = std::max(
        sync_ack_timeout_seconds_,
        pending_sync_estimated_transfer_ms_ / 1000.0 + sync_ack_margin_seconds_);
      next_sync_interval_seconds_.store(std::clamp(
        (encoding_ms / 1000.0 + pending_sync_estimated_transfer_ms_ / 1000.0) /
        sync_target_duty_cycle_, sync_interval_seconds_, sync_max_interval_seconds_));
      pending_sync_retry_count_ = 0U;
      publish_sync_status(surf_multirobot_msgs::msg::SyncStatus::STATE_QUEUED, "");
    }
    auto & queue =
      traffic_class == surf_multirobot_msgs::msg::CompressedVoxelDelta::TRAFFIC_SYNC ?
      sync_packet_queue_ : realtime_packet_queue_;
    for (auto & packet : packets) {
      queue.push_back(std::move(packet));
    }
  }

  bool has_pending_sync()
  {
    std::lock_guard<std::mutex> lock(transport_queue_mutex_);
    return !pending_sync_packets_.empty();
  }

  void retry_unacknowledged_sync()
  {
    std::lock_guard<std::mutex> lock(transport_queue_mutex_);
    if (pending_sync_packets_.empty() || !sync_packet_queue_.empty()) {
      return;
    }
    const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - sync_attempt_time_).count();
    if (elapsed < pending_sync_ack_timeout_seconds_) {
      return;
    }
    for (const auto & packet : pending_sync_packets_) {
      sync_packet_queue_.push_back(packet);
    }
    sync_attempt_time_ = std::chrono::steady_clock::now();
    ++pending_sync_retry_count_;
    publish_sync_status(surf_multirobot_msgs::msg::SyncStatus::STATE_RETRY,
      "acknowledgment timeout");
    RCLCPP_WARN(get_logger(),
      "Retrying unacknowledged full refresh epoch %lu version %lu (%zu chunks)",
      pending_sync_epoch_, pending_sync_version_, pending_sync_packets_.size());
  }

  void sync_ack_callback(const surf_multirobot_msgs::msg::SyncAck::SharedPtr ack)
  {
    std::lock_guard<std::mutex> lock(transport_queue_mutex_);
    if (ack->source_id != robot_name_ || ack->map_epoch != pending_sync_epoch_ ||
      ack->version != pending_sync_version_)
    {
      return;
    }
    RCLCPP_INFO(get_logger(), "Full refresh acknowledged: epoch %lu version %lu",
      ack->map_epoch, ack->version);
    const float acknowledgment_ms = static_cast<float>(std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - sync_first_attempt_time_).count());
    if (pending_sync_transfer_ms_ <= 0.0F) {
      pending_sync_transfer_ms_ = static_cast<float>(
        std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - sync_attempt_time_).count());
    }
    publish_sync_status(
      surf_multirobot_msgs::msg::SyncStatus::STATE_ACKNOWLEDGED, "", acknowledgment_ms);
    last_acknowledged_sync_version_ = pending_sync_version_;
    pending_sync_packets_.clear();
    sync_packet_queue_.clear();
    pending_sync_epoch_ = 0U;
    pending_sync_version_ = 0U;
  }

  void publish_realtime_ack_metric(
    uint64_t version, bool acknowledged, float update_rtt_ms, float final_chunk_rtt_ms)
  {
    surf_multirobot_msgs::msg::RealtimeAckMetrics metrics;
    metrics.header.stamp = now();
    metrics.map_epoch = map_epoch_;
    metrics.version = version;
    metrics.acknowledged = acknowledged;
    metrics.update_completion_rtt_ms = update_rtt_ms;
    metrics.final_chunk_rtt_ms = final_chunk_rtt_ms;
    metrics.timeout_ms = static_cast<float>(realtime_ack_timeout_seconds_ * 1000.0);
    realtime_ack_metrics_publisher_->publish(metrics);
  }

  void realtime_ack_callback(
    const surf_multirobot_msgs::msg::RealtimeAck::SharedPtr ack)
  {
    if (ack->map_epoch != map_epoch_) {
      return;
    }
    const auto pending = pending_realtime_acks_.find(ack->version);
    if (pending == pending_realtime_acks_.end()) {
      return;
    }
    const auto current = std::chrono::steady_clock::now();
    const float update_rtt_ms = static_cast<float>(std::chrono::duration<double, std::milli>(
      current - pending->second.first_chunk).count());
    const float final_chunk_rtt_ms = static_cast<float>(
      std::chrono::duration<double, std::milli>(current - pending->second.final_chunk).count());
    pending_realtime_acks_.erase(pending);
    publish_realtime_ack_metric(ack->version, true, update_rtt_ms, final_chunk_rtt_ms);
  }

  void expire_realtime_acks()
  {
    const auto current = std::chrono::steady_clock::now();
    for (auto pending = pending_realtime_acks_.begin();
      pending != pending_realtime_acks_.end();)
    {
      if (pending->second.final_chunk == std::chrono::steady_clock::time_point{}) {
        ++pending;
        continue;
      }
      const double age = std::chrono::duration<double>(
        current - pending->second.final_chunk).count();
      if (age < realtime_ack_timeout_seconds_) {
        ++pending;
        continue;
      }
      const uint64_t version = pending->first;
      pending = pending_realtime_acks_.erase(pending);
      publish_realtime_ack_metric(version, false, 0.0F, 0.0F);
    }
  }

  void sync_request_callback(const surf_multirobot_msgs::msg::SyncRequest::SharedPtr request)
  {
    if (request->source_id != robot_name_ ||
      (request->map_epoch != 0U && request->map_epoch != map_epoch_))
    {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(transport_queue_mutex_);
      // A retained startup request can arrive while the refresh that satisfies
      // it is already in flight (or immediately after its ACK). Do not turn
      // that stale request into a second full-map encoding.
      if ((!pending_sync_packets_.empty() && pending_sync_version_ > request->last_version) ||
        last_acknowledged_sync_version_ > request->last_version)
      {
        return;
      }
    }
    immediate_sync_requested_.store(true);
    surf_multirobot_msgs::msg::SyncStatus status;
    status.header.stamp = now();
    status.source_id = robot_name_;
    status.map_epoch = map_epoch_;
    status.version = request->last_version;
    status.state = surf_multirobot_msgs::msg::SyncStatus::STATE_REQUESTED;
    status.reason = request->reason;
    status.next_refresh_interval_s = static_cast<float>(next_sync_interval_seconds_.load());
    sync_status_publisher_->publish(status);
  }

  void publish_sync_status(uint8_t state, const std::string & reason, float ack_ms = 0.0F)
  {
    surf_multirobot_msgs::msg::SyncStatus status;
    status.header.stamp = now();
    status.source_id = robot_name_;
    status.map_epoch = pending_sync_epoch_;
    status.version = pending_sync_version_;
    status.state = state;
    status.wire_bytes = pending_sync_wire_bytes_;
    status.packet_count = static_cast<uint32_t>(pending_sync_packets_.size());
    status.retry_count = pending_sync_retry_count_;
    status.encoding_ms = pending_sync_encoding_ms_;
    status.estimated_transfer_ms = pending_sync_estimated_transfer_ms_;
    status.transfer_ms = pending_sync_transfer_ms_;
    status.acknowledgment_ms = ack_ms;
    status.acknowledgment_timeout_ms = static_cast<float>(
      pending_sync_ack_timeout_seconds_ * 1000.0);
    status.next_refresh_interval_s = static_cast<float>(next_sync_interval_seconds_.load());
    status.reason = reason;
    sync_status_publisher_->publish(status);
  }

  void publish_next_queued_packet(bool sync)
  {
    surf_multirobot_msgs::msg::CompressedVoxelDelta packet;
    {
      std::lock_guard<std::mutex> lock(transport_queue_mutex_);
      auto & queue = sync ? sync_packet_queue_ : realtime_packet_queue_;
      if (queue.empty()) {
        return;
      }
      packet = std::move(queue.front());
      queue.pop_front();
    }
    packet.transmit_stamp = now();
    if (sync) {
      sync_publisher_->publish(packet);
      std::lock_guard<std::mutex> lock(transport_queue_mutex_);
      if (sync_packet_queue_.empty() && !pending_sync_packets_.empty()) {
        pending_sync_transfer_ms_ = static_cast<float>(
          std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - sync_attempt_time_).count());
        publish_sync_status(
          surf_multirobot_msgs::msg::SyncStatus::STATE_TRANSMITTED, "");
      }
    } else {
      if (packet.chunk_index == 0U) {
        pending_realtime_acks_[packet.version].first_chunk = std::chrono::steady_clock::now();
      }
      if (packet.chunk_index + 1U == std::max(packet.chunk_count, 1U)) {
        // Start RTT at transmission of the final chunk, because the receiver
        // cannot acknowledge reconstruction before that packet arrives.
        pending_realtime_acks_[packet.version].final_chunk = std::chrono::steady_clock::now();
      }
      realtime_publisher_->publish(packet);
    }
  }

  AdaptiveModeController::Config load_adaptive_config()
  {
    AdaptiveModeController::Config config;
    config.full_min_mbps = declare_parameter<double>("adaptive.full_min_mbps", 12.0);
    config.delta_min_mbps = declare_parameter<double>("adaptive.delta_min_mbps", 3.0);
    config.dynamic_min_mbps = declare_parameter<double>("adaptive.dynamic_min_mbps", 0.75);
    config.up_hysteresis_mbps = declare_parameter<double>("adaptive.up_hysteresis_mbps", 1.0);
    config.down_hold_seconds = declare_parameter<double>("adaptive.down_hold_seconds", 2.0);
    config.up_hold_seconds = declare_parameter<double>("adaptive.up_hold_seconds", 5.0);
    config.minimum_dwell_seconds = declare_parameter<double>("adaptive.minimum_dwell_seconds", 5.0);
    config.ewma_alpha = declare_parameter<double>("adaptive.ewma_alpha", 0.2);
    config.congested_queue_depth = static_cast<uint32_t>(std::max<int64_t>(
      1, declare_parameter<int64_t>("adaptive.congested_queue_depth", 2)));
    return config;
  }

  void worker_loop()
  {
    while (!stop_.load()) {
      sensor_msgs::msg::PointCloud2::SharedPtr cloud;
      std::chrono::steady_clock::time_point received;
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_condition_.wait(lock, [this]() {return stop_.load() || latest_cloud_ != nullptr;});
        if (stop_.load()) {
          return;
        }
        cloud = std::move(latest_cloud_);
        received = latest_cloud_received_;
      }
      const float queue_wait_ms = static_cast<float>(
        std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - received).count());
      process_cloud(*cloud, queue_wait_ms);
    }
  }

  void static_map_callback(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    {
      std::lock_guard<std::mutex> lock(static_map_mutex_);
      if (static_prior_initialized_) {
        return;
      }
    }
    std::unordered_set<Coord, CoordHash> replacement;
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*message, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*message, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*message, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        if (std::isfinite(*x) && std::isfinite(*y) && std::isfinite(*z)) {
          replacement.insert(quantize(*x, *y, *z, resolution_));
        }
      }
    } catch (const std::runtime_error & exception) {
      RCLCPP_WARN(get_logger(), "Ignoring invalid static voxel cloud: %s", exception.what());
      return;
    }
    std::lock_guard<std::mutex> lock(static_map_mutex_);
    if (!static_prior_initialized_) {
      static_map_ = std::make_shared<const CoordSet>(std::move(replacement));
      static_prior_initialized_ = true;
      RCLCPP_INFO(get_logger(), "Captured %zu voxels in the static communication prior",
        static_map_->size());
    }
  }

  void link_metrics_callback(const surf_multirobot_msgs::msg::LinkMetrics::SharedPtr metrics)
  {
    std::lock_guard<std::mutex> lock(link_mutex_);
    available_bandwidth_mbps_ = metrics->configured_bandwidth_mbps;
    link_queue_depth_ = metrics->queue_depth;
  }

  uint8_t current_mode()
  {
    if (manual_mode_ >= 0 && manual_mode_ <= 3) {
      return static_cast<uint8_t>(manual_mode_);
    }
    if (!adaptive_enabled_) {
      return surf_multirobot_msgs::msg::VoxelDelta::MODE_VOXEL_DELTAS;
    }
    double bandwidth;
    uint32_t queue_depth;
    {
      std::lock_guard<std::mutex> lock(link_mutex_);
      bandwidth = available_bandwidth_mbps_;
      queue_depth = link_queue_depth_;
    }
    return adaptive_.update(steady_seconds(), bandwidth, queue_depth);
  }

  void trace_ray_for_clears(
    const Coord & origin, const Coord & endpoint,
    const std::unordered_set<Coord, CoordHash> & current,
    std::unordered_set<Coord, CoordHash> & traversed_known)
  {
    const int64_t dx = static_cast<int64_t>(endpoint.x) - origin.x;
    const int64_t dy = static_cast<int64_t>(endpoint.y) - origin.y;
    const int64_t dz = static_cast<int64_t>(endpoint.z) - origin.z;
    const int steps = static_cast<int>(std::min<int64_t>(
      maximum_ray_voxels_, std::max({std::llabs(dx), std::llabs(dy), std::llabs(dz)})));
    if (steps <= 1) {
      return;
    }
    Coord previous = origin;
    for (int step = 1; step < steps; ++step) {
      const double ratio = static_cast<double>(step) / static_cast<double>(steps);
      Coord coord{
        static_cast<int32_t>(std::llround(origin.x + ratio * dx)),
        static_cast<int32_t>(std::llround(origin.y + ratio * dy)),
        static_cast<int32_t>(std::llround(origin.z + ratio * dz))};
      if (coord == previous) {
        continue;
      }
      previous = coord;
      if (current.find(coord) == current.end() && cells_.find(coord) != cells_.end()) {
        traversed_known.insert(coord);
      }
    }
  }


  std::unique_ptr<HumanoidMask> humanoid_mask_at(const builtin_interfaces::msg::Time & stamp)
  {
    if (!humanoid_mask_enabled_) {
      return nullptr;
    }
    nav_msgs::msg::Odometry::SharedPtr nearest;
    int64_t nearest_difference_ns = std::numeric_limits<int64_t>::max();
    const rclcpp::Time target(stamp);
    {
      std::lock_guard<std::mutex> lock(humanoid_odometry_mutex_);
      for (const auto & candidate : humanoid_odometry_history_) {
        const int64_t difference = std::llabs(
          (rclcpp::Time(candidate->header.stamp) - target).nanoseconds());
        if (difference < nearest_difference_ns) {
          nearest_difference_ns = difference;
          nearest = candidate;
        }
      }
    }
    if (!nearest ||
      static_cast<double>(nearest_difference_ns) / 1.0e6 > humanoid_mask_max_age_ms_)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Humanoid communication mask has no state within %.0fms of the scan; passing scan unmasked",
        humanoid_mask_max_age_ms_);
      return nullptr;
    }

    auto mask = std::make_unique<HumanoidMask>();
    tf2::fromMsg(nearest->pose.pose.orientation, mask->world_from_model);
    if (mask->world_from_model.length2() < 1.0e-12) {
      mask->world_from_model = tf2::Quaternion::getIdentity();
    } else {
      mask->world_from_model.normalize();
    }
    const tf2::Vector3 model_position(
      nearest->pose.pose.position.x,
      nearest->pose.pose.position.y,
      nearest->pose.pose.position.z);
    mask->center = model_position + tf2::quatRotate(
      mask->world_from_model, tf2::Vector3(0.0, 0.0, humanoid_mask_center_z_));
    mask->half_x = 0.5 * humanoid_mask_size_x_ + humanoid_mask_padding_;
    mask->half_y = 0.5 * humanoid_mask_size_y_ + humanoid_mask_padding_;
    mask->half_z = 0.5 * humanoid_mask_size_z_ + humanoid_mask_padding_;
    return mask;
  }

  void process_cloud(const sensor_msgs::msg::PointCloud2 & cloud, float queue_wait_ms)
  {
    const auto processing_start = std::chrono::steady_clock::now();
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = resolve_transform(cloud);
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Communication filter cannot transform cloud: %s", exception.what());
      return;
    }
    const auto transform_complete = std::chrono::steady_clock::now();

    tf2::Quaternion rotation;
    tf2::fromMsg(transform.transform.rotation, rotation);
    const tf2::Vector3 translation(
      transform.transform.translation.x,
      transform.transform.translation.y,
      transform.transform.translation.z);
    const Coord origin = quantize(translation.x(), translation.y(), translation.z(), resolution_);
    const std::unique_ptr<HumanoidMask> humanoid_mask = humanoid_mask_at(cloud.header.stamp);

    uint32_t raw_points = cloud.width * cloud.height;
    uint32_t valid_points = 0;
    std::unordered_set<Coord, CoordHash> current;
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(cloud, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {
          continue;
        }
        const tf2::Vector3 local(*x, *y, *z);
        const double range = local.length();
        if (range < min_range_ || range > max_range_) {
          continue;
        }
        const tf2::Vector3 mapped = tf2::quatRotate(rotation, local) + translation;
        if (mapped.z() < min_z_ || mapped.z() > max_z_ ||
          (mapped - translation).length2() < self_radius_ * self_radius_)
        {
          continue;
        }
        if (humanoid_mask && humanoid_mask->contains(mapped.x(), mapped.y(), mapped.z())) {
          continue;
        }
        current.insert(quantize(mapped.x(), mapped.y(), mapped.z(), resolution_));
        ++valid_points;
      }
    } catch (const std::runtime_error & exception) {
      RCLCPP_ERROR(get_logger(), "Point cloud does not provide float x/y/z fields: %s",
        exception.what());
      return;
    }
    const auto point_preprocessing_complete = std::chrono::steady_clock::now();

    const auto raw_serialization_start = std::chrono::steady_clock::now();
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> cloud_serializer;
    rclcpp::SerializedMessage serialized_cloud;
    cloud_serializer.serialize_message(&cloud, &serialized_cloud);
    const std::size_t raw_serialized_bytes = serialized_cloud.size();
    const float raw_serialization_ms = static_cast<float>(
      std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - raw_serialization_start).count());

    ++version_;
    const uint64_t scan_time_ns = stamp_to_nanoseconds(cloud.header.stamp);
    const uint8_t mode = current_mode();
    surf_multirobot_msgs::msg::VoxelDelta realtime;
    initialize_delta(realtime, cloud, mode, false);
    realtime.sensor_origin.x = translation.x();
    realtime.sensor_origin.y = translation.y();
    realtime.sensor_origin.z = translation.z();

    std::shared_ptr<const CoordSet> static_snapshot;
    {
      std::lock_guard<std::mutex> lock(static_map_mutex_);
      static_snapshot = static_map_;
    }

    const auto occupancy_selection_start = std::chrono::steady_clock::now();
    uint32_t static_prior_voxels = 0U;

    // Communication state is deliberately scrubbed without publishing
    // tombstones. The humanoid owns this volume and fills it from its onboard
    // sensor; the drone's separate local map still receives the untouched scan.
    if (humanoid_mask) {
      const auto bounds = humanoid_mask->voxel_bounds(resolution_);
      for (const auto & coord : cells_.in_box(bounds.first, bounds.second)) {
        if (humanoid_mask->contains(coord, resolution_)) {
          cells_.erase(coord);
          dynamic_expiry_.cancel(coord);
          tombstones_.erase(coord);
        }
      }
      for (const auto & coord : tombstones_.in_box(bounds.first, bounds.second)) {
        if (humanoid_mask->contains(coord, resolution_)) {tombstones_.erase(coord);}
      }
    }

    for (const auto & coord : current) {
      tombstones_.erase(coord);
      auto & cell = cells_[coord];
      const bool represented_by_prior = static_snapshot->find(coord) != static_snapshot->end();
      if (represented_by_prior) {
        ++static_prior_voxels;
      }
      if (cell.last_seen_version + 1U == version_) {
        ++cell.consecutive_hits;
      } else {
        cell.consecutive_hits = 1U;
      }
      cell.consecutive_misses = 0U;
      cell.last_seen_version = version_;
      cell.last_observation_time_ns = scan_time_ns;
      if (represented_by_prior || cell.consecutive_hits >= static_cast<uint32_t>(static_min_hits_))
      {
        cell.static_known = true;
      }

      if (cell.static_known) {
        dynamic_expiry_.cancel(coord);
      } else {
        // Preserve the original strict > retention boundary.
        dynamic_expiry_.schedule(coord, version_ + dynamic_retention_scans_ + 1U);
      }

      const bool new_information = cell.last_sent_version == 0U ||
        cell.last_sent_static != cell.static_known;
      // Give every static coordinate a stable phase in the refresh cycle so
      // cells first observed together do not all become due on one scan.
      const uint64_t refresh_period = static_cast<uint64_t>(delta_refresh_scans_);
      const uint64_t refresh_phase = static_cast<uint64_t>(CoordHash{}(coord)) % refresh_period;
      const bool refresh_due = cell.static_known ?
        (cell.last_sent_version > 0U && version_ % refresh_period == refresh_phase) :
        (version_ - cell.last_sent_version >= refresh_period);
      bool send = mode == surf_multirobot_msgs::msg::VoxelDelta::MODE_FULL;
      if (mode == surf_multirobot_msgs::msg::VoxelDelta::MODE_VOXEL_DELTAS) {
        send = !represented_by_prior && (new_information || refresh_due);
      } else if (mode == surf_multirobot_msgs::msg::VoxelDelta::MODE_DYNAMIC_ONLY) {
        send = !represented_by_prior && !cell.static_known && (new_information || refresh_due);
      }
      if (send) {
        append_record(realtime, coord, cell.static_known ?
          surf_multirobot_msgs::msg::VoxelDelta::STATE_OCCUPIED_STATIC :
          surf_multirobot_msgs::msg::VoxelDelta::STATE_OCCUPIED_DYNAMIC,
          cell.last_observation_time_ns);
        cell.last_sent_version = version_;
        cell.last_sent_static = cell.static_known;
      }
    }
    const auto occupancy_selection_complete = std::chrono::steady_clock::now();

    std::unordered_set<Coord, CoordHash> traversed_known;
    const std::size_t maximum_clear_rays = static_cast<std::size_t>(maximum_clear_rays_);
    const std::size_t ray_stride = std::max<std::size_t>(
      1U, (current.size() + maximum_clear_rays - 1U) / maximum_clear_rays);
    const std::size_t ray_offset = static_cast<std::size_t>(version_) % ray_stride;
    std::size_t ray_index = 0U;
    std::size_t sampled_rays = 0U;
    for (const auto & endpoint : current) {
      if ((ray_index++ % ray_stride) != ray_offset) {
        continue;
      }
      trace_ray_for_clears(origin, endpoint, current, traversed_known);
      if (++sampled_rays >= maximum_clear_rays) {
        break;
      }
    }
    for (const auto & coord : traversed_known) {
      auto found = cells_.find(coord);
      if (found == cells_.end()) {
        continue;
      }
      auto & cell = found->second;
      ++cell.consecutive_misses;
      if (cell.consecutive_misses < static_cast<uint32_t>(clear_min_misses_)) {
        continue;
      }
      const uint8_t cleared_state = cell.static_known ?
        surf_multirobot_msgs::msg::VoxelDelta::STATE_DELETE :
        surf_multirobot_msgs::msg::VoxelDelta::STATE_FREE;
      tombstones_[coord] = {version_, cleared_state, scan_time_ns};
      if (mode != surf_multirobot_msgs::msg::VoxelDelta::MODE_METADATA_ONLY) {
        append_record(realtime, coord, cleared_state, scan_time_ns);
      }
      dynamic_expiry_.cancel(coord);
      cells_.erase(found);
    }

    // Dynamic cells that are no longer endpoints must not be resurrected by
    // periodic full refreshes merely because no sampled clearing ray hit them.
    for (const auto & coord : dynamic_expiry_.pop_due(version_)) {
      auto it = cells_.find(coord);
      if (it == cells_.end() || it->second.static_known) {continue;}
      tombstones_[coord] = {
        version_, surf_multirobot_msgs::msg::VoxelDelta::STATE_FREE, scan_time_ns};
      if (mode != surf_multirobot_msgs::msg::VoxelDelta::MODE_METADATA_ONLY) {
        append_record(realtime, coord,
          surf_multirobot_msgs::msg::VoxelDelta::STATE_FREE, scan_time_ns);
      }
      cells_.erase(it);
    }
    const auto clearing_complete = std::chrono::steady_clock::now();

    const float transform_lookup_ms = static_cast<float>(
      std::chrono::duration<double, std::milli>(transform_complete - processing_start).count());
    const float point_preprocessing_ms = static_cast<float>(
      std::chrono::duration<double, std::milli>(
        point_preprocessing_complete - transform_complete).count());
    const float occupancy_selection_ms = static_cast<float>(
      std::chrono::duration<double, std::milli>(
        occupancy_selection_complete - occupancy_selection_start).count());
    const float clearing_ms = static_cast<float>(
      std::chrono::duration<double, std::milli>(
        clearing_complete - occupancy_selection_complete).count());

    publish_delta(realtime,
      surf_multirobot_msgs::msg::CompressedVoxelDelta::TRAFFIC_REALTIME,
      raw_serialized_bytes, raw_points, valid_points,
      static_cast<uint32_t>(current.size()), static_prior_voxels, processing_start,
      transform_lookup_ms, point_preprocessing_ms, occupancy_selection_ms, clearing_ms,
      raw_serialization_ms, queue_wait_ms, static_cast<uint32_t>(cloud.data.size()),
      cloud.point_step);

    const double now = steady_seconds();
    const bool sync_due = sync_interval_seconds_ > 0.0 &&
      now - last_sync_time_ >= next_sync_interval_seconds_.load();
    if ((sync_due || immediate_sync_requested_.load()) && !has_pending_sync())
    {
      immediate_sync_requested_.store(false);
      surf_multirobot_msgs::msg::VoxelDelta sync;
      initialize_delta(sync, cloud, mode, true);
      sync.sensor_origin = realtime.sensor_origin;
      sync.base_version = 0U;
      for (const auto & [coord, cell] : cells_) {
        append_record(sync, coord, cell.static_known ?
          surf_multirobot_msgs::msg::VoxelDelta::STATE_OCCUPIED_STATIC :
          surf_multirobot_msgs::msg::VoxelDelta::STATE_OCCUPIED_DYNAMIC,
          cell.last_observation_time_ns);
      }
      for (auto it = tombstones_.begin(); it != tombstones_.end();) {
        if (version_ - it->second.version > static_cast<uint64_t>(tombstone_retention_scans_)) {
          it = tombstones_.erase(it);
          continue;
        }
        append_record(sync, it->first, it->second.state, it->second.observation_time_ns);
        ++it;
      }
      publish_delta(sync, surf_multirobot_msgs::msg::CompressedVoxelDelta::TRAFFIC_SYNC,
        0U, 0U, 0U, static_cast<uint32_t>(current.size()),
        static_prior_voxels, processing_start, transform_lookup_ms,
        point_preprocessing_ms, occupancy_selection_ms, clearing_ms,
        raw_serialization_ms, queue_wait_ms, static_cast<uint32_t>(cloud.data.size()),
        cloud.point_step);
      last_sync_time_ = now;
    }
  }

  geometry_msgs::msg::TransformStamped resolve_transform(
    const sensor_msgs::msg::PointCloud2 & cloud)
  {
    const auto timeout = rclcpp::Duration::from_seconds(transform_timeout_ms_ / 1000.0);
    if (transform_source_ == "tf") {
      return tf_buffer_->lookupTransform(
        map_frame_, cloud.header.frame_id, cloud.header.stamp, timeout);
    }

    nav_msgs::msg::Odometry::SharedPtr nearest;
    int64_t nearest_difference_ns = std::numeric_limits<int64_t>::max();
    const rclcpp::Time cloud_stamp(cloud.header.stamp);
    {
      std::lock_guard<std::mutex> lock(odometry_mutex_);
      for (const auto & candidate : odometry_history_) {
        const int64_t difference = std::llabs(
          (rclcpp::Time(candidate->header.stamp) - cloud_stamp).nanoseconds());
        if (difference < nearest_difference_ns) {
          nearest_difference_ns = difference;
          nearest = candidate;
        }
      }
    }
    if (!nearest || static_cast<double>(nearest_difference_ns) / 1.0e6 > pose_max_age_ms_) {
      throw tf2::TransformException("No odometry sample close enough to point-cloud timestamp");
    }
    if (!nearest->header.frame_id.empty() && nearest->header.frame_id != odometry_parent_frame_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Using odometry pose as %s although message frame is %s",
        odometry_parent_frame_.c_str(), nearest->header.frame_id.c_str());
    }

    const auto base_to_sensor_message = tf_buffer_->lookupTransform(
      base_frame_, cloud.header.frame_id,
      rclcpp::Time(0, 0, get_clock()->get_clock_type()), timeout);
    tf2::Transform parent_to_base;
    tf2::fromMsg(nearest->pose.pose, parent_to_base);
    tf2::Transform base_to_sensor;
    tf2::fromMsg(base_to_sensor_message.transform, base_to_sensor);
    geometry_msgs::msg::TransformStamped result;
    result.header.stamp = cloud.header.stamp;
    result.header.frame_id = odometry_parent_frame_;
    result.child_frame_id = cloud.header.frame_id;
    result.transform = tf2::toMsg(parent_to_base * base_to_sensor);
    return result;
  }

  void initialize_delta(
    surf_multirobot_msgs::msg::VoxelDelta & delta,
    const sensor_msgs::msg::PointCloud2 & cloud, uint8_t mode, bool full_refresh)
  {
    delta.header = cloud.header;
    delta.header.frame_id = map_frame_;
    delta.source_id = robot_name_;
    delta.map_epoch = map_epoch_;
    delta.version = version_;
    delta.base_version = version_ > 0U ? version_ - 1U : 0U;
    delta.operating_mode = mode;
    delta.full_refresh = full_refresh;
    delta.resolution = static_cast<float>(resolution_);
  }

  void publish_delta(
    surf_multirobot_msgs::msg::VoxelDelta & delta, uint8_t traffic_class,
    std::size_t raw_bytes, uint32_t raw_points, uint32_t valid_points,
    uint32_t unique_voxels, uint32_t static_prior_voxels,
    const std::chrono::steady_clock::time_point & processing_start,
    float transform_lookup_ms, float point_preprocessing_ms,
    float occupancy_selection_ms, float clearing_ms,
    float raw_serialization_ms, float queue_wait_ms,
    uint32_t raw_data_bytes, uint32_t point_step_bytes)
  {
    const bool is_sync =
      traffic_class == surf_multirobot_msgs::msg::CompressedVoxelDelta::TRAFFIC_SYNC;
    const std::size_t packet_budget = is_sync ?
      maximum_sync_packet_bytes_ : maximum_packet_bytes_;
    const std::size_t update_budget = is_sync ?
      maximum_sync_update_bytes_ : maximum_update_bytes_;
    // Budgeting keeps the prefix below. Order it so loss or budget pressure
    // preserves destructive and fast-changing evidence first, followed by
    // geometrically nearby evidence and finally distant static refreshes.
    std::vector<std::size_t> priority(delta.x.size());
    for (std::size_t index = 0U; index < priority.size(); ++index) {priority[index] = index;}
    const double origin_x = delta.sensor_origin.x / resolution_;
    const double origin_y = delta.sensor_origin.y / resolution_;
    const double origin_z = delta.sensor_origin.z / resolution_;
    auto rank = [&](std::size_t index) {
        const uint8_t state = delta.state[index];
        if (state == surf_multirobot_msgs::msg::VoxelDelta::STATE_DELETE) {return 0;}
        if (state == surf_multirobot_msgs::msg::VoxelDelta::STATE_FREE ||
          state == surf_multirobot_msgs::msg::VoxelDelta::STATE_OCCUPIED_DYNAMIC) {return 1;}
        return 2;
      };
    auto distance_squared = [&](std::size_t index) {
        const long double dx = static_cast<long double>(delta.x[index]) - origin_x;
        const long double dy = static_cast<long double>(delta.y[index]) - origin_y;
        const long double dz = static_cast<long double>(delta.z[index]) - origin_z;
        return dx * dx + dy * dy + dz * dz;
      };
    std::stable_sort(priority.begin(), priority.end(), [&](std::size_t lhs, std::size_t rhs) {
      const int lhs_rank = rank(lhs);
      const int rhs_rank = rank(rhs);
      return lhs_rank != rhs_rank ? lhs_rank < rhs_rank :
             distance_squared(lhs) < distance_squared(rhs);
    });
    auto reorder = [&](auto & values) {
        using Value = typename std::decay_t<decltype(values)>::value_type;
        std::vector<Value> ordered;
        ordered.reserve(values.size());
        for (const std::size_t index : priority) {ordered.push_back(values[index]);}
        values = std::move(ordered);
      };
    reorder(delta.x); reorder(delta.y); reorder(delta.z); reorder(delta.state);
    reorder(delta.observation_time_ns);

    const auto compression_start = std::chrono::steady_clock::now();
    std::vector<surf_multirobot_msgs::msg::CompressedVoxelDelta> packets;
    std::size_t offset = 0U;
    std::size_t update_bytes = 0U;
    rclcpp::Serialization<surf_multirobot_msgs::msg::CompressedVoxelDelta> serializer;
    auto encode_range = [&](std::size_t begin, std::size_t count,
      surf_multirobot_msgs::msg::CompressedVoxelDelta & wire, uint32_t & bytes) {
        auto part = delta;
        part.x.assign(delta.x.begin() + begin, delta.x.begin() + begin + count);
        part.y.assign(delta.y.begin() + begin, delta.y.begin() + begin + count);
        part.z.assign(delta.z.begin() + begin, delta.z.begin() + begin + count);
        part.state.assign(delta.state.begin() + begin, delta.state.begin() + begin + count);
        part.observation_time_ns.assign(delta.observation_time_ns.begin() + begin,
          delta.observation_time_ns.begin() + begin + count);
        const auto result = surf::comms::encode_delta(part, wire, compression_level_);
        if (!result.ok) {return result;}
        wire.traffic_class = traffic_class;
        wire.transmit_stamp = now();
        rclcpp::SerializedMessage serialized;
        serializer.serialize_message(&wire, &serialized);
        bytes = static_cast<uint32_t>(serialized.size());
        return result;
      };
    surf::comms::CodecResult result{true, {}};
    do {
      const std::size_t remaining = delta.x.size() - offset;
      std::size_t low = remaining == 0U ? 0U : 1U;
      std::size_t high = remaining;
      std::size_t best = 0U;
      surf_multirobot_msgs::msg::CompressedVoxelDelta best_wire;
      uint32_t best_bytes = 0U;
      while (low <= high) {
        const std::size_t candidate = low + (high - low) / 2U;
        surf_multirobot_msgs::msg::CompressedVoxelDelta wire;
        uint32_t bytes = 0U;
        result = encode_range(offset, candidate, wire, bytes);
        if (!result.ok) {break;}
        if (bytes <= packet_budget) {
          best = candidate; best_wire = std::move(wire); best_bytes = bytes; low = candidate + 1U;
        } else {
          if (candidate == 0U) {break;}
          high = candidate - 1U;
        }
      }
      if (!result.ok) {break;}
      if (best == 0U) {
        result = {false, "transport packet budget is too small for one voxel record"};
        break;
      }
      if (!packets.empty() && update_bytes + best_bytes > update_budget) {break;}
      update_bytes += best_bytes;
      packets.push_back(std::move(best_wire));
      offset += best;
    } while (offset < delta.x.size());
    if (delta.x.empty()) {
      surf_multirobot_msgs::msg::CompressedVoxelDelta wire;
      uint32_t bytes = 0U;
      result = encode_range(0U, 0U, wire, bytes);
      if (result.ok && bytes <= packet_budget) {packets.push_back(std::move(wire)); update_bytes = bytes;}
    }
    const float compression_latency_ms = static_cast<float>(
      std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - compression_start).count());
    if (!result.ok) {
      RCLCPP_ERROR(get_logger(), "Could not encode voxel delta: %s", result.error.c_str());
      return;
    }
    if (delta.full_refresh && offset != delta.x.size()) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
        "Full refresh requires more than transport.maximum_sync_update_bytes (%zu); not publishing a partial refresh",
        maximum_sync_update_bytes_);
      return;
    }
    if (!delta.full_refresh && offset != delta.x.size()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Realtime update byte budget selected %zu of %zu voxel records",
        offset, delta.x.size());
      // Selection tentatively marked occupied records as sent. Put records
      // outside the byte budget back into the next delta's candidate set.
      for (std::size_t index = offset; index < delta.x.size(); ++index) {
        const Coord coord{delta.x[index], delta.y[index], delta.z[index]};
        const auto cell = cells_.find(coord);
        if (cell != cells_.end()) {
          cell->second.last_sent_version = 0U;
        }
      }
    }
    const auto wire_serialization_start = std::chrono::steady_clock::now();
    const uint32_t chunk_count = static_cast<uint32_t>(packets.size());
    uint32_t wire_bytes = 0U;
    uint32_t payload_bytes = 0U;
    uint32_t uncompressed_bytes = 0U;
    uint32_t sent_voxels = 0U;
    for (uint32_t index = 0U; index < chunk_count; ++index) {
      auto & wire = packets[index];
      wire.chunk_index = index;
      wire.chunk_count = chunk_count;
      rclcpp::SerializedMessage serialized;
      serializer.serialize_message(&wire, &serialized);
      wire_bytes += static_cast<uint32_t>(serialized.size());
      payload_bytes += static_cast<uint32_t>(wire.payload.size());
      uncompressed_bytes += wire.uncompressed_bytes;
      sent_voxels += wire.voxel_count;
    }
    const float wire_serialization_ms = static_cast<float>(
      std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - wire_serialization_start).count());
    surf_multirobot_msgs::msg::PipelineMetrics metrics;
    metrics.header.stamp = delta.header.stamp;
    metrics.header.frame_id = map_frame_;
    metrics.transmit_stamp = packets.front().transmit_stamp;
    metrics.source_id = robot_name_;
    metrics.map_epoch = delta.map_epoch;
    metrics.version = delta.version;
    metrics.operating_mode = delta.operating_mode;
    metrics.traffic_class = traffic_class;
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      metrics.input_rate_hz = static_cast<float>(input_rate_hz_);
      metrics.stale_input_drops = stale_input_drops_;
    }
    metrics.raw_points = raw_points;
    metrics.valid_points = valid_points;
    metrics.unique_voxels = unique_voxels;
    metrics.static_prior_voxels = static_prior_voxels;
    metrics.selected_voxels = sent_voxels;
    metrics.packet_count = chunk_count;
    metrics.packet_budget_bytes = static_cast<uint32_t>(packet_budget);
    metrics.update_budget_bytes = static_cast<uint32_t>(update_budget);
    const uint32_t after_static = unique_voxels > static_prior_voxels ?
      unique_voxels - static_prior_voxels : 0U;
    metrics.temporal_suppressed_voxels = after_static > metrics.selected_voxels ?
      after_static - metrics.selected_voxels : 0U;
    metrics.raw_serialized_bytes = static_cast<uint32_t>(raw_bytes);
    metrics.raw_data_bytes = raw_data_bytes;
    metrics.point_step_bytes = point_step_bytes;
    metrics.uncompressed_bytes = uncompressed_bytes;
    metrics.payload_bytes = payload_bytes;
    metrics.wire_bytes = wire_bytes;
    metrics.codec = packets.size() == 1U ? packets.front().codec : "chunked-svd2";
    metrics.compression_ratio = payload_bytes > 0U ?
      static_cast<float>(uncompressed_bytes) / payload_bytes : 0.0F;
    metrics.compression_latency_ms = compression_latency_ms;
    metrics.raw_serialization_ms = raw_serialization_ms;
    metrics.wire_serialization_ms = wire_serialization_ms;
    metrics.queue_wait_ms = queue_wait_ms;
    metrics.transform_lookup_ms = transform_lookup_ms;
    metrics.point_preprocessing_ms = point_preprocessing_ms;
    metrics.occupancy_selection_ms = occupancy_selection_ms;
    metrics.clearing_ms = clearing_ms;
    for (std::size_t index = 0U; index < offset; ++index) {
      const uint8_t state = delta.state[index];
      if (state == surf_multirobot_msgs::msg::VoxelDelta::STATE_FREE) {
        ++metrics.free_updates;
      } else if (state == surf_multirobot_msgs::msg::VoxelDelta::STATE_DELETE) {
        ++metrics.delete_updates;
      } else {
        ++metrics.occupied_updates;
      }
    }
    metrics.processing_latency_ms = static_cast<float>(
      std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - processing_start).count());
    cumulative_raw_bytes_ += raw_bytes;
    cumulative_wire_bytes_ += wire_bytes;
    metrics.cumulative_raw_bytes = cumulative_raw_bytes_;
    metrics.cumulative_wire_bytes = cumulative_wire_bytes_;
    metrics_publisher_->publish(metrics);
    enqueue_packets(
      traffic_class, std::move(packets), wire_bytes, compression_latency_ms);
  }

  std::string robot_name_;
  std::string map_frame_;
  std::string input_topic_;
  std::string static_map_topic_;
  std::string realtime_topic_;
  std::string realtime_ack_topic_;
  std::string realtime_ack_metrics_topic_;
  std::string sync_topic_;
  std::string sync_ack_topic_;
  std::string sync_request_topic_;
  std::string sync_status_topic_;
  std::string link_metrics_topic_;
  std::string metrics_topic_;
  std::string transform_source_;
  std::string odometry_topic_;
  std::string odometry_parent_frame_;
  std::string base_frame_;
  double pose_max_age_ms_{50.0};
  double transform_timeout_ms_{500.0};
  double resolution_{0.05};
  double min_range_{0.75};
  double max_range_{50.0};
  double min_z_{0.25};
  double max_z_{20.0};
  double self_radius_{0.7};
  bool humanoid_mask_enabled_{true};
  std::string humanoid_odometry_topic_;
  double humanoid_mask_size_x_{0.70};
  double humanoid_mask_size_y_{0.66};
  double humanoid_mask_size_z_{0.825};
  double humanoid_mask_center_z_{0.4125};
  double humanoid_mask_padding_{0.20};
  double humanoid_mask_max_age_ms_{150.0};
  int static_min_hits_{20};
  int clear_min_misses_{3};
  int delta_refresh_scans_{50};
  int tombstone_retention_scans_{500};
  int dynamic_retention_scans_{20};
  int maximum_ray_voxels_{1200};
  int maximum_clear_rays_{256};
  double sync_interval_seconds_{10.0};
  int compression_level_{1};
  std::size_t maximum_packet_bytes_{1200U};
  std::size_t maximum_update_bytes_{12000U};
  std::size_t maximum_sync_packet_bytes_{65536U};
  std::size_t maximum_sync_update_bytes_{16U * 1024U * 1024U};
  int64_t chunk_publish_interval_ms_{15};
  double sync_ack_timeout_seconds_{2.0};
  double sync_ack_margin_seconds_{2.0};
  double realtime_ack_timeout_seconds_{2.0};
  double sync_max_interval_seconds_{60.0};
  double sync_target_duty_cycle_{0.10};
  std::atomic<double> next_sync_interval_seconds_{10.0};
  bool adaptive_enabled_{true};
  int manual_mode_{-1};
  uint64_t map_epoch_{0U};
  uint64_t version_{0U};
  uint64_t cumulative_raw_bytes_{0U};
  uint64_t cumulative_wire_bytes_{0U};
  uint64_t stale_input_drops_{0U};
  double input_rate_hz_{0.0};
  std::chrono::steady_clock::time_point last_input_time_{};
  double last_sync_time_{0.0};

  AdaptiveModeController adaptive_;
  SpatialMap<CellState> cells_;
  DynamicExpiry dynamic_expiry_;
  struct Tombstone
  {
    uint64_t version{0U};
    uint8_t state{surf_multirobot_msgs::msg::VoxelDelta::STATE_DELETE};
    uint64_t observation_time_ns{0U};
  };
  SpatialMap<Tombstone> tombstones_;
  using CoordSet = std::unordered_set<Coord, CoordHash>;
  std::shared_ptr<const CoordSet> static_map_{std::make_shared<const CoordSet>()};
  bool static_prior_initialized_{false};
  std::mutex static_map_mutex_;

  double available_bandwidth_mbps_{4.0};
  uint32_t link_queue_depth_{0U};
  std::mutex link_mutex_;

  std::atomic<bool> stop_{false};
  std::atomic<bool> immediate_sync_requested_{false};
  std::thread worker_;
  std::mutex queue_mutex_;
  std::condition_variable queue_condition_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;
  std::chrono::steady_clock::time_point latest_cloud_received_{};
  std::mutex transport_queue_mutex_;
  std::deque<surf_multirobot_msgs::msg::CompressedVoxelDelta> realtime_packet_queue_;
  std::deque<surf_multirobot_msgs::msg::CompressedVoxelDelta> sync_packet_queue_;
  std::vector<surf_multirobot_msgs::msg::CompressedVoxelDelta> pending_sync_packets_;
  uint64_t pending_sync_epoch_{0U};
  uint64_t pending_sync_version_{0U};
  uint64_t last_acknowledged_sync_version_{0U};
  std::chrono::steady_clock::time_point sync_attempt_time_{};
  std::chrono::steady_clock::time_point sync_first_attempt_time_{};
  uint32_t pending_sync_wire_bytes_{0U};
  uint32_t pending_sync_retry_count_{0U};
  float pending_sync_encoding_ms_{0.0F};
  float pending_sync_estimated_transfer_ms_{0.0F};
  float pending_sync_transfer_ms_{0.0F};
  double pending_sync_ack_timeout_seconds_{2.0};
  struct RealtimeAckPending
  {
    std::chrono::steady_clock::time_point first_chunk{};
    std::chrono::steady_clock::time_point final_chunk{};
  };
  std::unordered_map<uint64_t, RealtimeAckPending> pending_realtime_acks_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::mutex odometry_mutex_;
  std::deque<nav_msgs::msg::Odometry::SharedPtr> odometry_history_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  std::mutex humanoid_odometry_mutex_;
  std::deque<nav_msgs::msg::Odometry::SharedPtr> humanoid_odometry_history_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
    humanoid_odometry_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr static_map_subscription_;
  rclcpp::Subscription<surf_multirobot_msgs::msg::LinkMetrics>::SharedPtr
    link_metrics_subscription_;
  rclcpp::Subscription<surf_multirobot_msgs::msg::SyncAck>::SharedPtr
    sync_ack_subscription_;
  rclcpp::Subscription<surf_multirobot_msgs::msg::RealtimeAck>::SharedPtr
    realtime_ack_subscription_;
  rclcpp::Subscription<surf_multirobot_msgs::msg::SyncRequest>::SharedPtr
    sync_request_subscription_;
  rclcpp::Publisher<surf_multirobot_msgs::msg::CompressedVoxelDelta>::SharedPtr
    realtime_publisher_;
  rclcpp::Publisher<surf_multirobot_msgs::msg::CompressedVoxelDelta>::SharedPtr
    sync_publisher_;
  rclcpp::Publisher<surf_multirobot_msgs::msg::PipelineMetrics>::SharedPtr metrics_publisher_;
  rclcpp::Publisher<surf_multirobot_msgs::msg::RealtimeAckMetrics>::SharedPtr
    realtime_ack_metrics_publisher_;
  rclcpp::Publisher<surf_multirobot_msgs::msg::SyncStatus>::SharedPtr sync_status_publisher_;
  rclcpp::TimerBase::SharedPtr realtime_publish_timer_;
  rclcpp::TimerBase::SharedPtr sync_publish_timer_;
  rclcpp::TimerBase::SharedPtr sync_retry_timer_;
};

}  // namespace surf_drone

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<surf_drone::DroneScanSender>());
  rclcpp::shutdown();
  return 0;
}
