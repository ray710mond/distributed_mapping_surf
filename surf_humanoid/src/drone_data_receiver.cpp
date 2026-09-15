#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <map>
#include <set>
#include <memory>
#include <limits>
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
#include "surf_multirobot_comms/temporal_voxels.hpp"

namespace surf_humanoid
{
namespace detail
{
struct RayCoord {int32_t x, y, z; bool operator==(const RayCoord & b) const {
  return x == b.x && y == b.y && z == b.z;}};
RayCoord ray_quantize(double x, double y, double z, double resolution) {
  return {static_cast<int32_t>(std::floor(x / resolution)),
    static_cast<int32_t>(std::floor(y / resolution)),
    static_cast<int32_t>(std::floor(z / resolution))};
}
template<class Visitor>
void visit_ray(const geometry_msgs::msg::Point & origin, const geometry_msgs::msg::Point & endpoint,
  double resolution, std::size_t limit, Visitor visit)
{
  RayCoord c = ray_quantize(origin.x, origin.y, origin.z, resolution);
  const RayCoord end = ray_quantize(endpoint.x, endpoint.y, endpoint.z, resolution);
  const double origins[] = {origin.x, origin.y, origin.z};
  const double directions[] = {endpoint.x - origin.x, endpoint.y - origin.y, endpoint.z - origin.z};
  int32_t * axes[] = {&c.x, &c.y, &c.z};
  std::array<int, 3> step{};
  std::array<double, 3> next{}, increment{};
  for (int axis = 0; axis < 3; ++axis) {
    if (directions[axis] == 0) {
      next[axis] = increment[axis] = std::numeric_limits<double>::infinity();
    } else {
      step[axis] = directions[axis] > 0 ? 1 : -1;
      const double boundary = (*axes[axis] + (step[axis] > 0 ? 1.0 : 0.0)) * resolution;
      next[axis] = (boundary - origins[axis]) / directions[axis];
      increment[axis] = resolution / std::abs(directions[axis]);
    }
  }
  for (std::size_t visited = 0; !(c == end) && visited < limit; ++visited) {
    if (!visit(c)) break;
    const double crossing = *std::min_element(next.begin(), next.end());
    if (crossing >= 1.0) break;
    for (int axis = 0; axis < 3; ++axis) if (next[axis] <= crossing + 1e-12) {
      *axes[axis] += step[axis]; next[axis] += increment[axis];
    }
  }
}
}
using detail::RayCoord;
using detail::visit_ray;

class DroneDataReceiver : public rclcpp::Node
{
public:
  explicit DroneDataReceiver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("drone_data_receiver", options)
  {
    robot_name_ = declare_parameter<std::string>("robot_name", "humanoid");
    realtime_topic_ = declare_parameter<std::string>(
      "realtime_topic", "/" + robot_name_ + "/transport/realtime_tx");
    realtime_ack_topic_ = declare_parameter<std::string>(
      "realtime_ack_topic", "/" + robot_name_ + "/transport/realtime_ack");
    backlog_topic_ = declare_parameter<std::string>(
      "sync_topic", "/" + robot_name_ + "/transport/sync_tx");
    backlog_ack_topic_ = declare_parameter<std::string>(
      "sync_ack_topic", "/" + robot_name_ + "/transport/sync_ack");
    backlog_request_topic_ = declare_parameter<std::string>(
      "sync_request_topic", "/" + robot_name_ + "/transport/sync_request");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/" + robot_name_ + "/comm/drone_voxel_delta");
    metrics_topic_ = declare_parameter<std::string>(
      "metrics_topic", "/" + robot_name_ + "/comm/delivery_metrics");
    maximum_uncompressed_bytes_ = static_cast<std::size_t>(std::max<int64_t>(
      1024, declare_parameter<int64_t>("maximum_uncompressed_bytes", 64 * 1024 * 1024)));
    ray_min_range_ = declare_parameter<double>("free_ray.min_range", 0.75);
    ray_self_radius_ = declare_parameter<double>("free_ray.self_radius", 0.7);
    ray_min_z_ = declare_parameter<double>("free_ray.min_z", 0.25);
    ray_max_z_ = declare_parameter<double>("free_ray.max_z", 20.0);
    ray_radius_ = declare_parameter<double>("free_ray.radius_metres", 100.0);
    ray_resolution_ = declare_parameter<double>("free_ray.resolution", 0.05);
    ray_max_voxels_ = static_cast<std::size_t>(std::max<int64_t>(
      1, declare_parameter<int64_t>("free_ray.maximum_voxels", 1200)));
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

    // Local reliable delivery; recovery requests reconstruct lost receiver knowledge.
    publisher_ = create_publisher<surf_multirobot_msgs::msg::VoxelDelta>(
      output_topic_, rclcpp::QoS(rclcpp::KeepLast(128)).reliable().transient_local());
    metrics_publisher_ = create_publisher<surf_multirobot_msgs::msg::DeliveryMetrics>(
      metrics_topic_, rclcpp::QoS(10));
    realtime_ack_publisher_ = create_publisher<surf_multirobot_msgs::msg::RealtimeAck>(
      realtime_ack_topic_, rclcpp::QoS(1).best_effort().durability_volatile());
    backlog_ack_publisher_ = create_publisher<surf_multirobot_msgs::msg::SyncAck>(
      backlog_ack_topic_, rclcpp::QoS(10).reliable().transient_local());
    backlog_request_publisher_ = create_publisher<surf_multirobot_msgs::msg::SyncRequest>(
      backlog_request_topic_, rclcpp::QoS(10).reliable().transient_local());
    realtime_subscription_ = create_subscription<
      surf_multirobot_msgs::msg::CompressedVoxelDelta>(
      realtime_topic_, surf::comms::realtime_qos(),
      std::bind(&DroneDataReceiver::receive, this, std::placeholders::_1));
    backlog_subscription_ = create_subscription<surf_multirobot_msgs::msg::CompressedVoxelDelta>(
      backlog_topic_, surf::comms::sync_qos(),
      std::bind(&DroneDataReceiver::receive, this, std::placeholders::_1));

    local_replay_timer_ = create_wall_timer(std::chrono::seconds(5), [this]() {
      // Local DDS only: retain accepted state across downstream subscriber restarts.
      for (const auto & [id, metadata] : accepted_metadata_)
        publisher_->publish(temporal_sources_.at(id).snapshot(metadata));
    });
    recovery_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
      if (!information_epochs_.empty()) return;
      surf_multirobot_msgs::msg::SyncRequest request;
      request.header.stamp = now();  // Empty source ID discovers all peers after receiver restart.
      request.reason = "receiver startup: delivery knowledge unknown";
      backlog_request_publisher_->publish(request);
    });
    RCLCPP_INFO(get_logger(), "%s receiver: [%s, %s] -> %s",
      robot_name_.c_str(), realtime_topic_.c_str(), backlog_topic_.c_str(), output_topic_.c_str());
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
    backlog_request_publisher_->publish(request);
  }

  void receive(const surf_multirobot_msgs::msg::CompressedVoxelDelta::SharedPtr packet)
  {
    if (packet->operating_mode != 4) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Legacy adaptive/snapshot protocol rejected; upgrade both peers together");
      return;
    }
    receive_information(*packet);
  }

  bool expand_rays(surf_multirobot_msgs::msg::VoxelDelta & delta)
  {
    if (delta.ray_flags.empty()) return true;
    if (!std::isfinite(ray_resolution_) || ray_resolution_ <= 0 ||
      std::abs(ray_resolution_ - delta.resolution) > 1e-6) return false;
    const auto count = delta.x.size();
    if (delta.ray_flags.size() != count || delta.ray_origins.size() != count ||
      delta.ray_endpoints.size() != count) return false;
    auto expanded = delta;
    expanded.x.clear(); expanded.y.clear(); expanded.z.clear();
    expanded.state.clear(); expanded.observation_time_ns.clear();
    expanded.ray_flags.clear(); expanded.ray_origins.clear(); expanded.ray_endpoints.clear();
    std::set<std::array<int32_t, 3>> occupied_endpoints;
    for (std::size_t i = 0; i < count; ++i)
      if (delta.state[i] == delta.STATE_OCCUPIED_STATIC || delta.state[i] == delta.STATE_OCCUPIED_DYNAMIC)
        occupied_endpoints.insert({delta.x[i], delta.y[i], delta.z[i]});
    auto add = [&](RayCoord c, uint8_t state, uint64_t stamp) {
      expanded.x.push_back(c.x); expanded.y.push_back(c.y); expanded.z.push_back(c.z);
      expanded.state.push_back(state); expanded.observation_time_ns.push_back(stamp);
    };
    for (std::size_t i = 0; i < count; ++i) {
      const RayCoord target{delta.x[i], delta.y[i], delta.z[i]};
      if (!delta.ray_flags[i]) {
        add(target, delta.state[i], delta.observation_time_ns[i]);
        continue;
      }
      const bool occupied_ray = delta.ray_flags[i] == 2;
      if (delta.ray_flags[i] != 1 && !occupied_ray) return false;
      if (!occupied_ray && delta.state[i] != surf_multirobot_msgs::msg::VoxelDelta::STATE_FREE) return false;
      if (occupied_ray && delta.state[i] != delta.STATE_OCCUPIED_STATIC &&
        delta.state[i] != delta.STATE_OCCUPIED_DYNAMIC) return false;
      const auto & origin = delta.ray_origins[i];
      geometry_msgs::msg::Point endpoint = delta.ray_endpoints[i];
      if (occupied_ray) {
        endpoint.x = (static_cast<double>(target.x) + .5) * ray_resolution_;
        endpoint.y = (static_cast<double>(target.y) + .5) * ray_resolution_;
        endpoint.z = (static_cast<double>(target.z) + .5) * ray_resolution_;
      }
      if (!std::isfinite(origin.x + origin.y + origin.z + endpoint.x + endpoint.y + endpoint.z))
        return false;
      const double coordinate_limit =
        (static_cast<double>(std::numeric_limits<int32_t>::max()) - ray_max_voxels_) * ray_resolution_;
      if (std::max({std::abs(origin.x), std::abs(origin.y), std::abs(origin.z),
        std::abs(endpoint.x), std::abs(endpoint.y), std::abs(endpoint.z)}) > coordinate_limit)
        return false;
      bool reached = false;
      const std::size_t before = expanded.x.size();
      visit_ray(origin, endpoint, ray_resolution_, ray_max_voxels_, [&](RayCoord c) {
        if (occupied_ray && occupied_endpoints.count({c.x, c.y, c.z})) return false;
        const double cx = (static_cast<double>(c.x) + .5) * ray_resolution_;
        const double cy = (static_cast<double>(c.y) + .5) * ray_resolution_;
        const double cz = (static_cast<double>(c.z) + .5) * ray_resolution_;
        const double distance = std::sqrt((cx-origin.x)*(cx-origin.x) +
          (cy-origin.y)*(cy-origin.y) + (cz-origin.z)*(cz-origin.z));
        if (distance > ray_radius_) return false;
        if (distance >= std::max(ray_min_range_, ray_self_radius_) &&
          cz >= ray_min_z_ && cz <= ray_max_z_)
          add(c, surf_multirobot_msgs::msg::VoxelDelta::STATE_FREE, delta.observation_time_ns[i]);
        if (c == target) {reached = true; return false;}
        return true;
      });
      // A grazing beam or filter mismatch can diverge from sender traversal.
      // Keep the sampled observation without claiming an unmatched prefix.
      if (occupied_ray) {
        add(target, delta.state[i], delta.observation_time_ns[i]);
      } else if (!reached) {
        expanded.x.resize(before); expanded.y.resize(before); expanded.z.resize(before);
        expanded.state.resize(before); expanded.observation_time_ns.resize(before);
        add(target, delta.state[i], delta.observation_time_ns[i]);
      }
      if (expanded.x.size() > 65536) return false;
    }
    delta = std::move(expanded);
    return true;
  }

  void receive_information(const surf_multirobot_msgs::msg::CompressedVoxelDelta & packet)
  {
    const auto started = std::chrono::steady_clock::now();
    surf_multirobot_msgs::msg::VoxelDelta delta;
    const auto decoded = surf::comms::decode_delta(packet, delta, maximum_uncompressed_bytes_);
    const auto decoded_at = std::chrono::steady_clock::now();
    if (!decoded.ok || packet.source_id.empty() || packet.full_refresh ||
      packet.chunk_index != 0 || packet.chunk_count != 1 ||
      !std::isfinite(packet.resolution) || packet.resolution <= 0 ||
      std::any_of(delta.state.begin(), delta.state.end(), [](uint8_t s) {return s < 1 || s > 5;})) return;
    if (!expand_rays(delta)) return;
    const auto expanded_at = std::chrono::steady_clock::now();
    auto & temporal = temporal_sources_[packet.source_id];
    const auto stale_before = temporal.stale;
    const auto regressions_before = temporal.attempted_regressions;
    if (!temporal.filter(delta)) return;
    // Independent packet application permits spatial priority reordering. Even an
    // entirely stale retry is ACKed so a lost ACK cannot create endless debt.
    accepted_metadata_[packet.source_id] = delta;
    publisher_->publish(delta);
    if (packet.traffic_class == 2) publish_sync_ack(packet);
    else {
      surf_multirobot_msgs::msg::RealtimeAck ack;
      ack.map_epoch = packet.map_epoch; ack.version = packet.version;
      realtime_ack_publisher_->publish(ack);
    }
    if (information_epochs_[packet.source_id] != packet.map_epoch) {
      information_epochs_[packet.source_id] = packet.map_epoch;
      request_sync(packet, 0, "receiver delivery knowledge unknown; enqueue backlog");
    }
    surf_multirobot_msgs::msg::DeliveryMetrics m;
    m.header.stamp = now(); m.sensor_stamp = packet.header.stamp; m.transmit_stamp = packet.transmit_stamp;
    m.source_id = packet.source_id; m.map_epoch = packet.map_epoch; m.version = packet.version;
    m.traffic_class = packet.traffic_class; m.voxel_count = packet.voxel_count; m.chunk_count = 1;
    m.decode_latency_ms = std::chrono::duration<double, std::milli>(decoded_at - started).count();
    m.codec_reconstruction_ms = std::chrono::duration<double, std::milli>(expanded_at - started).count();
    // No downstream map integration ACK is available; do not report default zero latency.
    m.end_to_end_latency_ms = std::numeric_limits<float>::quiet_NaN();
    m.transport_latency_ms = std::numeric_limits<float>::quiet_NaN();
    m.accepted_voxel_count = delta.observation_time_ns.size();
    m.oldest_accepted_observation_age_s = m.newest_accepted_observation_age_s =
      m.mean_accepted_observation_age_s = std::numeric_limits<double>::quiet_NaN();
    if (m.accepted_voxel_count) {
      const double observation_now = now().seconds();
      double sum_age = 0;
      m.oldest_accepted_observation_age_s = -std::numeric_limits<double>::infinity();
      m.newest_accepted_observation_age_s = std::numeric_limits<double>::infinity();
      for (const auto stamp : delta.observation_time_ns) {
        const double age = observation_now - stamp / 1e9;
        sum_age += age;
        m.oldest_accepted_observation_age_s = std::max(m.oldest_accepted_observation_age_s, age);
        m.newest_accepted_observation_age_s = std::min(m.newest_accepted_observation_age_s, age);
      }
      m.mean_accepted_observation_age_s = sum_age / m.accepted_voxel_count;
    }
    m.accepted = true; m.stale_voxels_rejected = temporal.stale - stale_before;
    m.applied_temporal_regressions = 0;
    m.attempted_temporal_regressions = temporal.attempted_regressions - regressions_before;
    rclcpp::Serialization<surf_multirobot_msgs::msg::CompressedVoxelDelta> serializer;
    rclcpp::SerializedMessage serialized; serializer.serialize_message(&packet, &serialized);
    m.wire_bytes = serialized.size();
    m.sender_to_receiver_ms = (now() - rclcpp::Time(packet.transmit_stamp)).seconds() * 1000;
    m.sensor_to_receiver_ms = (now() - rclcpp::Time(packet.header.stamp)).seconds() * 1000;
    m.receiver_processing_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
    metrics_publisher_->publish(m);
  }
  std::unordered_map<std::string, surf::comms::TemporalVoxels> temporal_sources_;
  std::unordered_map<std::string, uint64_t> information_epochs_;

  void publish_sync_ack(const surf_multirobot_msgs::msg::CompressedVoxelDelta & packet)
  {
    surf_multirobot_msgs::msg::SyncAck ack;
    ack.header.stamp = now();
    ack.header.frame_id = packet.header.frame_id;
    ack.source_id = packet.source_id;
    ack.map_epoch = packet.map_epoch;
    ack.version = packet.version;
    backlog_ack_publisher_->publish(ack);
  }

  rclcpp::TimerBase::SharedPtr recovery_timer_, local_replay_timer_;
  std::unordered_map<std::string, surf_multirobot_msgs::msg::VoxelDelta> accepted_metadata_;
  std::string robot_name_;
  std::string realtime_topic_;
  std::string realtime_ack_topic_;
  std::string backlog_topic_;
  std::string backlog_ack_topic_;
  std::string backlog_request_topic_;
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
  double ray_min_range_{0.75}, ray_self_radius_{0.7};
  double ray_min_z_{0.25}, ray_max_z_{20.0}, ray_radius_{100.0};
  double ray_resolution_{0.05};
  std::size_t ray_max_voxels_{1200};
  std::chrono::steady_clock::time_point last_sync_request_time_{};

  rclcpp::Publisher<surf_multirobot_msgs::msg::VoxelDelta>::SharedPtr publisher_;
  rclcpp::Publisher<surf_multirobot_msgs::msg::DeliveryMetrics>::SharedPtr metrics_publisher_;
  rclcpp::Publisher<surf_multirobot_msgs::msg::RealtimeAck>::SharedPtr
    realtime_ack_publisher_;
  rclcpp::Publisher<surf_multirobot_msgs::msg::SyncAck>::SharedPtr backlog_ack_publisher_;
  rclcpp::Publisher<surf_multirobot_msgs::msg::SyncRequest>::SharedPtr
    backlog_request_publisher_;
  rclcpp::Subscription<surf_multirobot_msgs::msg::CompressedVoxelDelta>::SharedPtr
    realtime_subscription_;
  rclcpp::Subscription<surf_multirobot_msgs::msg::CompressedVoxelDelta>::SharedPtr
    backlog_subscription_;
};

}  // namespace surf_humanoid

#ifndef SURF_NODE_TEST
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<surf_humanoid::DroneDataReceiver>());
  rclcpp::shutdown();
  return 0;
}

#endif
