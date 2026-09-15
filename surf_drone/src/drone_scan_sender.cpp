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
#include "surf_drone/information_allocation.hpp"
#include "surf_drone/information_debt.hpp"
#include "surf_multirobot_msgs/msg/allocation_metrics.hpp"
#include "surf_drone/communication_state.hpp"
#include "surf_drone/observed_ray.hpp"
#include "surf_multirobot_comms/qos_profiles.hpp"
#include "surf_multirobot_comms/voxel_codec.hpp"

namespace surf_drone
{
namespace
{


void append_record(
  surf_multirobot_msgs::msg::VoxelDelta & delta,
  const Coord & coord, uint8_t state, uint64_t observation_time_ns,
  uint8_t ray_flag = 0, geometry_msgs::msg::Point ray_origin = geometry_msgs::msg::Point(),
  geometry_msgs::msg::Point ray_endpoint = geometry_msgs::msg::Point())
{
  delta.x.push_back(coord.x);
  delta.y.push_back(coord.y);
  delta.z.push_back(coord.z);
  delta.state.push_back(state);
  delta.observation_time_ns.push_back(observation_time_ns);
  delta.ray_flags.push_back(ray_flag);
  delta.ray_origins.push_back(ray_origin);
  delta.ray_endpoints.push_back(ray_endpoint);
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
  explicit DroneScanSender(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("drone_scan_sender", options)
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
    backlog_topic_ = declare_parameter<std::string>(
      "sync_topic", "/" + robot_name_ + "/transport/sync_tx");
    backlog_ack_topic_ = declare_parameter<std::string>(
      "sync_ack_topic", "/" + robot_name_ + "/transport/sync_ack");
    backlog_request_topic_ = declare_parameter<std::string>(
      "sync_request_topic", "/" + robot_name_ + "/transport/sync_request");
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
    dynamic_retention_scans_ = static_cast<int>(std::max<int64_t>(
      1, declare_parameter<int64_t>("dynamic_retention_scans", 20)));
    maximum_ray_voxels_ = static_cast<int>(std::max<int64_t>(
      1, declare_parameter<int64_t>("maximum_ray_voxels", 1200)));
    maximum_clear_rays_ = static_cast<int>(std::max<int64_t>(
      1, declare_parameter<int64_t>("maximum_clear_rays", 256)));
    maximum_ray_work_ = static_cast<std::size_t>(std::max<int64_t>(
      1, declare_parameter<int64_t>("mapping.maximum_ray_cells_per_scan", 32768)));
    new_free_ray_stride_ = static_cast<std::size_t>(std::max<int64_t>(
      0, declare_parameter<int64_t>("mapping.new_free_ray_stride", 1)));
    mapping_radius_ = declare_parameter<double>("mapping.radius_metres", 10.0);
    if (!std::isfinite(resolution_) || resolution_ <= 0 ||
      !std::isfinite(mapping_radius_) || mapping_radius_ <= 0)
      throw std::invalid_argument("mapping resolution and radius must be positive and finite");
    compression_level_ = static_cast<int>(declare_parameter<int64_t>("compression_level", 1));
    maximum_packet_bytes_ = static_cast<std::size_t>(std::max<int64_t>(256,
      declare_parameter<int64_t>("transport.maximum_packet_bytes", 1200)));
    realtime_ack_timeout_seconds_ = declare_parameter<double>("delivery.ack_timeout_seconds", 4.0);
    control_dt_ = declare_parameter<double>("allocation.dt", 0.1);
    schedule_hz_[0] = declare_parameter<double>("scheduling.delta_hz", 10.0);
    schedule_hz_[1] = declare_parameter<double>("scheduling.backlog_hz", 10.0);
    maximum_control_entries_ = static_cast<std::size_t>(std::max<int64_t>(16,
      declare_parameter<int64_t>("scheduling.maximum_entries_per_cycle", 4096)));
    maximum_cluster_entries_ = static_cast<std::size_t>(std::max<int64_t>(16,
      declare_parameter<int64_t>("scheduling.maximum_cluster_traversal_entries", 8192)));
    cluster_scheduling_enabled_ = declare_parameter<bool>("scheduling.cluster_enabled", true);
    cluster_neighbour_radius_ = static_cast<int>(std::clamp<int64_t>(
      declare_parameter<int64_t>("scheduling.cluster_neighbour_radius_voxels", 2), 1, 4));
    spatial_change_weight_ = declare_parameter<double>("scheduling.spatial_change_weight", 1.0);
    if (!std::isfinite(spatial_change_weight_) || spatial_change_weight_ < 0)
      throw std::invalid_argument("spatial change weight must be finite and nonnegative");
    starvation_fraction_ = declare_parameter<double>("scheduling.starvation_fraction", 0.1);
    starvation_age_seconds_ = declare_parameter<double>("scheduling.starvation_age_seconds", 30.0);
    for (double rate : schedule_hz_) if (!std::isfinite(rate) || rate <= 0)
      throw std::invalid_argument("scheduling frequencies must be finite and positive");
    defer_seconds_ = declare_parameter<double>("delivery.defer_seconds", 1.5);
    capacity_timeout_ = declare_parameter<double>("capacity.telemetry_timeout_seconds", 3.0);
    for (int i = 0; i < 2; ++i) {
      const std::string prefix = i == 0 ? "priority.delta." : "priority.backlog.";
      auto & w = weights_[i];
      w.base = declare_parameter<double>(prefix + "base", 1.0);
      w.age = declare_parameter<double>(prefix + "age", i == 0 ? 1.0 : 2.0);
      w.proximity = declare_parameter<double>(prefix + "proximity", 1.0);
      w.dynamic = declare_parameter<double>(prefix + "dynamic", 1.0);
      w.occupied = declare_parameter<double>(prefix + "occupied", 2.0);
      w.destructive = declare_parameter<double>(prefix + "destructive", 1.0);
      w.age_seconds = declare_parameter<double>(prefix + "age_seconds", 10.0);
      w.distance_metres = declare_parameter<double>(prefix + "distance_metres", 10.0);
      if (!std::isfinite(w.base + w.age + w.proximity + w.dynamic + w.destructive + w.occupied +
        w.age_seconds + w.distance_metres) || w.base <= 0 || w.age < 0 || w.proximity < 0 || w.dynamic < 0 ||
        w.destructive < 0 || w.occupied < 0 || w.age_seconds <= 0 || w.distance_metres <= 0)
        throw std::invalid_argument("invalid priority weights");
    }
    if (!std::isfinite(control_dt_) || control_dt_ <= 0 ||
      !std::isfinite(realtime_ack_timeout_seconds_) || realtime_ack_timeout_seconds_ <= 0 ||
      !std::isfinite(defer_seconds_) || defer_seconds_ < 0 ||
      !std::isfinite(capacity_timeout_) || capacity_timeout_ <= 0 ||
      !std::isfinite(starvation_fraction_) || starvation_fraction_ < 0 ||
      starvation_fraction_ > 1 || !std::isfinite(starvation_age_seconds_) ||
      starvation_age_seconds_ < 0)
      throw std::invalid_argument("invalid allocator timing/capacity");
    auto model = allocator_.model();
    for (auto item : {std::make_pair("a", &model.a), std::make_pair("b", &model.b),
      std::make_pair("q", &model.q), std::make_pair("r", &model.r)}) {
      std::vector<double> defaults;
      for (int row = 0; row < 2; ++row) for (int col = 0; col < 2; ++col)
        defaults.push_back((*item.second)(row, col));
      auto values = declare_parameter<std::vector<double>>(std::string("allocation.") + item.first, defaults);
      if (values.size() != 4) throw std::invalid_argument("allocation matrices require 4 row-major values");
      for (int j = 0; j < 4; ++j) (*item.second)(j / 2, j % 2) = values[j];
    }
    std::string error;
    if (!allocator_.update(model, error)) RCLCPP_ERROR(get_logger(), "Using development controller: %s", error.c_str());
    matrix_callback_ = add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> & ps) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      auto candidate = allocator_.model();
      rcl_interfaces::msg::SetParametersResult result; result.successful = true;
      bool changed = false;
      for (const auto & p : ps) {
        InformationAllocationController::Matrix * m = nullptr;
        if (p.get_name() == "allocation.a") m = &candidate.a;
        if (p.get_name() == "allocation.b") m = &candidate.b;
        if (p.get_name() == "allocation.q") m = &candidate.q;
        if (p.get_name() == "allocation.r") m = &candidate.r;
        if (!m) {
          if (p.get_name().rfind("allocation.", 0) == 0 || p.get_name().rfind("priority.", 0) == 0 ||
            p.get_name().rfind("scheduling.", 0) == 0 || p.get_name().rfind("capacity.", 0) == 0 ||
            p.get_name().rfind("delivery.", 0) == 0) {
            result.successful = false; result.reason = "only A/B/Q/R support runtime updates; restart for other settings"; return result;
          }
          continue;
        }
        if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY || p.as_double_array().size() != 4) {
          result.successful = false; result.reason = "matrix requires four doubles"; return result;
        }
        for (int j = 0; j < 4; ++j) (*m)(j / 2, j % 2) = p.as_double_array()[j];
        changed = true;
      }
      if (changed) {
        result.successful = allocator_.update(candidate, result.reason);
        if (result.successful) matrix_update_time_ = now().seconds();
        else RCLCPP_WARN(get_logger(), "Retaining controller: %s", result.reason.c_str());
      }
      return result;
    });

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
    backlog_publisher_ = create_publisher<surf_multirobot_msgs::msg::CompressedVoxelDelta>(
      backlog_topic_, surf::comms::sync_qos());
    metrics_publisher_ = create_publisher<surf_multirobot_msgs::msg::PipelineMetrics>(
      metrics_topic_, rclcpp::QoS(10));
    realtime_ack_metrics_publisher_ =
      create_publisher<surf_multirobot_msgs::msg::RealtimeAckMetrics>(
      realtime_ack_metrics_topic_, rclcpp::QoS(50));
    allocation_publisher_ = create_publisher<surf_multirobot_msgs::msg::AllocationMetrics>(
      "/" + robot_name_ + "/comm/allocation_metrics", rclcpp::QoS(100));
    realtime_publish_timer_ = create_wall_timer(std::chrono::duration<double>(control_dt_),
      [this]() {control_step();});
    ack_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions ack_options;
    ack_options.callback_group = ack_callback_group_;
    realtime_ack_subscription_ = create_subscription<surf_multirobot_msgs::msg::RealtimeAck>(
      realtime_ack_topic_, rclcpp::QoS(10).best_effort().durability_volatile(),
      std::bind(&DroneScanSender::realtime_ack_callback, this, std::placeholders::_1), ack_options);
    backlog_ack_subscription_ = create_subscription<surf_multirobot_msgs::msg::SyncAck>(
      backlog_ack_topic_, rclcpp::QoS(10).reliable().transient_local(),
      std::bind(&DroneScanSender::backlog_ack_callback, this, std::placeholders::_1), ack_options);
    backlog_request_subscription_ = create_subscription<surf_multirobot_msgs::msg::SyncRequest>(
      backlog_request_topic_, rclcpp::QoS(10).reliable().transient_local(),
      std::bind(&DroneScanSender::backlog_request_callback, this, std::placeholders::_1));

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
    {
      // Priority relevance consumes peer pose independently of mask filtering.
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

    worker_ = std::thread(&DroneScanSender::worker_loop, this);
    RCLCPP_INFO(get_logger(),
      "%s communication sender: %s -> [%s, %s], resolution %.2fm",
      robot_name_.c_str(), input_topic_.c_str(), realtime_topic_.c_str(), backlog_topic_.c_str(),
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
  struct DeliveryAttempt {double sent; int stream; std::vector<Coord> coordinates;};
  std::unordered_map<uint64_t, DeliveryAttempt> attempts_;
  void delivery_metric(uint64_t version, const DeliveryAttempt & attempt, bool acknowledged, double lock_wait_ms = 0) {
    surf_multirobot_msgs::msg::RealtimeAckMetrics m;
    m.header.stamp = now(); m.map_epoch = map_epoch_; m.version = version;
    m.traffic_class = attempt.stream + 1; m.acknowledged = acknowledged;
    m.timeout_ms = realtime_ack_timeout_seconds_ * 1000;
    m.ack_lock_wait_ms = lock_wait_ms;
    if (acknowledged) m.update_completion_rtt_ms = m.final_chunk_rtt_ms =
      (steady_seconds() - attempt.sent) * 1000;
    realtime_ack_metrics_publisher_->publish(m);
  }
  void acknowledge_packet(uint64_t version, int stream, double lock_wait_ms) {
    auto it = attempts_.find(version);
    if (it == attempts_.end() || it->second.stream != stream) return;
    debt_.ack(version, it->second.coordinates); delivery_metric(version, it->second, true, lock_wait_ms); attempts_.erase(it);
  }
  void realtime_ack_callback(const surf_multirobot_msgs::msg::RealtimeAck::SharedPtr ack)
  {
    const auto received = steady_seconds();
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (ack->map_epoch == map_epoch_) acknowledge_packet(ack->version, 0, (steady_seconds()-received)*1000);
  }
  void backlog_ack_callback(const surf_multirobot_msgs::msg::SyncAck::SharedPtr ack)
  {
    const auto received = steady_seconds();
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (ack->source_id == robot_name_ && ack->map_epoch == map_epoch_)
      acknowledge_packet(ack->version, 1, (steady_seconds()-received)*1000);
  }
  void backlog_request_callback(const surf_multirobot_msgs::msg::SyncRequest::SharedPtr request)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if ((request->source_id.empty() || request->source_id == robot_name_) &&
      (request->map_epoch == 0 || request->map_epoch == map_epoch_)) {
      debt_.request_recovery();
    }
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
    if (metrics->link_name != "halow") return;
    std::lock_guard<std::mutex> lock(state_mutex_);
    capacity_received_ = steady_seconds();
    link_metrics_ = *metrics;
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
    const std::unique_ptr<HumanoidMask> humanoid_mask = humanoid_mask_at(cloud.header.stamp);

    uint32_t raw_points = cloud.width * cloud.height;
    uint32_t valid_points = 0;
    std::unordered_set<Coord, CoordHash> current;
    std::unordered_map<Coord, tf2::Vector3, CoordHash> endpoints;
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
        const auto coord = quantize(mapped.x(), mapped.y(), mapped.z(), resolution_);
        current.insert(coord);
        endpoints.try_emplace(coord, mapped);
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

    std::unique_lock<std::mutex> state_lock(state_mutex_);
    if (have_scan_stamp_ && stamp_to_nanoseconds(cloud.header.stamp) <= last_scan_stamp_) {++debt_.stale_observations; return;}
    last_scan_stamp_ = stamp_to_nanoseconds(cloud.header.stamp);
    have_scan_stamp_ = true;
    ++version_;
    const uint64_t scan_time_ns = stamp_to_nanoseconds(cloud.header.stamp);
    const uint8_t mode = surf_multirobot_msgs::msg::VoxelDelta::MODE_VOXEL_DELTAS;
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
          debt_.exclude(coord);
          cells_.erase(coord);
          dynamic_expiry_.cancel(coord);
          tombstones_.erase(coord);
        }
      }
      for (const auto & coord : tombstones_.in_box(bounds.first, bounds.second)) {
        if (humanoid_mask->contains(coord, resolution_)) {debt_.exclude(coord); tombstones_.erase(coord);}
      }
    }

    std::size_t occupancy_work = 0;
    for (const auto & coord : current) {
      // Keep prior-overlap telemetry comparable with the full filtered scan;
      // the communication horizon controls admission, not this denominator.
      const bool represented_by_prior = static_snapshot->count(coord) != 0;
      if (represented_by_prior) ++static_prior_voxels;
      if ((endpoints.at(coord) - translation).length2() > mapping_radius_ * mapping_radius_) continue;
      if (++occupancy_work % 256 == 0) {
        state_lock.unlock();
        std::this_thread::yield();
        state_lock.lock();
      }
      tombstones_.erase(coord);
      auto & cell = cells_[coord];
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

      const bool new_information = cell.last_classified_version == 0U ||
        cell.last_classified_static != cell.static_known;
      // A prior voxel that was cleared remotely must be explicitly restored;
      // periodic whole-map snapshots no longer repair this case for us.
      const bool receiver_has_overlay = debt_.entries.find(coord) != debt_.entries.end();
      const bool send = (!represented_by_prior || receiver_has_overlay) && new_information;
      if (send) {
        geometry_msgs::msg::Point scan_origin;
        scan_origin.x = translation.x(); scan_origin.y = translation.y(); scan_origin.z = translation.z();
        append_record(realtime, coord, cell.static_known ?
          surf_multirobot_msgs::msg::VoxelDelta::STATE_OCCUPIED_STATIC :
          surf_multirobot_msgs::msg::VoxelDelta::STATE_OCCUPIED_DYNAMIC,
          cell.last_observation_time_ns, 2, scan_origin);
        cell.last_classified_version = version_;
        cell.last_classified_static = cell.static_known;
      }
    }
    const auto occupancy_selection_complete = std::chrono::steady_clock::now();
    // cells_, tombstones_ and dynamic_expiry_ belong exclusively to the scan worker.
    // Ray tracing does not touch the shared debt ledger. Let control and ACKs run
    // while this expensive local map work finishes; commit its observations below.
    state_lock.unlock();

    // Rotate a stable endpoint ordering, so the global work budget does not
    // repeatedly select the same prefix of an unordered point cloud.
    std::vector<Coord> ray_endpoints(current.begin(), current.end());
    std::sort(ray_endpoints.begin(), ray_endpoints.end(), [](const Coord & a, const Coord & b) {
      return std::tie(a.x, a.y, a.z) < std::tie(b.x, b.y, b.z);
    });
    std::unordered_set<Coord, CoordHash> observed_free;
    std::unordered_map<Coord, tf2::Vector3, CoordHash> selected_ray_targets;
    std::size_t ray_work = 0, sampled_rays = 0;
    while (!ray_endpoints.empty() && sampled_rays < ray_endpoints.size() &&
      sampled_rays < static_cast<std::size_t>(maximum_clear_rays_) && ray_work < maximum_ray_work_)
    {
      const Coord endpoint = ray_endpoints[ray_cursor_++ % ray_endpoints.size()];
      ++sampled_rays;
      bool safe_prefix = true;
      bool have_safe_target = false;
      Coord safe_target{};
      ray_work += visit_observed_ray(translation, endpoints.at(endpoint), resolution_,
        std::min(static_cast<std::size_t>(maximum_ray_voxels_), maximum_ray_work_ - ray_work),
        [&](const Coord & coord) {
          const tf2::Vector3 center((coord.x+.5)*resolution_, (coord.y+.5)*resolution_, (coord.z+.5)*resolution_);
          const double distance = (center - translation).length();
          if (distance > mapping_radius_) return false;
          // A measured endpoint, including another ray's endpoint, occludes free
          // evidence behind it. Never clear the peer-owned body or the sensor.
          if (current.count(coord) || (humanoid_mask && humanoid_mask->contains(coord, resolution_))) return false;
          const auto cleared = tombstones_.find(coord);
          const bool prior_cleared = cleared != tombstones_.end() &&
            cleared->second.state == surf_multirobot_msgs::msg::VoxelDelta::STATE_FREE;
          if (cells_.find(coord) != cells_.end() ||
            (static_snapshot->count(coord) && !prior_cleared)) safe_prefix = false;
          if (distance >= std::max(min_range_, self_radius_) && center.z() >= min_z_ && center.z() <= max_z_) {
            observed_free.insert(coord);
            if (safe_prefix && !prior_cleared) {safe_target = coord; have_safe_target = true;}
          }
          return true;
        });
      if (new_free_ray_stride_ && have_safe_target && new_free_ray_cursor_++ % new_free_ray_stride_ == 0)
        selected_ray_targets.try_emplace(safe_target, endpoints.at(endpoint));
    }
    uint32_t free_updates = 0, unknown_updates = 0;
    for (const auto & coord : observed_free) {
      const auto previous = tombstones_.find(coord);
      if (previous != tombstones_.end() && previous->second.state == surf_multirobot_msgs::msg::VoxelDelta::STATE_FREE) continue;
      // A shared-prior occupied cell also needs the configured miss evidence,
      // even if this sender has not hit that surface in the current run.
      if (cells_.find(coord) == cells_.end() && static_snapshot->count(coord))
        cells_[coord].static_known = true;
      auto found = cells_.find(coord);
      // Clearing an obstacle already known to either map is always delivered.
      // Novel open space is sampled to bound its network and recovery cost.
      const bool clears_occupied = found != cells_.end() || static_snapshot->count(coord) != 0;
      if (found != cells_.end()) {
        // Existing occupied evidence requires repeated misses; a previously
        // unknown cell may become free on its first measured ray traversal.
        if (++found->second.consecutive_misses < static_cast<uint32_t>(clear_min_misses_)) continue;
        cells_.erase(found);
        dynamic_expiry_.cancel(coord);
      }
      const auto old = tombstones_.find(coord);
      if (old != tombstones_.end() && old->second.state == surf_multirobot_msgs::msg::VoxelDelta::STATE_FREE) continue;
      const auto safe_endpoint = selected_ray_targets.find(coord);
      if (clears_occupied || safe_endpoint != selected_ray_targets.end()) {
        tombstones_[coord] = {version_, surf_multirobot_msgs::msg::VoxelDelta::STATE_FREE, scan_time_ns};
        geometry_msgs::msg::Point ray_origin, ray_endpoint;
        uint8_t ray_flag = 0;
        if (!clears_occupied && safe_endpoint != selected_ray_targets.end()) {
          ray_flag = 1;
          ray_origin.x = translation.x(); ray_origin.y = translation.y(); ray_origin.z = translation.z();
          ray_endpoint.x = safe_endpoint->second.x();
          ray_endpoint.y = safe_endpoint->second.y();
          ray_endpoint.z = safe_endpoint->second.z();
        }
        append_record(realtime, coord, surf_multirobot_msgs::msg::VoxelDelta::STATE_FREE,
          scan_time_ns, ray_flag, ray_origin, ray_endpoint);
        ++free_updates;
      }
    }

    // Lack of a return is not measured free space. Withdraw only this source's
    // expired dynamic observation; other sources' occupied evidence survives.
    for (const auto & coord : dynamic_expiry_.pop_due(version_)) {
      auto it = cells_.find(coord);
      if (it == cells_.end() || it->second.static_known) continue;
      tombstones_[coord] = {version_, surf_multirobot_msgs::msg::VoxelDelta::STATE_UNKNOWN, scan_time_ns};
      append_record(realtime, coord, surf_multirobot_msgs::msg::VoxelDelta::STATE_UNKNOWN, scan_time_ns);
      ++unknown_updates;
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

    state_lock.lock();
    // Publish metadata before exposing the first batch of debt to control.
    last_header_ = realtime.header;
    last_origin_ = realtime.sensor_origin;
    for (std::size_t i = 0; i < realtime.x.size(); ++i) {
      if (i && i % 256 == 0) {
        state_lock.unlock();
        std::this_thread::yield();
        state_lock.lock();
      }
      debt_.observe({realtime.x[i], realtime.y[i], realtime.z[i]}, realtime.state[i],
        realtime.observation_time_ns[i], steady_seconds(),
        information_priority(weights_[0], 0,
          humanoid_mask ? (tf2::Vector3((realtime.x[i]+.5)*resolution_,
            (realtime.y[i]+.5)*resolution_, (realtime.z[i]+.5)*resolution_) - humanoid_mask->center).length() :
            std::numeric_limits<double>::infinity(), realtime.state[i]),
        realtime.ray_flags[i], realtime.ray_origins[i], realtime.ray_endpoints[i]);
    }
    last_header_ = realtime.header;
    last_origin_ = realtime.sensor_origin;
    surf_multirobot_msgs::msg::PipelineMetrics metrics;
    metrics.header = realtime.header; metrics.transmit_stamp = now(); metrics.source_id = robot_name_;
    metrics.map_epoch = map_epoch_; metrics.version = version_; metrics.operating_mode = 4;
    {std::lock_guard<std::mutex> queue_lock(queue_mutex_);
      metrics.input_rate_hz = input_rate_hz_; metrics.stale_input_drops = stale_input_drops_;}
    metrics.raw_serialized_bytes = raw_serialized_bytes; metrics.raw_points = raw_points;
    metrics.valid_points = valid_points; metrics.unique_voxels = current.size();
    metrics.free_updates = free_updates;
    metrics.unknown_updates = unknown_updates;
    metrics.ray_cells_visited = ray_work;
    metrics.sampled_rays = sampled_rays;
    metrics.static_prior_voxels = static_prior_voxels; metrics.raw_data_bytes = cloud.data.size();
    metrics.point_step_bytes = cloud.point_step; metrics.queue_wait_ms = queue_wait_ms;
    metrics.transform_lookup_ms = transform_lookup_ms; metrics.point_preprocessing_ms = point_preprocessing_ms;
    metrics.occupancy_selection_ms = occupancy_selection_ms; metrics.clearing_ms = clearing_ms;
    metrics.raw_serialization_ms = raw_serialization_ms;
    metrics.processing_latency_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - processing_start).count();
    metrics_publisher_->publish(metrics);
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

  void control_step()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const double start = steady_seconds();
    const double actual_dt = last_control_time_ ? start - last_control_time_ : control_dt_;
    last_control_time_ = start;
    // Bound catch-up after executor stalls. Credits represent saved allocation,
    // with at most two nominal intervals of credit (minimum one packet).
    // Both classes debit the same bounded credit; no unlimited catch-up.
    const double elapsed = std::min(actual_dt, control_dt_ * 2);
    for (auto it = attempts_.begin(); it != attempts_.end();) {
      if (start - it->second.sent >= realtime_ack_timeout_seconds_) {
        delivery_metric(it->first, it->second, false);
        debt_.timeout_packet(it->first, it->second.coordinates);
        it = attempts_.erase(it);
      } else ++it;
    }
    tf2::Vector3 peer(0, 0, 0);
    bool peer_valid = false;
    {
      std::lock_guard<std::mutex> pose_lock(humanoid_odometry_mutex_);
      if (!humanoid_odometry_history_.empty()) {
        const auto & p = humanoid_odometry_history_.back()->pose.pose.position;
        const auto & pose = *humanoid_odometry_history_.back();
        peer_valid = pose.header.frame_id == map_frame_ &&
          std::abs((now() - rclcpp::Time(pose.header.stamp)).seconds()) <= humanoid_mask_max_age_ms_ / 1000;
        peer = tf2::Vector3(p.x, p.y, p.z);
      }
    }
    const double debt_cycle_start = steady_seconds();
    const auto recovered = debt_.recovery_slice(start, maximum_control_entries_ / 4);
    auto debt_cycle = debt_.bounded_cycle(
      start, now().seconds(), defer_seconds_, realtime_ack_timeout_seconds_,
      weights_, peer, resolution_, peer_valid, maximum_control_entries_ - recovered);
    const double debt_cycle_ms = (steady_seconds() - debt_cycle_start) * 1000;
    const auto & x = debt_cycle.debt;
    double capacity = 0;
    std::string method = "waiting_for_telemetry";
    if (capacity_received_ > 0) {
      capacity = start - capacity_received_ <= capacity_timeout_ && link_metrics_.usable_capacity_valid ?
        link_metrics_.usable_capacity_bytes_per_second : 0;
      method = start - capacity_received_ <= capacity_timeout_ ? link_metrics_.capacity_method : "stale_telemetry";
    }
    if (!std::isfinite(capacity) || capacity < 0) capacity = 0;
    const double controller_start = steady_seconds();
    const auto feedback = allocator_.allocate({x[0], x[1]}, capacity);
    auto serviceable_request = feedback.requested;
    for (int stream=0; stream<2; ++stream) {
      if (debt_cycle.candidates[stream].empty()) serviceable_request[stream] = 0;
    }
    // A class with no eligible records cannot spend its allocation. Project
    // with that actuator disabled instead of stranding capacity behind ACKs
    // or behind unvisited entries in the bounded scoring queue.
    auto allocation = allocator_.project(serviceable_request, capacity);
    const double controller_ms = (steady_seconds() - controller_start)*1000;
    if (capacity == 0) {credits_ = {}; shared_credit_ = 0;}
    shared_credit_ = std::min(std::max(double(maximum_packet_bytes_), capacity * control_dt_ * 2),
      shared_credit_ + capacity * elapsed);
    surf_multirobot_msgs::msg::AllocationMetrics m;
    m.header.stamp = now(); m.source_id = robot_name_; m.map_epoch = map_epoch_;
    m.controller_ms = controller_ms;
    // These legacy fields are zero because advance, scoring, metrics, and
    // candidate collection now share one traversal measured by debt_cycle_ms.
    m.debt_advance_ms = 0; m.debt_score_ms = 0; m.metrics_scan_ms = 0;
    m.debt_cycle_ms = debt_cycle_ms; m.priority_ms = debt_cycle_ms;
    m.step = ++control_sequence_; m.nominal_dt = control_dt_; m.actual_dt = actual_dt;
    m.debt = x; m.previous_debt = previous_debt_; previous_debt_ = x;
    m.usable_capacity = capacity; m.capacity_method = method;
    m.configuration_revision = allocator_.revision(); m.matrix_update_time = matrix_update_time_;
    m.rejected_matrix_updates = allocator_.rejected_updates(); m.projection_scale = allocation.scale;
    m.excluded_debt = debt_.excluded;
    m.disturbance = debt_.disturbance; m.timed_out_debt = debt_.timed_out;
    m.generated_debt = debt_.generated; m.acknowledged_debt = debt_.acknowledged;
    m.superseded_debt = debt_.superseded; m.reclassified_debt = debt_.reclassified;
    m.superseded_count = debt_.superseded_count; m.stale_observations = debt_.stale_observations;
    for (int j = 0; j < 4; ++j) {
      m.a[j] = allocator_.model().a(j / 2, j % 2); m.b[j] = allocator_.model().b(j / 2, j % 2);
      m.q[j] = allocator_.model().q(j / 2, j % 2); m.r[j] = allocator_.model().r(j / 2, j % 2);
      m.k[j] = allocator_.gain()(j / 2, j % 2);
    }
    m.stale_pending_discarded = debt_.stale_pending_discarded;
    m.retained_count = debt_.entries.size();
    m.pending_count = debt_cycle.pending_count;
    m.inflight_count = debt_cycle.inflight_count;
    m.pending_age_bucket_count = debt_cycle.pending_age_bucket_count;
    m.max_priority_remaining = debt_cycle.max_priority_remaining;
    m.ack_lag_samples = debt_cycle.ack_lag_samples;
    m.oldest_pending_age = debt_cycle.oldest_pending_age;
    m.oldest_observation_age_s = debt_cycle.oldest_observation_age_s;
    m.newest_observation_age_s = debt_cycle.newest_observation_age_s;
    m.max_ack_observation_lag_s = debt_cycle.max_ack_observation_lag_s;
    std::array<double, 2> selected_age_total{};
    for (int stream = 0; stream < 2; ++stream) {
      m.mean_priority_remaining[stream] = m.pending_count[stream] ? x[stream]/m.pending_count[stream] : 0;
      m.requested_rate[stream] = feedback.requested[stream];
      m.serviceable_requested_rate[stream] = allocation.requested[stream];
      m.eligible_candidate_count[stream] = debt_cycle.candidates[stream].size();
      m.allocated_rate[stream] = allocation.allocated[stream];
      m.target_bytes[stream] = allocation.allocated[stream] * control_dt_;
      credits_[stream] = std::min(std::max(double(maximum_packet_bytes_),
        allocation.allocated[stream] * control_dt_ * 2),
        credits_[stream] + allocation.allocated[stream] * elapsed);
      if (m.pending_count[stream] == 0) {credits_[stream] = 0; continue;}
      // Wall timers commonly fire a fraction early. Without tolerance, a 5 Hz
      // stream on a 5 Hz controller skips every other opportunity and halves
      // useful throughput. Half a controller tick cannot create an extra
      // scheduling opportunity between controller callbacks.
      if (start - last_scheduled_[stream] + control_dt_ * .5 < 1.0 / schedule_hz_[stream]) continue;
      last_scheduled_[stream] = start;
      const double ordering_start = steady_seconds();
      const auto reserve = static_cast<std::size_t>(std::ceil(
        maximum_packet_bytes_ * 0.25 * starvation_fraction_));
      debt_.score_spatial_change(debt_cycle.candidates[stream],
        cluster_scheduling_enabled_ ? spatial_change_weight_ : 0.0, cluster_neighbour_radius_);
      auto candidates = InformationDebt::rank_candidates(
        std::move(debt_cycle.candidates[stream]), maximum_control_entries_,
        start, starvation_age_seconds_, reserve);
      InformationDebt::ClusterSelection cluster;
      if (stream == 1 && cluster_scheduling_enabled_ && !candidates.empty()) {
        cluster = debt_.connected_candidates(
          stream, candidates, maximum_packet_bytes_ * 2, maximum_cluster_entries_,
          cluster_neighbour_radius_);
        // BACKLOG grows connected components from the spatially ranked seeds.
        // DELTA uses per-voxel spatial priority without waiting for completion.
        candidates = std::move(cluster.entries);
        m.focus_cluster_valid = cluster.valid;
        m.focus_cluster_seed = {cluster.seed.x, cluster.seed.y, cluster.seed.z};
        m.focus_cluster_visited = cluster.visited;
        m.focus_cluster_candidates = cluster.pending;
        m.selected_cluster_count = cluster.components;
        m.focus_cluster_truncated = cluster.truncated;
        m.completed_cluster_deliveries = cluster.completed;
      }
      if (candidates.size() > maximum_packet_bytes_ * 2) candidates.resize(maximum_packet_bytes_ * 2);
      m.selection_ms[stream] = (steady_seconds()-ordering_start)*1000;
      m.priority_ms += m.selection_ms[stream];
      if (candidates.empty()) continue;
      std::size_t offset = 0;
      // Bound callback work while allowing small packets to consume accrued credit.
      for (int packet = 0; packet < 16 && offset < candidates.size(); ++packet) {
        if (std::min(credits_[stream], shared_credit_) <= 0) break;
        surf_multirobot_msgs::msg::VoxelDelta delta;
        delta.header = last_header_; delta.source_id = robot_name_; delta.map_epoch = map_epoch_;
        delta.version = packet_version_ + 1; delta.operating_mode = 4;
        delta.resolution = resolution_; delta.sensor_origin = last_origin_;
        delta.chunk_count = 1; // Independently ACKed packets; no cross-packet assembly dependency.
        const double budget = std::min({credits_[stream], shared_credit_, double(maximum_packet_bytes_)});
        surf_multirobot_msgs::msg::CompressedVoxelDelta best;
        std::size_t count = 0, bytes = 0;
        // Bounded prefix search retains only explicitly encoded fitting candidates.
        // Nonmonotone compression may miss a larger fit, never exceed the budget.
        std::size_t trial = std::min<std::size_t>(candidates.size() - offset, maximum_packet_bytes_);
        std::size_t low = 0, high = trial;
        const double encoding_start = steady_seconds();
        for (int encoding_trial = 0; trial > low && encoding_trial < 12; ++encoding_trial) {
          delta.x.clear(); delta.y.clear(); delta.z.clear(); delta.state.clear(); delta.observation_time_ns.clear();
          delta.ray_flags.clear(); delta.ray_origins.clear(); delta.ray_endpoints.clear();
          for (std::size_t j = 0; j < trial; ++j)
            append_record(delta, candidates[offset+j]->coord, candidates[offset+j]->state,
              candidates[offset+j]->stamp, candidates[offset+j]->ray_flag,
              candidates[offset+j]->ray_origin, candidates[offset+j]->ray_endpoint);
          surf_multirobot_msgs::msg::CompressedVoxelDelta wire;
          const auto result = surf::comms::encode_delta(delta, wire, compression_level_);
          if (!result.ok) {RCLCPP_ERROR(get_logger(), "%s", result.error.c_str()); break;}
          wire.traffic_class = stream == 0 ? 1 : 2; wire.transmit_stamp = now();
          rclcpp::Serialization<surf_multirobot_msgs::msg::CompressedVoxelDelta> serializer;
          rclcpp::SerializedMessage serialized; serializer.serialize_message(&wire, &serialized);
          if (serialized.size() <= budget && serialized.size() <= maximum_packet_bytes_) {
            best = std::move(wire); count = trial; bytes = serialized.size(); low = trial;
          } else {
            high = trial - 1;
          }
          if (low >= high) break;
          trial = low + (high - low + 1) / 2;
        }
        m.encoding_ms += (steady_seconds() - encoding_start)*1000;
        if (!count) break;
        ++packet_version_;
        const double sent_at = steady_seconds();
        best.transmit_stamp = now();
        auto & attempt = attempts_[packet_version_];
        attempt.sent = sent_at; attempt.stream = stream; attempt.coordinates.reserve(count);
        for (std::size_t j = 0; j < count; ++j) {
          auto * candidate = candidates[offset+j];
          debt_.mark_sent(*candidate, packet_version_, sent_at);
          attempt.coordinates.push_back(candidate->coord);
          m.sent_priority[stream] += candidate->priority;
          m.max_priority_sent[stream] = std::max(m.max_priority_sent[stream], candidate->priority);
          const double pending_age = std::max(0.0, sent_at - candidate->created);
          selected_age_total[stream] += pending_age;
          m.max_selected_pending_age_s[stream] = std::max(
            m.max_selected_pending_age_s[stream], pending_age);
          if (pending_age >= starvation_age_seconds_) ++m.old_pending_selected_count[stream];
          if (candidate->state == surf_multirobot_msgs::msg::VoxelDelta::STATE_UNKNOWN)
            ++m.selected_unknown_count[stream];
          if (candidate->state >= 1 && candidate->state <= 4) {
            ++m.selected_state_count[stream * 4 + candidate->state - 1];
          }
        }
        credits_[stream] -= bytes; shared_credit_ -= bytes;
        m.selected_count[stream] += count; m.wire_bytes[stream] += bytes;
        m.uncompressed_bytes[stream] += best.uncompressed_bytes;
        m.compressed_bytes[stream] += best.payload.size();
        if (m.codec[stream].empty()) m.codec[stream] = best.codec;
        else if (m.codec[stream] != best.codec) m.codec[stream] = "mixed";
        (stream == 0 ? realtime_publisher_ : backlog_publisher_)->publish(best);
        offset += count;
      }
      if (m.selected_count[stream]) {
        m.mean_selected_pending_age_s[stream] =
          selected_age_total[stream] / m.selected_count[stream];
      }
    }
    m.scored_entries = debt_cycle.scored_entries;
    m.sampled_age_metrics = debt_cycle.sampled_metrics;
    m.recovery_entries_processed = recovered;
    m.recovery_entries_remaining = debt_.recovery_remaining();
    const auto region = debt_.region_progress();
    m.focus_region_valid = region.valid;
    m.focus_region = {region.coord.x, region.coord.y, region.coord.z};
    m.focus_region_revision = region.revision;
    m.focus_region_known = region.known;
    m.focus_region_pending = region.pending;
    m.completed_region_deliveries = region.completed;
    m.computation_ms = (steady_seconds() - start) * 1000;
    allocation_publisher_->publish(m);
  }

  uint64_t control_sequence_{0};
  std::string robot_name_;
  std::string map_frame_;
  std::string input_topic_;
  std::string static_map_topic_;
  std::string realtime_topic_;
  std::string realtime_ack_topic_;
  std::string realtime_ack_metrics_topic_;
  std::string backlog_topic_;
  std::string backlog_ack_topic_;
  std::string backlog_request_topic_;
  std::string link_metrics_topic_;
  std::string metrics_topic_;
  std::string transform_source_;
  std::string odometry_topic_;
  std::string odometry_parent_frame_;
  std::string base_frame_;
  double pose_max_age_ms_{50.0};
  double transform_timeout_ms_{500.0};
  double resolution_{0.05};
  double mapping_radius_{10.0};
  std::size_t maximum_ray_work_{32768}, ray_cursor_{0}, maximum_control_entries_{4096};
  std::size_t new_free_ray_stride_{1}, new_free_ray_cursor_{0};
  std::size_t maximum_cluster_entries_{8192};
  int cluster_neighbour_radius_{2};
  bool cluster_scheduling_enabled_{true};
  double spatial_change_weight_{1.0};
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
  int dynamic_retention_scans_{20};
  int maximum_ray_voxels_{1200};
  int maximum_clear_rays_{256};
  int compression_level_{1};
  std::size_t maximum_packet_bytes_{1200};
  double realtime_ack_timeout_seconds_{4};
  uint64_t map_epoch_{0U};
  uint64_t version_{0U};
  uint64_t cumulative_raw_bytes_{0U};
  uint64_t cumulative_wire_bytes_{0U};
  uint64_t stale_input_drops_{0U};
  double input_rate_hz_{0.0};
  std::chrono::steady_clock::time_point last_input_time_{};

  InformationAllocationController allocator_;
  InformationDebt debt_;
  std::array<PriorityWeights, 2> weights_;
  std::mutex state_mutex_;
  double control_dt_{.1}, defer_seconds_{1.5}, capacity_timeout_{3};
  double capacity_received_{0}, last_control_time_{0}, matrix_update_time_{0};
  std::array<double, 2> credits_{}, last_scheduled_{};
  std::array<double, 2> schedule_hz_{10,10};
  double starvation_fraction_{0.1}, starvation_age_seconds_{30};
  std::array<double, 2> previous_debt_{};
  double shared_credit_{0};
  uint64_t packet_version_{0}, last_scan_stamp_{0};
  bool have_scan_stamp_{false};
  std_msgs::msg::Header last_header_;
  geometry_msgs::msg::Point last_origin_;
  surf_multirobot_msgs::msg::LinkMetrics link_metrics_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr matrix_callback_;
  rclcpp::Publisher<surf_multirobot_msgs::msg::AllocationMetrics>::SharedPtr allocation_publisher_;
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

  std::atomic<bool> stop_{false};
  std::thread worker_;
  std::mutex queue_mutex_;
  std::condition_variable queue_condition_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;
  std::chrono::steady_clock::time_point latest_cloud_received_{};
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
    backlog_ack_subscription_;
  rclcpp::Subscription<surf_multirobot_msgs::msg::RealtimeAck>::SharedPtr
    realtime_ack_subscription_;
  rclcpp::Subscription<surf_multirobot_msgs::msg::SyncRequest>::SharedPtr
    backlog_request_subscription_;
  rclcpp::Publisher<surf_multirobot_msgs::msg::CompressedVoxelDelta>::SharedPtr
    realtime_publisher_;
  rclcpp::Publisher<surf_multirobot_msgs::msg::CompressedVoxelDelta>::SharedPtr
    backlog_publisher_;
  rclcpp::Publisher<surf_multirobot_msgs::msg::PipelineMetrics>::SharedPtr metrics_publisher_;
  rclcpp::Publisher<surf_multirobot_msgs::msg::RealtimeAckMetrics>::SharedPtr
    realtime_ack_metrics_publisher_;
  rclcpp::CallbackGroup::SharedPtr ack_callback_group_;
  rclcpp::TimerBase::SharedPtr realtime_publish_timer_;
};

}  // namespace surf_drone

#ifndef SURF_NODE_TEST
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto sender = std::make_shared<surf_drone::DroneScanSender>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(sender);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

#endif
