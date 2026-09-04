#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "px4_msgs/msg/sensor_combined.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

namespace
{

template<typename MessageT, typename = void>
struct has_heading_var : std::false_type {};

template<typename MessageT>
struct has_heading_var<
  MessageT, std::void_t<decltype(std::declval<MessageT>().heading_var)>>
  : std::true_type {};

template<typename MessageT>
double heading_variance_or_unknown(const MessageT & message)
{
  if constexpr (has_heading_var<MessageT>::value) {
    return message.heading_var;
  }
  // ROS covariance values of zero mean that the covariance is unknown.
  return 0.0;
}

geometry_msgs::msg::Quaternion to_msg(const tf2::Quaternion & q)
{
  geometry_msgs::msg::Quaternion result;
  result.x = q.x();
  result.y = q.y();
  result.z = q.z();
  result.w = q.w();
  return result;
}

tf2::Quaternion normalized_px4_quaternion(const std::array<float, 4> & q)
{
  // PX4 uses Hamilton order [w, x, y, z].
  tf2::Quaternion result(q[1], q[2], q[3], q[0]);
  if (result.length2() > std::numeric_limits<double>::epsilon()) {
    result.normalize();
  } else {
    result.setValue(0.0, 0.0, 0.0, 1.0);
  }
  return result;
}

bool valid_quaternion(const std::array<float, 4> & q)
{
  return std::all_of(q.begin(), q.end(), [](float value) {return std::isfinite(value);}) &&
         (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]) >
         std::numeric_limits<float>::epsilon();
}

tf2::Quaternion ned_frd_to_enu_flu(const std::array<float, 4> & q)
{
  const double s = std::sqrt(0.5);
  const tf2::Quaternion enu_from_ned(s, s, 0.0, 0.0);
  const tf2::Quaternion frd_from_flu(1.0, 0.0, 0.0, 0.0);
  tf2::Quaternion result = enu_from_ned * normalized_px4_quaternion(q) * frd_from_flu;
  result.normalize();
  return result;
}

tf2::Quaternion frd_frd_to_flu_flu(const std::array<float, 4> & q)
{
  const tf2::Quaternion flu_from_frd(1.0, 0.0, 0.0, 0.0);
  tf2::Quaternion result = flu_from_frd * normalized_px4_quaternion(q) * flu_from_frd;
  result.normalize();
  return result;
}

std::array<double, 3> ned_to_enu(const std::array<float, 3> & value)
{
  return {value[1], value[0], -value[2]};
}

std::array<double, 3> frd_to_flu(const std::array<float, 3> & value)
{
  return {value[0], -value[1], -value[2]};
}

void set_diagonal(
  std::array<double, 36> & covariance, const std::array<float, 3> & variance,
  bool swap_xy)
{
  covariance.fill(0.0);
  covariance[0] = swap_xy ? variance[1] : variance[0];
  covariance[7] = swap_xy ? variance[0] : variance[1];
  covariance[14] = variance[2];
}

diagnostic_msgs::msg::KeyValue key_value(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}

}  // namespace

class Px4StateBridge : public rclcpp::Node
{
public:
  Px4StateBridge()
  : Node("px4_state_bridge")
  {
    const auto prefix = declare_parameter<std::string>("px4_topic_prefix", "/Drone1/fmu/out");
    robot_name_ = declare_parameter<std::string>("robot_name", "drone");
    imu_frame_ = declare_parameter<std::string>("imu_frame", "drone/px4_imu");
    odom_frame_ = declare_parameter<std::string>("odometry_frame", "drone/px4_odom");
    body_frame_ = declare_parameter<std::string>("body_frame", "drone/body");

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("px4/imu", rclcpp::SensorDataQoS());
    attitude_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "px4/attitude", rclcpp::SensorDataQoS());
    odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "px4/odometry", rclcpp::SensorDataQoS());
    local_odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "px4/local_odometry", rclcpp::SensorDataQoS());
    status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("px4/status", 10);

    const auto qos = rclcpp::SensorDataQoS().keep_last(20);
    sensor_sub_ = create_subscription<px4_msgs::msg::SensorCombined>(
      prefix + "/sensor_combined", qos,
      std::bind(&Px4StateBridge::sensor_callback, this, std::placeholders::_1));
    attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
      prefix + "/vehicle_attitude", qos,
      std::bind(&Px4StateBridge::attitude_callback, this, std::placeholders::_1));
    odometry_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
      prefix + "/vehicle_odometry", qos,
      std::bind(&Px4StateBridge::odometry_callback, this, std::placeholders::_1));
    local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      prefix + "/vehicle_local_position", qos,
      std::bind(&Px4StateBridge::local_position_callback, this, std::placeholders::_1));
    status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      prefix + "/vehicle_status", qos,
      std::bind(&Px4StateBridge::status_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "PX4 state bridge reading %s/* and publishing ROS ENU/FLU state",
      prefix.c_str());
  }

private:
  rclcpp::Time stamp(uint64_t timestamp_us) const
  {
    // uXRCE-DDS synchronizes PX4's clock to the agent clock. A zero timestamp
    // is retained only during early boot, where receipt time is the best datum.
    return timestamp_us == 0U ? now() : rclcpp::Time(
      static_cast<int64_t>(timestamp_us) * 1000, get_clock()->get_clock_type());
  }

  void sensor_callback(const px4_msgs::msg::SensorCombined::SharedPtr message)
  {
    sensor_msgs::msg::Imu output;
    output.header.stamp = stamp(message->timestamp);
    output.header.frame_id = imu_frame_;
    const auto acceleration = frd_to_flu(message->accelerometer_m_s2);
    const auto angular_velocity = frd_to_flu(message->gyro_rad);
    output.linear_acceleration.x = acceleration[0];
    output.linear_acceleration.y = acceleration[1];
    output.linear_acceleration.z = acceleration[2];
    output.angular_velocity.x = angular_velocity[0];
    output.angular_velocity.y = angular_velocity[1];
    output.angular_velocity.z = angular_velocity[2];
    output.orientation_covariance[0] = -1.0;  // SensorCombined has no attitude.
    imu_pub_->publish(output);
  }

  void attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr message)
  {
    if (!valid_quaternion(message->q)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Ignoring invalid PX4 attitude quaternion");
      return;
    }
    latest_attitude_ = ned_frd_to_enu_flu(message->q);
    have_attitude_ = true;
    geometry_msgs::msg::PoseStamped output;
    output.header.stamp = stamp(message->timestamp_sample);
    output.header.frame_id = odom_frame_;
    output.pose.orientation = to_msg(latest_attitude_);
    attitude_pub_->publish(output);
  }

  void odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr message)
  {
    if (!valid_quaternion(message->q) ||
      (message->pose_frame != px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED &&
      message->pose_frame != px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Ignoring PX4 odometry with invalid pose frame");
      return;
    }
    nav_msgs::msg::Odometry output;
    output.header.stamp = stamp(message->timestamp_sample);
    output.header.frame_id = odom_frame_;
    output.child_frame_id = body_frame_;

    const bool ned_pose =
      message->pose_frame == px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
    const auto position = ned_pose ? ned_to_enu(message->position) : frd_to_flu(message->position);
    const tf2::Quaternion orientation = ned_pose ?
      ned_frd_to_enu_flu(message->q) : frd_frd_to_flu_flu(message->q);
    output.pose.pose.position.x = position[0];
    output.pose.pose.position.y = position[1];
    output.pose.pose.position.z = position[2];
    output.pose.pose.orientation = to_msg(orientation);
    set_diagonal(output.pose.covariance, message->position_variance, ned_pose);
    output.pose.covariance[21] = message->orientation_variance[0];
    output.pose.covariance[28] = message->orientation_variance[1];
    output.pose.covariance[35] = message->orientation_variance[2];

    std::array<double, 3> body_velocity{};
    if (message->velocity_frame == px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD) {
      body_velocity = frd_to_flu(message->velocity);
    } else {
      std::array<double, 3> world_velocity{};
      if (message->velocity_frame == px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED) {
        world_velocity = ned_to_enu(message->velocity);
      } else if (
        message->velocity_frame == px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_FRD)
      {
        world_velocity = frd_to_flu(message->velocity);
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "Ignoring PX4 odometry with invalid velocity frame");
        return;
      }
      const tf2::Vector3 world(world_velocity[0], world_velocity[1], world_velocity[2]);
      const tf2::Vector3 body = tf2::quatRotate(orientation.inverse(), world);
      body_velocity = {body.x(), body.y(), body.z()};
    }
    const auto angular_velocity = frd_to_flu(message->angular_velocity);
    output.twist.twist.linear.x = body_velocity[0];
    output.twist.twist.linear.y = body_velocity[1];
    output.twist.twist.linear.z = body_velocity[2];
    output.twist.twist.angular.x = angular_velocity[0];
    output.twist.twist.angular.y = angular_velocity[1];
    output.twist.twist.angular.z = angular_velocity[2];
    set_diagonal(output.twist.covariance, message->velocity_variance, false);
    odometry_pub_->publish(output);
  }

  void local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr message)
  {
    nav_msgs::msg::Odometry output;
    output.header.stamp = stamp(message->timestamp_sample);
    output.header.frame_id = odom_frame_;
    output.child_frame_id = body_frame_;
    output.pose.pose.position.x = message->y;
    output.pose.pose.position.y = message->x;
    output.pose.pose.position.z = -message->z;
    output.pose.pose.orientation = to_msg(latest_attitude_);
    if (have_attitude_) {
      const tf2::Vector3 world_velocity(message->vy, message->vx, -message->vz);
      const tf2::Vector3 body_velocity = tf2::quatRotate(latest_attitude_.inverse(), world_velocity);
      output.twist.twist.linear.x = body_velocity.x();
      output.twist.twist.linear.y = body_velocity.y();
      output.twist.twist.linear.z = body_velocity.z();
    } else {
      output.twist.covariance[0] = -1.0;
    }
    if (!message->xy_valid) {
      output.pose.covariance[0] = output.pose.covariance[7] = -1.0;
    } else {
      output.pose.covariance[0] = output.pose.covariance[7] = message->eph * message->eph;
    }
    output.pose.covariance[14] = message->z_valid ? message->epv * message->epv : -1.0;
    output.pose.covariance[35] = heading_variance_or_unknown(*message);
    if (!have_attitude_) {
      output.pose.covariance[21] = -1.0;
    }
    local_odometry_pub_->publish(output);
  }

  void status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr message)
  {
    diagnostic_msgs::msg::DiagnosticArray output;
    output.header.stamp = stamp(message->timestamp);
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "PX4 flight controller";
    status.hardware_id = robot_name_ + ":Drone1";
    status.level = message->failsafe ?
      diagnostic_msgs::msg::DiagnosticStatus::ERROR :
      (message->pre_flight_checks_pass ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN);
    status.message = message->failsafe ? "failsafe" :
      (message->pre_flight_checks_pass ? "ready" : "pre-flight checks incomplete");
    status.values.push_back(key_value("arming_state", std::to_string(message->arming_state)));
    status.values.push_back(key_value("navigation_state", std::to_string(message->nav_state)));
    status.values.push_back(key_value("failsafe", message->failsafe ? "true" : "false"));
    status.values.push_back(key_value(
      "pre_flight_checks_pass", message->pre_flight_checks_pass ? "true" : "false"));
    output.status.push_back(status);
    status_pub_->publish(output);
  }

  std::string imu_frame_;
  std::string odom_frame_;
  std::string body_frame_;
  std::string robot_name_;
  bool have_attitude_{false};
  tf2::Quaternion latest_attitude_{0.0, 0.0, 0.0, 1.0};

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr attitude_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr local_odometry_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr sensor_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Px4StateBridge>());
  rclcpp::shutdown();
  return 0;
}
