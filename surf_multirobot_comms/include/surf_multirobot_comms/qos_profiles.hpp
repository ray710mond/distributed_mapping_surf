#pragma once

#include "rclcpp/qos.hpp"

namespace surf::comms
{

inline rclcpp::QoS realtime_qos()
{
  return rclcpp::QoS(rclcpp::KeepLast(2)).best_effort().durability_volatile();
}

inline rclcpp::QoS sync_qos()
{
  // Keep the latest full refresh available across bridge reconnects and node
  // startup ordering. A volatile receiver can see only refreshes published
  // after it subscribes, leaving it stuck while real-time deltas are ignored.
  return rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
}

}  // namespace surf::comms
