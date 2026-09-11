#pragma once

#include "rclcpp/qos.hpp"

namespace surf::comms
{

inline rclcpp::QoS realtime_qos()
{
  // Local DDS hops carry every chunk; the inter-robot hop remains UDP.
  // A two-sample history can discard a six-chunk update during an executor stall.
  return rclcpp::QoS(rclcpp::KeepLast(256)).reliable().durability_volatile();
}

inline rclcpp::QoS sync_qos()
{
  // Compatibility name: BACKLOG uses application ACK/reselection. Volatile durability
  // prevents unbudgeted replay of obsolete DDS history after bridge reconnects.
  return rclcpp::QoS(rclcpp::KeepLast(256)).reliable().durability_volatile();
}

}  // namespace surf::comms
