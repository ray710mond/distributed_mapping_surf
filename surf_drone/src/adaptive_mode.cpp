#include "surf_drone/adaptive_mode.hpp"

#include <algorithm>

#include "surf_multirobot_msgs/msg/voxel_delta.hpp"

namespace surf_drone
{

AdaptiveModeController::AdaptiveModeController()
: AdaptiveModeController(Config{})
{
}

AdaptiveModeController::AdaptiveModeController(Config config)
: config_(config)
{
}

uint8_t AdaptiveModeController::desired_mode(double bandwidth_mbps, bool moving_up) const
{
  const double margin = moving_up ? config_.up_hysteresis_mbps : 0.0;
  if (bandwidth_mbps >= config_.full_min_mbps + margin) {
    return surf_multirobot_msgs::msg::VoxelDelta::MODE_FULL;
  }
  if (bandwidth_mbps >= config_.delta_min_mbps + margin) {
    return surf_multirobot_msgs::msg::VoxelDelta::MODE_VOXEL_DELTAS;
  }
  if (bandwidth_mbps >= config_.dynamic_min_mbps + margin) {
    return surf_multirobot_msgs::msg::VoxelDelta::MODE_DYNAMIC_ONLY;
  }
  return surf_multirobot_msgs::msg::VoxelDelta::MODE_METADATA_ONLY;
}

uint8_t AdaptiveModeController::update(
  double now_seconds, double available_mbps, uint32_t queue_depth)
{
  if (!initialized_) {
    smoothed_mbps_ = std::max(0.0, available_mbps);
    initialized_ = true;
    last_change_ = now_seconds;
  } else {
    smoothed_mbps_ = config_.ewma_alpha * std::max(0.0, available_mbps) +
      (1.0 - config_.ewma_alpha) * smoothed_mbps_;
  }

  uint8_t desired = desired_mode(smoothed_mbps_, false);
  if (desired < mode_) {
    desired = desired_mode(smoothed_mbps_, true);
  }
  if (queue_depth >= config_.congested_queue_depth && desired <
    surf_multirobot_msgs::msg::VoxelDelta::MODE_METADATA_ONLY)
  {
    desired = static_cast<uint8_t>(desired + 1U);
    // Queue pressure may degrade the current mode, but never trigger an
    // upgrade merely because the bandwidth estimate is high.
    desired = std::max(desired, mode_);
  }

  if (desired == mode_) {
    candidate_mode_ = mode_;
    candidate_since_ = now_seconds;
    return mode_;
  }
  if (desired != candidate_mode_) {
    candidate_mode_ = desired;
    candidate_since_ = now_seconds;
    return mode_;
  }

  const bool moving_up = desired < mode_;
  const double hold = moving_up ? config_.up_hold_seconds : config_.down_hold_seconds;
  if ((now_seconds - candidate_since_) >= hold &&
    (now_seconds - last_change_) >= config_.minimum_dwell_seconds)
  {
    mode_ = desired;
    last_change_ = now_seconds;
    candidate_since_ = now_seconds;
  }
  return mode_;
}

}  // namespace surf_drone
