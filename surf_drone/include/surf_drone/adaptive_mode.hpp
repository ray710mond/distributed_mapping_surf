#pragma once

#include <cstdint>

namespace surf_drone
{

class AdaptiveModeController
{
public:
  struct Config
  {
    double full_min_mbps{12.0};
    double delta_min_mbps{3.0};
    double dynamic_min_mbps{0.75};
    double up_hysteresis_mbps{1.0};
    double down_hold_seconds{2.0};
    double up_hold_seconds{5.0};
    double minimum_dwell_seconds{5.0};
    double ewma_alpha{0.2};
    uint32_t congested_queue_depth{2};
  };

  AdaptiveModeController();
  explicit AdaptiveModeController(Config config);

  uint8_t update(double now_seconds, double available_mbps, uint32_t queue_depth);
  uint8_t mode() const noexcept {return mode_;}
  double smoothed_bandwidth_mbps() const noexcept {return smoothed_mbps_;}

private:
  uint8_t desired_mode(double bandwidth_mbps, bool moving_up) const;

  Config config_;
  uint8_t mode_{1};
  double smoothed_mbps_{0.0};
  bool initialized_{false};
  double candidate_since_{0.0};
  double last_change_{0.0};
  uint8_t candidate_mode_{1};
};

}  // namespace surf_drone
