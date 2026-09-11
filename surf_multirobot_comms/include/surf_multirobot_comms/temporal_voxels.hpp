#pragma once
#include <array>
#include <map>
#include <set>
#include <string>
#include "surf_multirobot_msgs/msg/voxel_delta.hpp"

namespace surf::comms {
// Timestamp tombstones remain after deletes. Epoch IDs are identities, not ordered integers.
class TemporalVoxels {
public:
  using Delta = surf_multirobot_msgs::msg::VoxelDelta;
  uint64_t stale{0}, retired_packets{0}, attempted_regressions{0};
  bool filter(Delta & delta) {
    const auto count = delta.x.size();
    if (delta.y.size() != count || delta.z.size() != count || delta.state.size() != count ||
      delta.observation_time_ns.size() != count) return false;
    if (retired_.count(delta.map_epoch)) {++retired_packets; return false;}
    if (initialized_ && epoch_ == delta.map_epoch &&
      (frame_ != delta.header.frame_id || resolution_ != delta.resolution)) return false;
    if (!initialized_ || epoch_ != delta.map_epoch) {
      if (initialized_) retired_.insert(epoch_);
      epoch_ = delta.map_epoch; initialized_ = true; newest_.clear(); states_.clear();
      frame_ = delta.header.frame_id; resolution_ = delta.resolution;
    }
    Delta valid = delta;
    valid.x.clear(); valid.y.clear(); valid.z.clear(); valid.state.clear();
    valid.observation_time_ns.clear();
    for (std::size_t i = 0; i < delta.x.size(); ++i) {
      const std::array<int32_t, 3> c{delta.x[i], delta.y[i], delta.z[i]};
      const auto stamp = delta.observation_time_ns[i];
      auto it = newest_.find(c);
      if (it != newest_.end() && stamp <= it->second) {
        ++stale; if (stamp < it->second) ++attempted_regressions; continue;
      }
      newest_[c] = stamp; states_[c] = delta.state[i];
      valid.x.push_back(c[0]); valid.y.push_back(c[1]); valid.z.push_back(c[2]);
      valid.state.push_back(delta.state[i]); valid.observation_time_ns.push_back(stamp);
    }
    delta = std::move(valid); return true;
  }
  Delta snapshot(const Delta & metadata) const {
    Delta out = metadata;
    out.x.clear(); out.y.clear(); out.z.clear(); out.state.clear(); out.observation_time_ns.clear();
    for (const auto & [c, stamp] : newest_) {
      out.x.push_back(c[0]); out.y.push_back(c[1]); out.z.push_back(c[2]);
      out.state.push_back(states_.at(c)); out.observation_time_ns.push_back(stamp);
    }
    return out;
  }
private:
  bool initialized_{false}; uint64_t epoch_{0}; float resolution_{0}; std::string frame_;
  std::set<uint64_t> retired_;
  std::map<std::array<int32_t, 3>, uint64_t> newest_;
  std::map<std::array<int32_t, 3>, uint8_t> states_;
};
}  // namespace surf::comms
