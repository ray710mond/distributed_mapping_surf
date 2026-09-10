#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <map>
#include <string>

#include "surf_multirobot_msgs/msg/voxel_delta.hpp"

namespace surf::comms
{
// Holds every *applied* realtime delta newer than the last snapshot, including
// deltas applied before the first snapshot chunk arrives. Missing network
// versions do not count as history loss: they were never applied locally.
class SnapshotRecovery
{
public:
  using Delta = surf_multirobot_msgs::msg::VoxelDelta;
  SnapshotRecovery() : SnapshotRecovery(64U * 1024U * 1024U) {}
  explicit SnapshotRecovery(std::size_t maximum_bytes)
  : maximum_bytes_(maximum_bytes) {}

  uint64_t latest_snapshot() const {return latest_snapshot_;}
  uint64_t completed_snapshot() const {return completed_snapshot_;}
  uint64_t history_floor() const {return history_floor_;}
  std::size_t history_bytes() const {return history_bytes_;}

  std::string rejection(uint64_t version) const
  {
    if (version < latest_snapshot_) {return "superseded full refresh";}
    if (version < history_floor_) {return "full refresh predates retained realtime history";}
    return {};
  }

  // Call only after decoding a valid chunk. Corrupt newer data must not evict
  // the valid in-flight snapshot.
  void begin(uint64_t version) {latest_snapshot_ = std::max(latest_snapshot_, version);}

  void remember(const Delta & delta)
  {
    if (delta.version <= completed_snapshot_ || history_.count(delta.version)) {return;}
    history_[delta.version] = delta;
    history_bytes_ += bytes(delta);
    while (history_bytes_ > maximum_bytes_ && !history_.empty()) {
      history_floor_ = std::max(history_floor_, history_.begin()->first);
      erase_first();
    }
  }

  // Publish one atomic full replacement, rebased to the newest applied version.
  // Records are ordered snapshot first, then newer deltas; the downstream map
  // applies records in order, preserving later occupied/free/delete operations.
  bool rebase(Delta & snapshot) const
  {
    if (!rejection(snapshot.version).empty()) {return false;}
    const auto first = history_.upper_bound(snapshot.version);
    for (auto it = first; it != history_.end(); ++it) {
      if (it->second.resolution != snapshot.resolution ||
        it->second.header.frame_id != snapshot.header.frame_id) {return false;}
    }
    for (auto it = first; it != history_.end(); ++it) {
      const auto & delta = it->second;
      append(snapshot, delta);
      snapshot.version = delta.version;
      snapshot.header.stamp = delta.header.stamp;
    }
    snapshot.base_version = 0U;
    snapshot.chunk_index = 0U;
    snapshot.chunk_count = 1U;
    return true;
  }

  // Use the original wire snapshot version, not the rebased publication version.
  void complete(uint64_t version)
  {
    completed_snapshot_ = version;
    latest_snapshot_ = std::max(latest_snapshot_, version);
    history_floor_ = std::max(history_floor_, version);
    while (!history_.empty() && history_.begin()->first <= version) {erase_first();}
  }

private:
  static std::size_t bytes(const Delta & d)
  {
    return sizeof(Delta) + d.header.frame_id.size() + d.source_id.size() +
           d.x.size() * sizeof(int32_t) + d.y.size() * sizeof(int32_t) +
           d.z.size() * sizeof(int32_t) + d.state.size() +
           d.observation_time_ns.size() * sizeof(uint64_t);
  }
  static void append(Delta & to, const Delta & from)
  {
    to.x.insert(to.x.end(), from.x.begin(), from.x.end());
    to.y.insert(to.y.end(), from.y.begin(), from.y.end());
    to.z.insert(to.z.end(), from.z.begin(), from.z.end());
    to.state.insert(to.state.end(), from.state.begin(), from.state.end());
    to.observation_time_ns.insert(to.observation_time_ns.end(),
      from.observation_time_ns.begin(), from.observation_time_ns.end());
  }
  void erase_first()
  {
    history_bytes_ -= bytes(history_.begin()->second);
    history_.erase(history_.begin());
  }
  std::size_t maximum_bytes_;
  std::size_t history_bytes_{0U};
  uint64_t latest_snapshot_{0U};
  uint64_t completed_snapshot_{0U};
  uint64_t history_floor_{0U};
  std::map<uint64_t, Delta> history_;
};
}  // namespace surf::comms
