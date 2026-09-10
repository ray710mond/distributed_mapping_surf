#pragma once

#include <cmath>
#include <cstdint>
#include <utility>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "tf2/LinearMath/Quaternion.h"

namespace surf_drone
{
struct Coord
{
  int32_t x{0}, y{0}, z{0};
  bool operator==(const Coord & b) const noexcept {return x == b.x && y == b.y && z == b.z;}
};
struct CoordHash
{
  std::size_t operator()(const Coord & value) const noexcept
  {
    std::size_t seed = std::hash<int32_t>{}(value.x);
    seed ^= std::hash<int32_t>{}(value.y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    seed ^= std::hash<int32_t>{}(value.z) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    return seed;
  }
};

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

inline Coord quantize(double x, double y, double z, double resolution)
{
  return {
    static_cast<int32_t>(std::floor(x / resolution)),
    static_cast<int32_t>(std::floor(y / resolution)),
    static_cast<int32_t>(std::floor(z / resolution))};
}

struct HumanoidMask
{
  tf2::Vector3 center;
  tf2::Quaternion world_from_model;
  double half_x{0.0};
  double half_y{0.0};
  double half_z{0.0};

  bool contains(double x, double y, double z) const
  {
    const tf2::Vector3 local =
      tf2::quatRotate(world_from_model.inverse(), tf2::Vector3(x, y, z) - center);
    return std::abs(local.x()) <= half_x &&
           std::abs(local.y()) <= half_y &&
           std::abs(local.z()) <= half_z;
  }

  std::pair<Coord, Coord> voxel_bounds(double resolution) const
  {
    // Enclose all eight corners of the oriented mask. Exact center testing
    // below retains the original mask semantics, including rotated boxes.
    tf2::Vector3 low = center, high = center;
    for (int x : {-1, 1}) {
      for (int y : {-1, 1}) {
        for (int z : {-1, 1}) {
          const auto corner = center + tf2::quatRotate(world_from_model,
            tf2::Vector3(x * half_x, y * half_y, z * half_z));
          low.setMin(corner);
          high.setMax(corner);
        }
      }
    }
    return {quantize(low.x(), low.y(), low.z(), resolution),
      quantize(high.x(), high.y(), high.z(), resolution)};
  }

  bool contains(const Coord & coord, double resolution) const
  {
    // Test voxel centers. The configured padding is at least one voxel in
    // the configured occupancy profile, making boundary quantization conservative.
    return contains(
      (static_cast<double>(coord.x) + 0.5) * resolution,
      (static_cast<double>(coord.y) + 0.5) * resolution,
      (static_cast<double>(coord.z) + 0.5) * resolution);
  }
};


// Fixed voxel blocks avoid work proportional to accumulated map size for local queries.
// Mutations must go through this wrapper so the spatial index cannot become stale.
template<typename Value>
class SpatialMap
{
  using Map = std::unordered_map<Coord, Value, CoordHash>;
  Map values_;
  std::unordered_map<Coord, std::unordered_set<Coord, CoordHash>, CoordHash> blocks_;
  static int32_t floor_block(int32_t v)
  {
    return v / 16 - (v % 16 < 0 ? 1 : 0);
  }
  static Coord block(const Coord & c)
  {
    return {floor_block(c.x), floor_block(c.y), floor_block(c.z)};
  }
public:
  using iterator = typename Map::iterator;
  Value & operator[](const Coord & c)
  {
    auto [it, inserted] = values_.try_emplace(c);
    if (inserted) {blocks_[block(c)].insert(c);}
    return it->second;
  }
  iterator begin() {return values_.begin();}
  iterator end() {return values_.end();}
  auto begin() const {return values_.begin();}
  auto end() const {return values_.end();}
  iterator find(const Coord & c) {return values_.find(c);}
  auto find(const Coord & c) const {return values_.find(c);}
  std::size_t size() const {return values_.size();}
  iterator erase(iterator it)
  {
    auto b = blocks_.find(block(it->first));
    b->second.erase(it->first);
    if (b->second.empty()) {blocks_.erase(b);}
    return values_.erase(it);
  }
  std::size_t erase(const Coord & c)
  {
    auto it = find(c);
    if (it == end()) {return 0;}
    erase(it);
    return 1;
  }
  // Returns a snapshot so callers may erase matching entries during traversal.
  std::vector<Coord> in_box(const Coord & low, const Coord & high) const
  {
    std::vector<Coord> result;
    const auto a = block(low), b = block(high);
    for (int64_t x = a.x; x <= b.x; ++x) {
      for (int64_t y = a.y; y <= b.y; ++y) {
        for (int64_t z = a.z; z <= b.z; ++z) {
          auto it = blocks_.find({static_cast<int32_t>(x), static_cast<int32_t>(y),
              static_cast<int32_t>(z)});
          if (it == blocks_.end()) {continue;}
          for (const auto & c : it->second) {
            if (c.x >= low.x && c.x <= high.x && c.y >= low.y && c.y <= high.y &&
              c.z >= low.z && c.z <= high.z) {result.push_back(c);}
          }
        }
      }
    }
    return result;
  }
};

// One expiry entry per dynamic coordinate. Refreshing an observation reschedules it;
// promotion to static or removal cancels it without retaining stale heap entries.
class DynamicExpiry
{
  std::map<uint64_t, std::unordered_set<Coord, CoordHash>> due_;
  std::unordered_map<Coord, uint64_t, CoordHash> versions_;
public:
  void cancel(const Coord & c)
  {
    auto it = versions_.find(c);
    if (it == versions_.end()) {return;}
    auto bucket = due_.find(it->second);
    bucket->second.erase(c);
    if (bucket->second.empty()) {due_.erase(bucket);}
    versions_.erase(it);
  }
  void schedule(const Coord & c, uint64_t version)
  {
    cancel(c);
    versions_[c] = version;
    due_[version].insert(c);
  }
  std::vector<Coord> pop_due(uint64_t version)
  {
    std::vector<Coord> result;
    while (!due_.empty() && due_.begin()->first <= version) {
      for (const auto & c : due_.begin()->second) {
        result.push_back(c);
        versions_.erase(c);
      }
      due_.erase(due_.begin());
    }
    return result;
  }
  std::size_t size() const {return versions_.size();}
};
}  // namespace surf_drone
