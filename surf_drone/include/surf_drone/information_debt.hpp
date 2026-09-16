#pragma once
#include "surf_drone/communication_state.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <unordered_set>
#include <list>
#include <queue>

namespace surf_drone {
struct PriorityWeights {
  double base{1}, age{1}, proximity{1}, dynamic{1}, destructive{1}, occupied{2};
  double age_seconds{10}, distance_metres{10};
};
inline double information_priority(const PriorityWeights & w, double age, double distance,
  uint8_t state) {
  return w.base + w.age * std::clamp(age / w.age_seconds, 0.0, 1.0) +
    w.proximity / (1.0 + std::max(0.0, distance) / w.distance_metres) +
    w.dynamic * (state == 2) + w.destructive * (state == 3 || state == 4) +
    w.occupied * (state == 1 || state == 2);
}
// Stable round-robin traversal with O(1) insertion/removal, independent of hash
// rehashing. Resolved entries can be removed while retaining recovery history.
class ActiveCoordinates {
  std::list<Coord> order_;
  std::unordered_map<Coord, std::list<Coord>::iterator, CoordHash> index_;
public:
  ActiveCoordinates() = default;
  ActiveCoordinates(const ActiveCoordinates &) = delete;
  ActiveCoordinates & operator=(const ActiveCoordinates &) = delete;
  ActiveCoordinates(ActiveCoordinates &&) = default;
  ActiveCoordinates & operator=(ActiveCoordinates &&) = default;
  auto begin() const {return order_.begin();}
  auto end() const {return order_.end();}
  std::size_t size() const {return order_.size();}
  bool empty() const {return order_.empty();}
  const Coord & front() const {return order_.front();}
  void insert(Coord c) {
    if (index_.count(c)) return;
    order_.push_back(c); index_.emplace(c, std::prev(order_.end()));
  }
  void erase(Coord c) {
    auto it = index_.find(c);
    if (it != index_.end()) {order_.erase(it->second); index_.erase(it);}
  }
  Coord rotate() {
    const auto c = order_.front();
    order_.splice(order_.end(), order_, order_.begin());
    return c;
  }
};

// One authoritative observation per coordinate, retained after ACK for receiver recovery.
// Observation stamps are strictly increasing; equal-time conflicting states are rejected.
class InformationDebt {
public:
  struct Entry {
    Coord coord; uint8_t state{0}; uint64_t stamp{0};
    bool pending{true}; int stream{0}; uint64_t packet{0};
    double created{0}, sent{0}, priority{1};
    uint64_t acknowledged_stamp{0};
    double spatial_bonus{0}; // Selection only; does not alter conserved information debt.
    uint8_t ray_flag{0};
    geometry_msgs::msg::Point ray_origin, ray_endpoint;
  };
  struct CycleSnapshot {
    std::array<double, 2> debt{};
    std::array<uint64_t, 2> pending_count{};
    std::array<uint64_t, 2> inflight_count{};
    std::array<uint64_t, 8> pending_age_bucket_count{};
    std::array<double, 2> max_priority_remaining{};
    uint64_t ack_lag_samples{0};
    double oldest_pending_age{0};
    double oldest_observation_age_s{0};
    double newest_observation_age_s{0};
    double max_ack_observation_lag_s{0};
    std::array<std::vector<Entry *>, 2> candidates;
    uint64_t scored_entries{0};
    bool sampled_metrics{false};
  };
  std::unordered_map<Coord, Entry, CoordHash> entries;
  // Scheduling walks only outstanding entries. Resolved entries remain in
  // entries for recovery without increasing every control-step traversal.
  ActiveCoordinates active;
private:
  ActiveCoordinates fresh_, active_regions_, history_;
  std::size_t recovery_remaining_{0};
  struct Region {
    ActiveCoordinates pending;
    uint64_t revision{0}, known{0};
  };
  std::unordered_map<Coord, Region, CoordHash> regions_;
  std::array<uint64_t, 2> pending_counts_{}, inflight_counts_{};
  uint64_t completed_regions_{0};
  double region_started_{0};
  Coord previous_region_{};
  bool have_region_{false};
  std::array<uint64_t, 2> completed_clusters_{};
  static std::vector<Coord> neighbours(Coord c, int radius) {
    std::vector<Coord> result;
    result.reserve(26 + 6 * (radius - 1));
    // Immediate 26-neighborhood groups diagonally quantized samples. Larger
    // radii bridge only axial quantization holes; a full radius-two cube would
    // require 124 hash probes per visited voxel and recreate callback stalls.
    for (int dx=-1; dx<=1; ++dx)
      for (int dy=-1; dy<=1; ++dy)
        for (int dz=-1; dz<=1; ++dz)
          if (dx || dy || dz) result.push_back({c.x+dx,c.y+dy,c.z+dz});
    for (int offset=2; offset<=radius; ++offset) {
      result.push_back({c.x+offset,c.y,c.z}); result.push_back({c.x-offset,c.y,c.z});
      result.push_back({c.x,c.y+offset,c.z}); result.push_back({c.x,c.y-offset,c.z});
      result.push_back({c.x,c.y,c.z+offset}); result.push_back({c.x,c.y,c.z-offset});
    }
    return result;
  }
  void add_pending(Coord c, bool new_coordinate) {
    active.insert(c); fresh_.insert(c);
    auto & region = regions_[region_key(c)];
    region.pending.insert(c); ++region.revision;
    if (new_coordinate) {++region.known; history_.insert(c);}
    active_regions_.insert(region_key(c));
  }
  void remove_pending(Coord c, bool delivered) {
    active.erase(c); fresh_.erase(c);
    auto & region = regions_.at(region_key(c));
    region.pending.erase(c);
    if (region.pending.empty()) {
      active_regions_.erase(region_key(c));
      if (delivered) ++completed_regions_;
    }
  }
  void clear_packet(Entry & e) {
    if (e.packet && inflight_counts_[e.stream]) --inflight_counts_[e.stream];
    e.packet = 0;
  }
  void defer(Entry & e) {
    reclassified[0] += e.priority; disturbance[0] -= e.priority;
    disturbance[1] += e.priority;
    --pending_counts_[0]; ++pending_counts_[1]; e.stream = 1;
  }
public:
  // 16^3 cells bounds an individual region to 4096 records, at any resolution.
  static Coord region_key(Coord c) {
    auto floor = [](int32_t x) {return x / 16 - (x % 16 < 0 ? 1 : 0);};
    return {floor(c.x), floor(c.y), floor(c.z)};
  }
  struct RegionProgress {
    bool valid{false}; Coord coord{};
    uint64_t revision{0}, known{0}, pending{0}, completed{0};
  };
  RegionProgress region_progress() const {
    if (active_regions_.empty()) return {false, {}, 0, 0, 0, completed_regions_};
    const auto c = active_regions_.front(); const auto & r = regions_.at(c);
    return {true, c, r.revision, r.known, r.pending.size(), completed_regions_};
  }
  void mark_sent(Entry & e, uint64_t packet, double now) {
    if (!e.packet) ++inflight_counts_[e.stream];
    e.packet = packet; e.sent = now;
  }
  struct ClusterSelection {
    std::vector<Entry *> entries;
    // Exclusive end offsets identify proximity-connected groups in output order.
    std::vector<std::size_t> component_ends;
    Coord seed{};
    uint64_t visited{0};
    uint64_t pending{0};
    uint64_t components{0};
    uint64_t completed{0};
    bool valid{false};
    bool truncated{false};
  };

  // Select bounded proximity-connected components of known evidence. Traversal may cross
  // already acknowledged records, which lets the frontier grow continuously
  // from delivered space. UNKNOWN records break connectivity. The search is
  // bounded and never scans the full ledger.
  ClusterSelection connected_candidates(
    int stream, const std::vector<Entry *> & ranked_seeds, std::size_t limit,
    std::size_t traversal_budget, int neighbour_radius = 1)
  {
    ClusterSelection result;
    if (ranked_seeds.empty() || limit == 0 || traversal_budget == 0) {
      return result;
    }
    auto eligible = [&](const Entry & e) {
      return e.pending && !e.packet && e.stream == stream && e.state != 5;
    };
    Coord seed = ranked_seeds.front()->coord;
    // Reconsider the focus each cycle: a previously truncated component must
    // not hide new change hotspots or the age-reserved seeds.
    for (auto * candidate : ranked_seeds) {
      if (!eligible(*candidate)) continue;
      bool anchored = false;
      for (const auto & neighbour : neighbours(candidate->coord, neighbour_radius)) {
        auto it = entries.find(neighbour);
        if (it != entries.end() && !it->second.pending && it->second.state != 5) {
          anchored = true; break;
        }
      }
      if (anchored) {seed = candidate->coord; break;}
    }

    result.seed = seed;
    std::unordered_set<Coord, CoordHash> visited;
    std::size_t next_seed = 0;
    bool first_component = true;
    while (result.entries.size() < limit && visited.size() < traversal_budget) {
      if (!first_component) {
        while (next_seed < ranked_seeds.size() &&
          (visited.count(ranked_seeds[next_seed]->coord) || !eligible(*ranked_seeds[next_seed])))
          ++next_seed;
        if (next_seed == ranked_seeds.size()) break;
        seed = ranked_seeds[next_seed++]->coord;
      }
      first_component = false;
      std::queue<Coord> frontier;
      if (!visited.insert(seed).second) continue;
      frontier.push(seed);
      const auto before = result.entries.size();
      while (!frontier.empty() && visited.size() < traversal_budget && result.entries.size() < limit) {
        const Coord current = frontier.front(); frontier.pop();
        auto found = entries.find(current);
        if (found == entries.end() || found->second.state == 5) continue;
        if (eligible(found->second)) result.entries.push_back(&found->second);
        for (const auto & neighbour : neighbours(current, neighbour_radius)) {
          if (visited.size() >= traversal_budget) break;
          auto adjacent = entries.find(neighbour);
          if (adjacent != entries.end() && adjacent->second.state != 5 &&
            visited.insert(neighbour).second) frontier.push(neighbour);
        }
      }
      if (result.entries.size() > before) {
        result.component_ends.push_back(result.entries.size());
        ++result.components;
      }
      if (!frontier.empty()) {
        result.truncated = true;
        break;
      }
      // A component is complete only when its connected known evidence has no
      // eligible pending record. Selecting its records is not completion; ACKs
      // remain authoritative.
      if (result.entries.size() == before) ++completed_clusters_[stream];
    }
    result.visited = visited.size();
    result.pending = result.entries.size();
    result.valid = true;
    result.completed = completed_clusters_[stream];
    return result;
  }
  std::array<double, 2> generated{}, acknowledged{}, superseded{}, reclassified{}, disturbance{}, timed_out{}, excluded{};
  uint64_t superseded_count{0}, stale_observations{0}, stale_pending_discarded{0};
  bool observe(Coord c, uint8_t state, uint64_t stamp, double now, double initial_priority = 1.0,
    uint8_t ray_flag = 0, geometry_msgs::msg::Point ray_origin = geometry_msgs::msg::Point(),
    geometry_msgs::msg::Point ray_endpoint = geometry_msgs::msg::Point()) {
    auto it = entries.find(c);
    const uint64_t known_stamp = it == entries.end() ? 0 : it->second.acknowledged_stamp;
    if (it != entries.end()) {
      if (stamp <= it->second.stamp) {++stale_observations; return false;}
      // Unchanged physical state does not create new debt or starve in-flight ACKs.
      if (state == it->second.state) return false;
      if (it->second.pending) {
        if (!it->second.packet) ++stale_pending_discarded;
        --pending_counts_[it->second.stream];
        clear_packet(it->second);
        superseded[it->second.stream] += it->second.priority;
        disturbance[it->second.stream] -= it->second.priority; ++superseded_count;
      }
    }
    const bool new_coordinate = it == entries.end();
    entries[c] = {c, state, stamp, true, 0, 0, now, 0, initial_priority, known_stamp,
      0, ray_flag, ray_origin, ray_endpoint};
    entries[c].ray_flag = ray_flag;
    entries[c].ray_origin = ray_origin;
    entries[c].ray_endpoint = ray_endpoint;
    ++pending_counts_[0]; add_pending(c, new_coordinate);
    generated[0] += initial_priority; disturbance[0] += initial_priority; return true;
  }
  void exclude(Coord c) {
    auto it = entries.find(c);
    if (it == entries.end()) return;
    if (it->second.pending) {
      --pending_counts_[it->second.stream]; clear_packet(it->second);
      excluded[it->second.stream] += it->second.priority;
      disturbance[it->second.stream] -= it->second.priority;
    }
    remove_pending(c, false);
    --regions_.at(region_key(c)).known;
    history_.erase(c); entries.erase(it);
  }
  void request_recovery() {recovery_remaining_ = history_.size();}
  std::size_t recovery_remaining() const {return recovery_remaining_;}
  std::size_t recovery_slice(double now, std::size_t budget) {
    const auto count = std::min({budget, recovery_remaining_, history_.size()});
    for (std::size_t i=0; i<count; ++i) {
      const auto c = history_.rotate(); auto & e = entries.at(c);
      if (!e.pending) {
        e.pending = true; e.stream = 1; e.created = now;
        generated[1] += e.priority; disturbance[1] += e.priority;
        ++pending_counts_[1]; add_pending(c, false);
      }
      clear_packet(e);
      if (e.stream == 0) defer(e);
      e.acknowledged_stamp = 0;
    }
    recovery_remaining_ -= count;
    if (history_.empty()) recovery_remaining_ = 0;
    return count;
  }
  void timeout_packet(uint64_t packet, const std::vector<Coord> & coordinates) {
    for (const auto & c : coordinates) {
      auto it = entries.find(c);
      if (it != entries.end() && it->second.packet == packet) {
        timed_out[it->second.stream] += it->second.priority; clear_packet(it->second);
      }
    }
  }
  void recover(double now) {
    for (auto & [c, e] : entries) {
      (void)c;
      if (!e.pending) {e.pending = true; e.stream = 1; e.created = now;
        generated[1] += e.priority; disturbance[1] += e.priority; ++pending_counts_[1]; add_pending(c, false);}
      clear_packet(e);
      if (e.stream == 0) defer(e);
      e.packet = 0; e.acknowledged_stamp = 0;
    }
  }
  void advance(double now, double defer_seconds, double timeout) {
    for (const auto & c : active) {
      auto & e = entries.at(c);
      if (e.packet && now - e.sent >= timeout) {timed_out[e.stream] += e.priority; clear_packet(e);}
      if (!e.packet && e.stream == 0 && now - e.created >= defer_seconds) {
        defer(e);
      }
    }
  }
  double ack(uint64_t packet) {
    if (packet == 0) return 0;
    double resolved = 0;
    for (auto & [c, e] : entries) {
      (void)c;
      if (e.pending && e.packet == packet) {
        resolved += e.priority;
        acknowledged[e.stream] += e.priority; --pending_counts_[e.stream];
        clear_packet(e); e.pending = false;
        e.acknowledged_stamp = e.stamp;
        remove_pending(c, true);
      }
    }
    return resolved;
  }
  double ack(uint64_t packet, const std::vector<Coord> & coordinates) {
    if (!packet) return 0;
    double resolved = 0;
    for (const auto & c : coordinates) {
      auto it = entries.find(c);
      if (it == entries.end()) continue;
      auto & e = it->second;
      if (e.pending && e.packet == packet) {
        resolved += e.priority;
        acknowledged[e.stream] += e.priority; --pending_counts_[e.stream];
        clear_packet(e); e.pending = false;
        e.acknowledged_stamp = e.stamp;
        remove_pending(c, true);
      }
    }
    return resolved;
  }
  std::array<double, 2> score(double now, const std::array<PriorityWeights, 2> & weights,
    const tf2::Vector3 & peer, double resolution, bool peer_valid = true) {
    std::array<double, 2> debt{};
    for (const auto & c : active) {
      auto & e = entries.at(c);
      const tf2::Vector3 position((c.x + .5) * resolution, (c.y + .5) * resolution,
        (c.z + .5) * resolution);
      const double previous = e.priority;
      e.priority = information_priority(weights[e.stream], now - e.created,
        peer_valid ? (position - peer).length() : std::numeric_limits<double>::infinity(), e.state);
      disturbance[e.stream] += e.priority - previous;
      debt[e.stream] += e.priority;
    }
    return debt;
  }
  // Performs timeout/defer transitions, priority scoring, telemetry aggregation,
  // and candidate collection in one active-set traversal. The controller used to
  // walk this set five times per cycle (advance, score, metrics, and two streams).
  CycleSnapshot cycle(
    double now, double observation_now, double defer_seconds, double timeout,
    const std::array<PriorityWeights, 2> & weights, const tf2::Vector3 & peer,
    double resolution, bool peer_valid = true)
  {
    CycleSnapshot snapshot;
    bool first_pending = true;
    for (const auto & c : active) {
      auto & e = entries.at(c);
      if (e.packet && now - e.sent >= timeout) {
        timed_out[e.stream] += e.priority;
        clear_packet(e);
      }
      if (!e.packet && e.stream == 0 && now - e.created >= defer_seconds) {
        defer(e);
      }

      const tf2::Vector3 position(
        (c.x + .5) * resolution, (c.y + .5) * resolution, (c.z + .5) * resolution);
      const double previous = e.priority;
      e.priority = information_priority(
        weights[e.stream], now - e.created,
        peer_valid ? (position - peer).length() : std::numeric_limits<double>::infinity(),
        e.state);
      disturbance[e.stream] += e.priority - previous;
      snapshot.debt[e.stream] += e.priority;
      ++snapshot.pending_count[e.stream];
      snapshot.max_priority_remaining[e.stream] =
        std::max(snapshot.max_priority_remaining[e.stream], e.priority);

      const double pending_age = std::max(0.0, now - e.created);
      const int age_bucket = pending_age < 10 ? 0 : pending_age < 30 ? 1 :
        pending_age < 60 ? 2 : 3;
      ++snapshot.pending_age_bucket_count[e.stream * 4 + age_bucket];
      snapshot.oldest_pending_age = std::max(snapshot.oldest_pending_age, pending_age);
      const double observation_age = std::max(0.0, observation_now - e.stamp / 1e9);
      snapshot.oldest_observation_age_s =
        std::max(snapshot.oldest_observation_age_s, observation_age);
      snapshot.newest_observation_age_s = first_pending ? observation_age :
        std::min(snapshot.newest_observation_age_s, observation_age);
      first_pending = false;
      if (e.acknowledged_stamp) {
        ++snapshot.ack_lag_samples;
        snapshot.max_ack_observation_lag_s = std::max(
          snapshot.max_ack_observation_lag_s,
          (e.stamp - e.acknowledged_stamp) / 1e9);
      }
      if (e.packet) {
        ++snapshot.inflight_count[e.stream];
      } else {
        snapshot.candidates[e.stream].push_back(&e);
      }
    }
    return snapshot;
  }

  // Cached global debt/counts remain conserved. Only age/proximity scores and
  // age/max telemetry are refreshed over a bounded sample, explicitly labelled.
  // New observations, round-robin history, and the focused region each get work.
  CycleSnapshot bounded_cycle(double now, double observation_now, double defer_seconds,
    double timeout, const std::array<PriorityWeights, 2> & weights,
    const tf2::Vector3 & peer, double resolution, bool peer_valid, std::size_t budget)
  {
    CycleSnapshot snapshot; snapshot.sampled_metrics = true;
    std::unordered_set<Entry *> visited;
    auto refresh = [&](Coord c) {
      auto & e = entries.at(c);
      if (!visited.insert(&e).second) return;
      ++snapshot.scored_entries;
      if (e.packet && now - e.sent >= timeout) {timed_out[e.stream] += e.priority; clear_packet(e);}
      if (!e.packet && e.stream == 0 && now - e.created >= defer_seconds) defer(e);
      const tf2::Vector3 position((c.x+.5)*resolution, (c.y+.5)*resolution, (c.z+.5)*resolution);
      const double previous = e.priority;
      e.priority = information_priority(weights[e.stream], now-e.created,
        peer_valid ? (position-peer).length() : std::numeric_limits<double>::infinity(), e.state);
      disturbance[e.stream] += e.priority - previous;
      const double age = std::max(0.0, now-e.created);
      ++snapshot.pending_age_bucket_count[e.stream*4 + (age<10 ? 0 : age<30 ? 1 : age<60 ? 2 : 3)];
      snapshot.oldest_pending_age = std::max(snapshot.oldest_pending_age, age);
      snapshot.max_priority_remaining[e.stream] = std::max(snapshot.max_priority_remaining[e.stream], e.priority);
      const double observation_age = std::max(0.0, observation_now-e.stamp/1e9);
      snapshot.oldest_observation_age_s = std::max(snapshot.oldest_observation_age_s, observation_age);
      snapshot.newest_observation_age_s = snapshot.scored_entries == 1 ? observation_age :
        std::min(snapshot.newest_observation_age_s, observation_age);
      if (e.acknowledged_stamp) {
        ++snapshot.ack_lag_samples;
        snapshot.max_ack_observation_lag_s = std::max(snapshot.max_ack_observation_lag_s,
          (e.stamp-e.acknowledged_stamp)/1e9);
      }
      if (!e.packet) snapshot.candidates[e.stream].push_back(&e);
    };
    // New observations receive half the bounded work so a compressible packet
    // stream can expose enough eligible records to spend its byte allocation.
    const auto fresh_count = std::min(fresh_.size(), budget/2);
    for (std::size_t i=0; i<fresh_count; ++i) {
      const auto c = fresh_.front(); fresh_.erase(c); refresh(c);
    }
    if (!active_regions_.empty()) {
      // A continuously changing region cannot monopolize BACKLOG indefinitely.
      if (have_region_ && active_regions_.front() == previous_region_ && now-region_started_ >= 10)
        active_regions_.rotate();
      if (!have_region_ || !(active_regions_.front() == previous_region_) || now-region_started_ >= 10) {
        previous_region_ = active_regions_.front(); region_started_ = now; have_region_ = true;
      }
      auto & pending = regions_.at(active_regions_.front()).pending;
      const auto count = std::min(pending.size(), budget/4);
      for (std::size_t i=0; i<count; ++i) refresh(pending.rotate());
    } else have_region_ = false;
    const auto count = std::min(active.size(), budget - fresh_count - std::min<std::size_t>(budget/4,
      active_regions_.empty() ? 0 : regions_.at(active_regions_.front()).pending.size()));
    for (std::size_t i=0; i<count; ++i) refresh(active.rotate());
    snapshot.pending_count = pending_counts_; snapshot.inflight_count = inflight_counts_;
    for (int stream=0; stream<2; ++stream)
      snapshot.debt[stream] = pending_counts_[stream] ?
        std::max(0.0, disturbance[stream]-acknowledged[stream]) : 0;
    return snapshot;
  }

  void prefer_region(std::vector<Entry *> & candidates) const {
    if (active_regions_.empty()) return;
    const auto focus = active_regions_.front();
    std::stable_partition(candidates.begin(), candidates.end(), [&](const Entry * e) {
      return region_key(e->coord) == focus;
    });
  }

  // Score local outstanding change, including neighbors outside the bounded
  // candidate sample. Resolved history and updates already on the wire do not
  // inflate density. The logarithm favors coherent changes without making a
  // singleton worthless; the age reserve in rank_candidates still comes first.
  void score_spatial_change(const std::vector<Entry *> & candidates,
    double weight = 1.0, int radius = 2) const
  {
    radius = std::clamp(radius, 1, 4);
    for (auto * e : candidates) {
      e->spatial_bonus = 0;
      if (weight <= 0 || e->state == 5) continue;
      std::size_t changing = 0;
      for (const auto & c : neighbours(e->coord, radius)) {
        const auto it = entries.find(c);
        if (it != entries.end() && it->second.pending && !it->second.packet &&
          it->second.state != 5) ++changing;
      }
      e->spatial_bonus = weight * std::log1p(static_cast<double>(changing));
    }
  }

  static std::vector<Entry *> rank_candidates(
    std::vector<Entry *> out,
    std::size_t limit = std::numeric_limits<std::size_t>::max(),
    double now = 0, double starvation_age = std::numeric_limits<double>::infinity(),
    std::size_t starvation_reserve = 0)
  {
    const auto higher_priority = [](const auto * a, const auto * b) {
      const double a_score = a->priority + a->spatial_bonus;
      const double b_score = b->priority + b->spatial_bonus;
      if (a_score != b_score) return a_score > b_score;
      if (a->stamp != b->stamp) return a->stamp < b->stamp;
      return std::tie(a->coord.x, a->coord.y, a->coord.z) <
        std::tie(b->coord.x, b->coord.y, b->coord.z);
    };
    starvation_reserve = std::min({starvation_reserve, limit, out.size()});
    std::vector<Entry *> oldest;
    if (starvation_reserve) {
      for (auto * e : out) if (now - e->created >= starvation_age) oldest.push_back(e);
      const auto older = [](const auto * a, const auto * b) {
          if (a->created != b->created) return a->created < b->created;
          if (a->stamp != b->stamp) return a->stamp < b->stamp;
          return std::tie(a->coord.x, a->coord.y, a->coord.z) <
            std::tie(b->coord.x, b->coord.y, b->coord.z);
        };
      if (oldest.size() > starvation_reserve) {
        std::nth_element(oldest.begin(), oldest.begin() + starvation_reserve, oldest.end(), older);
        oldest.resize(starvation_reserve);
      }
      std::sort(oldest.begin(), oldest.end(), older);
    }
    const std::unordered_set<Entry *> reserved(oldest.begin(), oldest.end());
    out.erase(std::remove_if(out.begin(), out.end(),
      [&](auto * e) {return reserved.find(e) != reserved.end();}), out.end());
    const std::size_t priority_limit = limit - oldest.size();
    if (out.size() > priority_limit) {
      std::nth_element(out.begin(), out.begin() + priority_limit, out.end(), higher_priority);
      out.resize(priority_limit);
    }
    std::sort(out.begin(), out.end(), higher_priority);
    oldest.insert(oldest.end(), out.begin(), out.end());
    return oldest;
  }

  std::vector<Entry *> candidates(
    int stream, std::size_t limit = std::numeric_limits<std::size_t>::max(),
    double now = 0, double starvation_age = std::numeric_limits<double>::infinity(),
    std::size_t starvation_reserve = 0)
  {
    std::vector<Entry *> out;
    for (const auto & c : active) {
      auto & e = entries.at(c);
      if (!e.packet && e.stream == stream) out.push_back(&e);
    }
    return rank_candidates(std::move(out), limit, now, starvation_age, starvation_reserve);
  }
};
}  // namespace surf_drone
