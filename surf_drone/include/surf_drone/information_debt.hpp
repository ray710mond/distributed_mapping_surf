#pragma once
#include "surf_drone/communication_state.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <unordered_set>

namespace surf_drone {
struct PriorityWeights {
  double base{1}, age{1}, proximity{1}, dynamic{1}, destructive{1};
  double age_seconds{10}, distance_metres{10};
};
inline double information_priority(const PriorityWeights & w, double age, double distance,
  uint8_t state) {
  return w.base + w.age * std::clamp(age / w.age_seconds, 0.0, 1.0) +
    w.proximity / (1.0 + std::max(0.0, distance) / w.distance_metres) +
    w.dynamic * (state == 2) + w.destructive * (state == 3 || state == 4);
}
// One authoritative observation per coordinate, retained after ACK for receiver recovery.
// Observation stamps are strictly increasing; equal-time conflicting states are rejected.
class InformationDebt {
public:
  struct Entry {
    Coord coord; uint8_t state{0}; uint64_t stamp{0};
    bool pending{true}; int stream{0}; uint64_t packet{0};
    double created{0}, sent{0}, priority{1};
    uint64_t acknowledged_stamp{0};
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
  };
  std::unordered_map<Coord, Entry, CoordHash> entries;
  // Scheduling walks only outstanding entries. Resolved entries remain in
  // entries for recovery without increasing every control-step traversal.
  std::unordered_set<Coord, CoordHash> active;
  std::array<double, 2> generated{}, acknowledged{}, superseded{}, reclassified{}, disturbance{}, timed_out{}, excluded{};
  uint64_t superseded_count{0}, stale_observations{0}, stale_pending_discarded{0};
  bool observe(Coord c, uint8_t state, uint64_t stamp, double now, double initial_priority = 1.0) {
    auto it = entries.find(c);
    const uint64_t known_stamp = it == entries.end() ? 0 : it->second.acknowledged_stamp;
    if (it != entries.end()) {
      if (stamp <= it->second.stamp) {++stale_observations; return false;}
      // Unchanged physical state does not create new debt or starve in-flight ACKs.
      if (state == it->second.state) return false;
      if (it->second.pending) {
        if (!it->second.packet) ++stale_pending_discarded;
        superseded[it->second.stream] += it->second.priority;
        disturbance[it->second.stream] -= it->second.priority; ++superseded_count;
      }
    }
    entries[c] = {c, state, stamp, true, 0, 0, now, 0, initial_priority, known_stamp};
    active.insert(c);
    generated[0] += initial_priority; disturbance[0] += initial_priority; return true;
  }
  void exclude(Coord c) {
    auto it = entries.find(c);
    if (it == entries.end()) return;
    if (it->second.pending) {
      excluded[it->second.stream] += it->second.priority;
      disturbance[it->second.stream] -= it->second.priority;
    }
    active.erase(c); entries.erase(it);
  }
  void recover(double now) {
    for (auto & [c, e] : entries) {
      (void)c;
      if (!e.pending) {e.pending = true; e.stream = 1; e.created = now;
        generated[1] += e.priority; disturbance[1] += e.priority; active.insert(c);}
      if (e.stream == 0) {
        reclassified[0] += e.priority; disturbance[0] -= e.priority;
        disturbance[1] += e.priority; e.stream = 1;
      }
      e.packet = 0; e.acknowledged_stamp = 0;
    }
  }
  void advance(double now, double defer_seconds, double timeout) {
    for (const auto & c : active) {
      auto & e = entries.at(c);
      if (e.packet && now - e.sent >= timeout) {timed_out[e.stream] += e.priority; e.packet = 0;}
      if (!e.packet && e.stream == 0 && now - e.created >= defer_seconds) {
        reclassified[0] += e.priority; disturbance[0] -= e.priority;
        disturbance[1] += e.priority; e.stream = 1;
      }
    }
  }
  void ack(uint64_t packet) {
    if (packet == 0) return;
    for (auto & [c, e] : entries) {
      (void)c;
      if (e.pending && e.packet == packet) {
        acknowledged[e.stream] += e.priority; e.pending = false; e.packet = 0;
        e.acknowledged_stamp = e.stamp;
        active.erase(c);
      }
    }
  }
  void ack(uint64_t packet, const std::vector<Coord> & coordinates) {
    if (!packet) return;
    for (const auto & c : coordinates) {
      auto it = entries.find(c);
      if (it == entries.end()) continue;
      auto & e = it->second;
      if (e.pending && e.packet == packet) {
        acknowledged[e.stream] += e.priority; e.pending = false; e.packet = 0;
        e.acknowledged_stamp = e.stamp;
        active.erase(c);
      }
    }
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
        e.packet = 0;
      }
      if (!e.packet && e.stream == 0 && now - e.created >= defer_seconds) {
        reclassified[0] += e.priority;
        disturbance[0] -= e.priority;
        disturbance[1] += e.priority;
        e.stream = 1;
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

  static std::vector<Entry *> rank_candidates(
    std::vector<Entry *> out,
    std::size_t limit = std::numeric_limits<std::size_t>::max(),
    double now = 0, double starvation_age = std::numeric_limits<double>::infinity(),
    std::size_t starvation_reserve = 0)
  {
    const auto higher_priority = [](const auto * a, const auto * b) {
      if (a->priority != b->priority) return a->priority > b->priority;
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
