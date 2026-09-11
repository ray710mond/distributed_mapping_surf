#pragma once
#include "surf_drone/communication_state.hpp"
#include <algorithm>
#include <array>
#include <limits>

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
  std::unordered_map<Coord, Entry, CoordHash> entries;
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
    generated[0] += initial_priority; disturbance[0] += initial_priority; return true;
  }
  void exclude(Coord c) {
    auto it = entries.find(c);
    if (it == entries.end()) return;
    if (it->second.pending) {
      excluded[it->second.stream] += it->second.priority;
      disturbance[it->second.stream] -= it->second.priority;
    }
    entries.erase(it);
  }
  void recover(double now) {
    for (auto & [c, e] : entries) {
      (void)c;
      if (!e.pending) {e.pending = true; e.stream = 1; e.created = now;
        generated[1] += e.priority; disturbance[1] += e.priority;}
      if (e.stream == 0) {
        reclassified[0] += e.priority; disturbance[0] -= e.priority;
        disturbance[1] += e.priority; e.stream = 1;
      }
      e.packet = 0; e.acknowledged_stamp = 0;
    }
  }
  void advance(double now, double defer_seconds, double timeout) {
    for (auto & [c, e] : entries) {
      (void)c; if (!e.pending) continue;
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
      }
    }
  }
  std::array<double, 2> score(double now, const std::array<PriorityWeights, 2> & weights,
    const tf2::Vector3 & peer, double resolution, bool peer_valid = true) {
    std::array<double, 2> debt{};
    for (auto & [c, e] : entries) {
      if (!e.pending) continue;
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
  std::vector<Entry *> candidates(int stream, std::size_t limit = std::numeric_limits<std::size_t>::max()) {
    std::vector<Entry *> out;
    for (auto & [c, e] : entries) {
      (void)c; if (e.pending && !e.packet && e.stream == stream) out.push_back(&e);
    }
    const auto higher_priority = [](const auto * a, const auto * b) {
      if (a->priority != b->priority) return a->priority > b->priority;
      if (a->stamp != b->stamp) return a->stamp < b->stamp;
      return std::tie(a->coord.x, a->coord.y, a->coord.z) <
        std::tie(b->coord.x, b->coord.y, b->coord.z);
    };
    if (out.size() > limit) {
      std::nth_element(out.begin(), out.begin() + limit, out.end(), higher_priority);
      out.resize(limit);
    }
    std::sort(out.begin(), out.end(), higher_priority);
    return out;
  }
};
}  // namespace surf_drone
