#pragma once

#include "surf_drone/communication_state.hpp"
#include <algorithm>
#include <array>
#include <limits>

namespace surf_drone {
// Visit the contiguous, measured ray prefix, excluding the occupied endpoint.
// A work limit truncates the prefix; it must never increase the step length and
// invent free evidence in unvisited cells. Edge/corner-only contacts are omitted.
template<class Visitor>
std::size_t visit_observed_ray(const tf2::Vector3 & origin,
  const tf2::Vector3 & endpoint, double resolution, std::size_t limit, Visitor visit)
{
  Coord c = quantize(origin.x(), origin.y(), origin.z(), resolution);
  const Coord end = quantize(endpoint.x(), endpoint.y(), endpoint.z(), resolution);
  const auto direction = endpoint - origin;
  std::array<int, 3> step{};
  std::array<double, 3> next{}, increment{};
  int32_t * coordinates[] = {&c.x, &c.y, &c.z};
  for (int axis = 0; axis < 3; ++axis) {
    if (direction[axis] == 0) {
      next[axis] = increment[axis] = std::numeric_limits<double>::infinity();
    } else {
      step[axis] = direction[axis] > 0 ? 1 : -1;
      const double boundary = (*coordinates[axis] + (step[axis] > 0 ? 1.0 : 0.0)) * resolution;
      next[axis] = (boundary - origin[axis]) / direction[axis];
      increment[axis] = resolution / std::abs(direction[axis]);
    }
  }
  std::size_t visited = 0;
  while (!(c == end) && visited < limit) {
    ++visited;
    if (!visit(c)) break;
    const double crossing = *std::min_element(next.begin(), next.end());
    if (crossing >= 1.0) break;
    for (int axis = 0; axis < 3; ++axis) {
      if (next[axis] <= crossing + 1e-12) {
        *coordinates[axis] += step[axis];
        next[axis] += increment[axis];
      }
    }
  }
  return visited;
}
}  // namespace surf_drone
