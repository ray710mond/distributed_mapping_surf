// Synthetic lookup benchmark; not an end-to-end sender or Jetson measurement.
#include <chrono>
#include <iostream>
#include "surf_drone/communication_state.hpp"

int main()
{
  using namespace surf_drone;
  std::cout << "cells,full_scan_ms,indexed_query_ms,matches\n";
  for (int count : {100000, 500000, 1000000}) {
    SpatialMap<int> cells;
    for (int i = 0; i < count; ++i) {
      cells[{i % 1000 - 500, (i / 1000) % 100 - 50, i / 100000 - 5}] = i;
    }
    const Coord low{-12, -12, -12}, high{12, 12, 12};
    std::size_t full_matches = 0, indexed_matches = 0;
    const auto start = std::chrono::steady_clock::now();
    for (int repeat = 0; repeat < 100; ++repeat) {
      for (const auto & [c, value] : cells) {
        (void)value;
        if (c.x >= low.x && c.x <= high.x && c.y >= low.y && c.y <= high.y &&
          c.z >= low.z && c.z <= high.z) {++full_matches;}
      }
    }
    const auto middle = std::chrono::steady_clock::now();
    for (int repeat = 0; repeat < 100; ++repeat) {
      indexed_matches += cells.in_box(low, high).size();
    }
    const auto end = std::chrono::steady_clock::now();
    if (full_matches != indexed_matches) {return 1;}
    std::cout << count << ',' << std::chrono::duration<double, std::milli>(middle-start).count()/100
              << ',' << std::chrono::duration<double, std::milli>(end-middle).count()/100
              << ',' << indexed_matches/100 << '\n';
  }
}
