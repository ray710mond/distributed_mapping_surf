#include <algorithm>
#include <random>
#include <unordered_set>
#include <gtest/gtest.h>
#include "surf_drone/communication_state.hpp"

using surf_drone::Coord;
using surf_drone::CoordHash;
using surf_drone::DynamicExpiry;
using surf_drone::SpatialMap;

TEST(CommunicationState, SpatialQueriesMatchFullScanAfterMutations)
{
  SpatialMap<int> cells;
  std::mt19937 generator(81);
  std::uniform_int_distribution<int> coordinate(-100, 100);
  for (int i = 0; i < 20000; ++i) {
    cells[{coordinate(generator), coordinate(generator), coordinate(generator)}] = i;
  }
  for (int trial = 0; trial < 200; ++trial) {
    Coord low{coordinate(generator), coordinate(generator), coordinate(generator)};
    Coord high{low.x + 20, low.y + 30, low.z + 10};
    std::unordered_set<Coord, CoordHash> expected;
    for (const auto & [c, value] : cells) {
      (void)value;
      if (c.x >= low.x && c.x <= high.x && c.y >= low.y && c.y <= high.y &&
        c.z >= low.z && c.z <= high.z) {expected.insert(c);}
    }
    auto candidates = cells.in_box(low, high);
    EXPECT_EQ((std::unordered_set<Coord, CoordHash>(candidates.begin(), candidates.end())),
      expected);
    for (const auto & c : candidates) {cells.erase(c);}
    EXPECT_TRUE(cells.in_box(low, high).empty());
  }
}

TEST(CommunicationState, BlockBoundariesAndIteratorErasure)
{
  SpatialMap<int> cells;
  for (int x : {-33, -32, -17, -16, -1, 0, 15, 16, 32}) {
    Coord c{x, x, x};
    cells[c] = 1;
    cells[c] = 2;
    ASSERT_EQ(cells.in_box(c, c).size(), 1U);
    cells.erase(cells.find(c));
    EXPECT_TRUE(cells.in_box(c, c).empty());
  }
  EXPECT_EQ(cells.size(), 0U);
}

TEST(CommunicationState, ExpiryMatchesStrictRetentionAndRescheduling)
{
  DynamicExpiry expiry;
  Coord a{-1, 0, 0}, b{2, 0, 0};
  expiry.schedule(a, 10 + 20 + 1);
  EXPECT_TRUE(expiry.pop_due(30).empty());
  expiry.schedule(a, 15 + 20 + 1);  // Seen again before old expiry.
  expiry.schedule(b, 31);
  EXPECT_EQ(expiry.size(), 2U);
  EXPECT_EQ(expiry.pop_due(31), (std::vector<Coord>{b}));
  EXPECT_TRUE(expiry.pop_due(35).empty());
  EXPECT_EQ(expiry.pop_due(36), (std::vector<Coord>{a}));
  EXPECT_EQ(expiry.size(), 0U);
  expiry.schedule(a, 40);
  expiry.cancel(a);  // Promotion, masking or ray clearing.
  EXPECT_TRUE(expiry.pop_due(100).empty());
}

TEST(CommunicationState, ExpiryMatchesReferenceAcrossLongRun)
{
  DynamicExpiry expiry;
  std::unordered_map<Coord, uint64_t, CoordHash> reference;
  std::mt19937 generator(7);
  for (uint64_t version = 1; version < 1000; ++version) {
    for (int i = 0; i < 20; ++i) {
      Coord c{static_cast<int32_t>(generator() % 100), 0, 0};
      if (generator() % 5 == 0) {expiry.cancel(c); reference.erase(c);}
      else {expiry.schedule(c, version + 21); reference[c] = version;}
    }
    std::unordered_set<Coord, CoordHash> expected;
    for (auto it = reference.begin(); it != reference.end();) {
      if (version - it->second > 20) {expected.insert(it->first); it = reference.erase(it);}
      else {++it;}
    }
    auto due = expiry.pop_due(version);
    EXPECT_EQ((std::unordered_set<Coord, CoordHash>(due.begin(), due.end())), expected);
    EXPECT_EQ(expiry.size(), reference.size());
  }
}

TEST(CommunicationState, RotatedMaskQueryMatchesOriginalFullScan)
{
  SpatialMap<int> cells;
  for (int x = -30; x <= 30; ++x) {
    for (int y = -30; y <= 30; ++y) {
      for (int z = -20; z <= 20; ++z) {cells[{x, y, z}] = 1;}
    }
  }
  for (int angle = 0; angle < 36; ++angle) {
    surf_drone::HumanoidMask mask;
    mask.center = tf2::Vector3(-0.137, 0.213, -0.083);
    mask.world_from_model.setRPY(angle * 0.07, angle * 0.03, angle * 0.17);
    mask.half_x = 0.55; mask.half_y = 0.53; mask.half_z = 0.6125;
    const auto bounds = mask.voxel_bounds(0.05);
    std::unordered_set<Coord, CoordHash> expected, actual;
    for (const auto & [c, value] : cells) {
      (void)value;
      if (mask.contains(c, 0.05)) {expected.insert(c);}
    }
    for (const auto & c : cells.in_box(bounds.first, bounds.second)) {
      if (mask.contains(c, 0.05)) {actual.insert(c);}
    }
    EXPECT_EQ(actual, expected);
  }
}
