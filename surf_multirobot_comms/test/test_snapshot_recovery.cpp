#include <gtest/gtest.h>
#include "surf_multirobot_comms/snapshot_recovery.hpp"

using surf::comms::SnapshotRecovery;
using Delta = SnapshotRecovery::Delta;

static Delta record(uint64_t version, int x, uint8_t state)
{
  Delta d;
  d.version = version; d.resolution = 0.05F; d.header.frame_id = "map";
  d.x = {x}; d.y = {0}; d.z = {0}; d.state = {state};
  d.observation_time_ns = {version * 100};
  return d;
}

TEST(SnapshotRecovery, DelayedSnapshotPreservesAlreadyAppliedChanges)
{
  SnapshotRecovery recovery;
  recovery.complete(147);
  recovery.remember(record(160, 1, Delta::STATE_FREE));
  recovery.remember(record(266, 2, Delta::STATE_DELETE));
  auto snapshot = record(159, 1, Delta::STATE_OCCUPIED_STATIC);
  snapshot.full_refresh = true;
  recovery.begin(159);
  ASSERT_TRUE(recovery.rebase(snapshot));
  EXPECT_EQ(snapshot.version, 266U);
  EXPECT_EQ(snapshot.state, (std::vector<uint8_t>{
    Delta::STATE_OCCUPIED_STATIC, Delta::STATE_FREE, Delta::STATE_DELETE}));
  EXPECT_EQ(snapshot.observation_time_ns.back(), 26600U);
  recovery.complete(159);
  // The wire ACK refers to 159; a subsequent full snapshot 200 still needs delta 266.
  auto next = record(200, 2, Delta::STATE_OCCUPIED_STATIC);
  recovery.begin(200);
  ASSERT_TRUE(recovery.rebase(next));
  EXPECT_EQ(next.version, 266U);
  EXPECT_EQ(next.state.back(), Delta::STATE_DELETE);
}

TEST(SnapshotRecovery, NewFullGenerationSupersedesOldFullOnly)
{
  SnapshotRecovery recovery;
  recovery.begin(159);
  recovery.remember(record(266, 1, Delta::STATE_FREE));
  EXPECT_TRUE(recovery.rejection(159).empty());
  recovery.begin(270);
  EXPECT_EQ(recovery.rejection(159), "superseded full refresh");
  EXPECT_TRUE(recovery.rejection(270).empty());
}

TEST(SnapshotRecovery, HistoryEvictionRejectsRollbackButAllowsFreshSnapshot)
{
  SnapshotRecovery recovery(1);
  recovery.remember(record(266, 1, Delta::STATE_DELETE));
  EXPECT_LE(recovery.history_bytes(), 1U);
  auto old = record(159, 1, Delta::STATE_OCCUPIED_STATIC);
  EXPECT_FALSE(recovery.rebase(old));
  auto fresh = record(267, 2, Delta::STATE_OCCUPIED_STATIC);
  EXPECT_TRUE(recovery.rebase(fresh));
}

TEST(SnapshotRecovery, IncompatibleFrameOrResolutionDoesNotPartiallyRebase)
{
  SnapshotRecovery recovery;
  auto newer = record(20, 1, Delta::STATE_FREE);
  newer.resolution = 0.1F;
  recovery.remember(newer);
  auto snapshot = record(10, 1, Delta::STATE_OCCUPIED_STATIC);
  EXPECT_FALSE(recovery.rebase(snapshot));
  EXPECT_EQ(snapshot.x.size(), 1U);
  EXPECT_EQ(snapshot.version, 10U);
}
