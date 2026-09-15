#include <gtest/gtest.h>
#include "surf_drone/information_allocation.hpp"
#include "surf_drone/information_debt.hpp"
#include "surf_multirobot_comms/temporal_voxels.hpp"
using namespace surf_drone;
TEST(Allocation, FeedbackAndSharedProjection) {
  InformationAllocationController c;
  EXPECT_DOUBLE_EQ(c.allocate({0, 0}, 100).allocated.norm(), 0);
  EXPECT_GT(c.allocate({10, 0}, 1e6).allocated[0], c.allocate({1, 0}, 1e6).allocated[0]);
  EXPECT_GT(c.allocate({0, 10}, 1e6).allocated[1], c.allocate({0, 1}, 1e6).allocated[1]);
  auto p = c.project({300, 100}, 100);
  EXPECT_NEAR(p.allocated[0], 75, 1e-8); EXPECT_NEAR(p.allocated.sum(), 100, 1e-8);
  EXPECT_EQ(c.project({1, 2}, 0).allocated.norm(), 0);
  EXPECT_EQ(c.project({-1, -2}, 100).allocated.norm(), 0);
  EXPECT_TRUE(c.project({INFINITY, NAN}, 100).allocated.allFinite());
}
TEST(Allocation, AtomicMatrixValidation) {
  InformationAllocationController c; std::string error;
  auto model = c.model(); const auto old = c.gain(); const auto rev = c.revision();
  model.r(0, 0) = -1; EXPECT_FALSE(c.update(model, error));
  EXPECT_EQ(c.revision(), rev); EXPECT_TRUE(c.gain().isApprox(old));
  model = c.model(); model.q *= 2; EXPECT_TRUE(c.update(model, error));
  EXPECT_EQ(c.revision(), rev + 1); EXPECT_FALSE(c.gain().isApprox(old));
  EXPECT_TRUE(c.update(model, error)); EXPECT_EQ(c.revision(), rev + 1);
  model.a(0,0) = NAN; EXPECT_FALSE(c.update(model, error));
  model = c.model(); model.q(0,1) = 1; EXPECT_FALSE(c.update(model, error));
}
TEST(Allocation, OutdoorCostsFavorDeltaWithoutSlowingBacklogTail) {
  InformationAllocationController tuned, baseline;
  auto old_model = baseline.model(); old_model.q.setIdentity();
  std::string error;
  ASSERT_TRUE(baseline.update(old_model, error));
  ASSERT_GT(tuned.revision(), 0U);
  // Representative outdoor imbalance: equal gains gave fresh debt about 1%.
  const auto before = baseline.allocate({10000, 1000000}, 10000);
  const auto after = tuned.allocate({10000, 1000000}, 10000);
  EXPECT_GT(after.allocated[0], 6 * before.allocated[0]);
  EXPECT_NEAR(after.allocated.sum(), 10000, 1e-8);
  EXPECT_NEAR(tuned.allocate({0, 2}, 10000).allocated[1],
    baseline.allocate({0, 2}, 10000).allocated[1], 1e-5);
  EXPECT_DOUBLE_EQ(tuned.allocate({10000, 1000000}, 0).allocated.norm(), 0);
}

TEST(Debt, OutdoorDeliveryWindowSurvivesSlowCycleAndDelayedAck) {
  InformationDebt debt;
  const Coord coord{1, 0, 0};
  debt.observe(coord, 1, 1, 0);
  std::array<PriorityWeights, 2> weights{};
  auto snapshot = debt.cycle(1.01, 1.01, 1.5, 4., weights, tf2::Vector3(0,0,0), .05, false);
  ASSERT_EQ(snapshot.candidates[0].size(), 1U);
  debt.entries.at(coord).packet = 7;
  debt.entries.at(coord).sent = 1.01;
  debt.advance(3.7, 1.5, 4.); // 2.69 s ACK delay must retain packet identity.
  EXPECT_EQ(debt.entries.at(coord).packet, 7U);
  debt.ack(7, {coord});
  EXPECT_FALSE(debt.entries.at(coord).pending);
  // Undelivered updates still defer; lost attempts still eventually time out.
  debt.observe(coord, 2, 2, 4.);
  debt.advance(5.6, 1.5, 4.);
  EXPECT_EQ(debt.entries.at(coord).stream, 1);
  debt.entries.at(coord).packet = 8;
  debt.entries.at(coord).sent = 5.6;
  debt.advance(9.7, 1.5, 4.);
  EXPECT_EQ(debt.entries.at(coord).packet, 0U);
  EXPECT_TRUE(debt.entries.at(coord).pending);
}
TEST(Debt, PriorityLifecycleAndSupersession) {
  InformationDebt d; const Coord a{1,0,0}, b{2,0,0};
  EXPECT_TRUE(d.observe(a, 1, 100, 0)); EXPECT_FALSE(d.observe(a, 2, 99, 0));
  EXPECT_TRUE(d.observe(a, 2, 105, 0)); EXPECT_TRUE(d.observe(b, 4, 90, 0));
  std::array<PriorityWeights,2> w{}; w[0].destructive = 10;
  auto debt = d.score(1, w, tf2::Vector3(0,0,0), 1);
  EXPECT_DOUBLE_EQ(debt[0], d.entries[a].priority + d.entries[b].priority);
  EXPECT_EQ(d.candidates(0).front()->coord, b);
  EXPECT_EQ(d.entries.size(), 2U); EXPECT_EQ(d.superseded_count, 1U);
  d.entries[a].packet = 7; d.entries[a].sent = 1;
  d.advance(1, .1, 2); EXPECT_EQ(d.entries[b].stream, 1); EXPECT_EQ(d.entries[a].stream, 0);
  d.ack(7); EXPECT_FALSE(d.entries[a].pending); EXPECT_TRUE(d.entries[b].pending);
  d.recover(3); EXPECT_TRUE(d.entries[a].pending); EXPECT_EQ(d.entries[a].stream, 1);
  EXPECT_TRUE(d.observe(a, 4, 110, 3)); d.ack(7); EXPECT_TRUE(d.entries[a].pending);
}
TEST(Priority, NormalizedFactors) {
  PriorityWeights w; auto p = information_priority(w,0,0,1);
  EXPECT_GT(information_priority(w,10,0,1), p); EXPECT_LT(information_priority(w,0,10,1), p);
  EXPECT_GT(information_priority(w,0,0,2), p);
  EXPECT_GT(p, information_priority(w,0,0,3)); // Obstacle evidence precedes new free space.
  EXPECT_GT(information_priority(w,0,0,4), information_priority(w,0,0,5));
  EXPECT_DOUBLE_EQ(information_priority(w,100,0,1), information_priority(w,10,0,1));
}
TEST(Temporal, ReorderedBacklogRetriesAndRetiredEpochs) {
  surf::comms::TemporalVoxels t; surf_multirobot_msgs::msg::VoxelDelta d;
  d.map_epoch = 1; d.resolution = 1; d.header.frame_id = "map";
  auto apply = [&](int x, uint64_t stamp, uint8_t state) {
    d.x = {x}; d.y = {0}; d.z = {0}; d.state = {state}; d.observation_time_ns = {stamp};
    EXPECT_TRUE(t.filter(d)); return d.x.size();
  };
  EXPECT_EQ(apply(1,105,3),1U); EXPECT_EQ(apply(1,100,1),0U);
  EXPECT_EQ(apply(2,90,1),1U); EXPECT_EQ(apply(1,105,3),0U);
  EXPECT_EQ(apply(1,110,4),1U); EXPECT_EQ(apply(1,106,1),0U);
  d.map_epoch = 2; EXPECT_EQ(apply(1,1,1),1U); d.map_epoch = 1; EXPECT_FALSE(t.filter(d));
}
TEST(Debt, EventBalanceAndRawCountsAreDistinct) {
  InformationDebt d; std::array<PriorityWeights,2> w{};
  d.observe({1,0,0},2,1,0); d.observe({2,0,0},3,2,0);
  auto verify = [&](double time) {
    auto x=d.score(time,w,tf2::Vector3(0,0,0),1);
    for(int i=0;i<2;++i) EXPECT_NEAR(x[i],d.disturbance[i]-d.acknowledged[i],1e-10);
    return x;
  };
  EXPECT_GT(verify(.1)[0],double(d.entries.size()));
  d.advance(1,.2,2);verify(1);
  d.entries[{1,0,0}].packet=9;d.ack(9);verify(2);
  d.recover(3);verify(3);d.observe({2,0,0},4,3,3);verify(4);
  d.exclude({1,0,0});verify(5);d.ack(0);EXPECT_TRUE(d.entries.begin()->second.pending);
}
TEST(Allocation, SingularAndUnstabilizableModelsRetainGain) {
  InformationAllocationController c;std::string error;auto m=c.model();auto revision=c.revision();
  m.r.setZero();EXPECT_FALSE(c.update(m,error));m=c.model();m.b.setZero();EXPECT_FALSE(c.update(m,error));
  EXPECT_EQ(c.revision(),revision);
  m=c.model();m.b*=1e300;EXPECT_FALSE(c.update(m,error));
  EXPECT_EQ(c.revision(),revision);
  m=c.model();m.q(0,0)=-1;EXPECT_FALSE(c.update(m,error));
}
TEST(Allocation, FeasibleForManyDebtAndCapacityPairs) {
  InformationAllocationController c;
  for(int i=0;i<1000;++i) {
    double capacity=i%31;
    auto p=c.project({double(i*i),double((1000-i)*(1000-i))},capacity);
    EXPECT_GE(p.allocated.minCoeff(),0);EXPECT_LE(p.allocated.sum(),capacity+1e-10);
  }
}

TEST(InformationDebtPerformance, BoundedSelectionMatchesFullOrderAndIndexedAck)
{
  surf_drone::InformationDebt debt;
  for (int i = 0; i < 60000; ++i) {
    debt.observe({i, i % 7, 0}, 1, 100 + i, 0, 1 + (i % 101));
  }
  auto all = debt.candidates(0);
  auto selected = debt.candidates(0, 2400);
  ASSERT_EQ(selected.size(), 2400U);
  for (std::size_t i = 0; i < selected.size(); ++i) EXPECT_EQ(selected[i], all[i]);
  const auto coord = selected[0]->coord;
  const auto second = selected[1]->coord;
  selected[0]->packet = selected[1]->packet = 42;
  debt.observe(coord, 3, 100000, 1); // New observation invalidates old packet association.
  debt.ack(42, {coord, second});
  EXPECT_TRUE(debt.entries.at(coord).pending);
  EXPECT_FALSE(debt.entries.at(second).pending);
  EXPECT_TRUE(debt.candidates(0, 0).empty());
}

TEST(Debt, ActiveIndexExcludesResolvedRecoveryHistory)
{
  InformationDebt debt;
  for (int i = 0; i < 10000; ++i) debt.observe({i, 0, 0}, 1, i + 1, 0);
  for (int i = 0; i < 9000; ++i) {
    auto & entry = debt.entries.at({i, 0, 0});
    entry.packet = 1;
  }
  std::vector<Coord> acknowledged;
  for (int i = 0; i < 9000; ++i) acknowledged.push_back({i, 0, 0});
  debt.ack(1, acknowledged);
  EXPECT_EQ(debt.entries.size(), 10000U);
  EXPECT_EQ(debt.active.size(), 1000U);
  EXPECT_EQ(debt.candidates(0).size(), 1000U);
  debt.recover(2);
  EXPECT_EQ(debt.active.size(), debt.entries.size());
}

TEST(Priority, StarvationReservePrecedesHigherPriorityFreshDebt)
{
  InformationDebt debt;
  const Coord old_low{1, 0, 0};
  debt.observe(old_low, 1, 1, 0, 1);
  for (int i = 0; i < 20; ++i) debt.observe({100 + i, 0, 0}, 2, 100 + i, 95, 100);
  auto selected = debt.candidates(0, 10, 100, 30, 2);
  ASSERT_EQ(selected.size(), 10U);
  EXPECT_EQ(selected.front()->coord, old_low);
  // Without a reserve, normal priority order remains unchanged.
  EXPECT_FALSE(debt.candidates(0, 10).front()->coord == old_low);
}

TEST(Debt, FusedCycleTraversesAndAggregatesOnce)
{
  InformationDebt debt;
  const Coord deferred{1, 0, 0};
  const Coord inflight{2, 0, 0};
  const Coord timed_out{3, 0, 0};
  debt.observe(deferred, 1, 1000000000, 0);
  debt.observe(inflight, 2, 2000000000, 9);
  debt.observe(timed_out, 3, 3000000000, 0);
  debt.entries.at(inflight).packet = 7;
  debt.entries.at(inflight).sent = 9;
  debt.entries.at(timed_out).packet = 8;
  debt.entries.at(timed_out).sent = 0;
  std::array<PriorityWeights, 2> weights{};

  auto snapshot = debt.cycle(
    10, 12, 0.2, 2, weights, tf2::Vector3(0, 0, 0), 1, false);

  EXPECT_EQ(debt.entries.at(deferred).stream, 1);
  EXPECT_EQ(debt.entries.at(timed_out).packet, 0U);
  EXPECT_EQ(snapshot.pending_count[0], 1U);
  EXPECT_EQ(snapshot.pending_count[1], 2U);
  EXPECT_EQ(snapshot.inflight_count[0], 1U);
  EXPECT_EQ(snapshot.candidates[0].size(), 0U);
  EXPECT_EQ(snapshot.candidates[1].size(), 2U);
  EXPECT_EQ(snapshot.pending_age_bucket_count[0], 1U);
  EXPECT_EQ(snapshot.pending_age_bucket_count[5], 2U);
  EXPECT_EQ(snapshot.pending_age_bucket_count[6], 0U);
  EXPECT_DOUBLE_EQ(snapshot.debt[0], debt.entries.at(inflight).priority);
  EXPECT_DOUBLE_EQ(
    snapshot.debt[1], debt.entries.at(deferred).priority + debt.entries.at(timed_out).priority);
  EXPECT_EQ(snapshot.ack_lag_samples, 0U);
  EXPECT_DOUBLE_EQ(snapshot.oldest_pending_age, 10);
  EXPECT_DOUBLE_EQ(snapshot.oldest_observation_age_s, 11);
  EXPECT_DOUBLE_EQ(snapshot.newest_observation_age_s, 9);
}

TEST(Debt, BoundedCycleConservesGlobalDebtAndVisitsHistory)
{
  InformationDebt d;
  constexpr int count=30000;
  for (int i=0; i<count; ++i) d.observe({i,0,0},1,i+1,0);
  std::array<PriorityWeights,2> w{};
  std::unordered_set<Coord,CoordHash> seen;
  for (int cycle=0; cycle<300; ++cycle) {
    auto s=d.bounded_cycle(2,2,1.5,4,w,tf2::Vector3(0,0,0),.05,false,256);
    EXPECT_LE(s.scored_entries,256U);
    EXPECT_EQ(s.pending_count[0]+s.pending_count[1],count);
    EXPECT_TRUE(s.sampled_metrics);
    for (const auto & stream:s.candidates) for (auto * e:stream) seen.insert(e->coord);
    if (cycle==0) {
      // The state must include the unvisited majority, not just the sample.
      EXPECT_GT(s.debt[0]+s.debt[1],count-1);
    }
  }
  EXPECT_EQ(seen.size(),count);
  std::array<double,2> exact{};
  for (auto & [c,e]:d.entries) { (void)c; exact[e.stream]+=e.priority; }
  auto s=d.bounded_cycle(2,2,1.5,4,w,tf2::Vector3(0,0,0),.05,false,0);
  for(int i=0;i<2;++i) EXPECT_NEAR(s.debt[i],exact[i],1e-6);
}

TEST(Debt, BlockCompletionRequiresCurrentRevisionAckAndRecoveryIsBounded)
{
  InformationDebt d;
  d.observe({-1,0,0},1,1,0);
  d.observe({-2,0,0},3,1,0);
  d.observe({32,0,0},1,1,0);
  auto progress=d.region_progress();
  EXPECT_EQ(progress.coord,(Coord{-1,0,0})); EXPECT_EQ(progress.pending,2U);
  d.mark_sent(d.entries.at({-1,0,0}),1,0);
  d.mark_sent(d.entries.at({-2,0,0}),1,0);
  d.observe({-1,0,0},5,2,1); // Old ACK cannot complete a superseding UNKNOWN.
  d.ack(1,{{-1,0,0},{-2,0,0}});
  EXPECT_EQ(d.region_progress().pending,1U);
  EXPECT_EQ(d.region_progress().completed,0U);
  d.mark_sent(d.entries.at({-1,0,0}),2,1);
  d.ack(2,{{-1,0,0}});
  EXPECT_EQ(d.region_progress().completed,1U);
  EXPECT_EQ(d.region_progress().coord,(Coord{2,0,0}));
  d.request_recovery();
  EXPECT_EQ(d.recovery_slice(3,1),1U);
  EXPECT_EQ(d.recovery_remaining(),2U);
  while(d.recovery_remaining()) d.recovery_slice(3,1);
  EXPECT_EQ(d.active.size(),3U);
  std::array<PriorityWeights,2> w{};
  auto s=d.bounded_cycle(3,3,1.5,4,w,tf2::Vector3(0,0,0),.05,false,32);
  EXPECT_EQ(s.pending_count[0]+s.pending_count[1],3U);
  EXPECT_EQ(s.inflight_count[0]+s.inflight_count[1],0U);
  EXPECT_EQ(d.entries.at({-1,0,0}).state,5);
}

TEST(Debt, RegionPreferenceAndTimeoutDoNotDiscardOtherRegions)
{
  InformationDebt d;
  d.observe({0,0,0},3,1,0,1);
  d.observe({32,0,0},1,1,0,10);
  d.advance(2,1.5,4);
  auto candidates=d.candidates(1);
  EXPECT_EQ(candidates.front()->coord,(Coord{32,0,0}));
  d.prefer_region(candidates);
  EXPECT_EQ(candidates.front()->coord,(Coord{0,0,0}));
  auto & e=d.entries.at({0,0,0});
  d.mark_sent(e,9,2); d.timeout_packet(9,{{0,0,0}});
  EXPECT_TRUE(e.pending); EXPECT_EQ(e.packet,0U);
  EXPECT_EQ(d.region_progress().pending,1U);
  std::array<PriorityWeights,2> w{};
  d.bounded_cycle(2,2,1.5,4,w,tf2::Vector3(0,0,0),.05,false,32);
  d.bounded_cycle(13,13,1.5,4,w,tf2::Vector3(0,0,0),.05,false,32);
  EXPECT_EQ(d.region_progress().coord,(Coord{2,0,0})); // Bounded region lease.
}

TEST(Debt, BacklogSelectionGroupsProximityComponents)
{
  InformationDebt d;
  // Component A has lower scalar priority but touches acknowledged evidence.
  d.observe({0,0,0},3,1,0,1);
  d.observe({1,0,0},3,2,0,1);
  d.observe({2,0,0},1,3,0,1);
  // A distant component remains separate at radius one.
  d.observe({5,1,0},1,4,0,100);
  d.observe({6,1,0},1,5,0,100);
  d.advance(2,1.5,4);
  d.mark_sent(d.entries.at({0,0,0}),7,2);
  d.ack(7,{{0,0,0}});

  auto seeds=d.candidates(1);
  auto selected=d.connected_candidates(1,seeds,20,100);
  ASSERT_TRUE(selected.valid);
  ASSERT_EQ(selected.seed,(Coord{1,0,0}));
  std::unordered_set<Coord,CoordHash> coordinates;
  for(auto * entry:selected.entries) coordinates.insert(entry->coord);
  EXPECT_EQ(coordinates,
    (std::unordered_set<Coord,CoordHash>{{1,0,0},{2,0,0},{5,1,0},{6,1,0}}));
  ASSERT_EQ(selected.component_ends.size(),2U);
  EXPECT_EQ(selected.component_ends[0],2U);
  // Components share a cycle in component order; this preserves throughput
  // when individual components are smaller than packet overhead.
  EXPECT_EQ(selected.component_ends[1],4U);
}

TEST(Debt, ProximityRadiusClustersSparseQuantizedSamples)
{
  InformationDebt d;
  d.observe({0,0,0},3,1,0);
  d.observe({2,0,0},3,2,0);
  d.observe({4,0,0},1,3,0);
  d.advance(2,1.5,4);
  auto radius_one=d.connected_candidates(1,d.candidates(1),20,100,1);
  EXPECT_EQ(radius_one.component_ends.size(),3U);
  InformationDebt d2;
  d2.observe({0,0,0},3,1,0);d2.observe({2,0,0},3,2,0);d2.observe({4,0,0},1,3,0);
  d2.advance(2,1.5,4);
  auto radius_two=d2.connected_candidates(1,d2.candidates(1),20,100,2);
  EXPECT_EQ(radius_two.component_ends.size(),1U);
  EXPECT_EQ(radius_two.entries.size(),3U);
}

TEST(Debt, ConnectedSelectionIsBoundedAndUnknownBreaksTheGraph)
{
  InformationDebt d;
  for(int x=0;x<100;++x) d.observe({x,0,0},3,x+1,0);
  d.observe({5,0,0},5,1000,1);  // Withdraw a cell in the chain.
  d.advance(2,1.5,4);
  auto selected=d.connected_candidates(1,d.candidates(1),100,10);
  EXPECT_LE(selected.visited,10U);
  EXPECT_TRUE(selected.truncated);
  ASSERT_FALSE(selected.component_ends.empty());
  for(std::size_t i=0;i<selected.component_ends.front();++i) {
    auto * entry=selected.entries[i];
    EXPECT_LT(entry->coord.x,5);
    EXPECT_NE(entry->state,5);
  }
}

TEST(Debt, ConnectedSelectionFillsLimitFromLargeComponent)
{
  InformationDebt d;
  for(int x=0;x<10000;++x) d.observe({x,0,0},3,x+1,0);
  d.advance(2,1.5,4);
  auto selected=d.connected_candidates(1,d.candidates(1),2400,8192,2);
  EXPECT_EQ(selected.entries.size(),2400U);
  ASSERT_EQ(selected.component_ends.size(),1U);
  EXPECT_EQ(selected.component_ends.front(),2400U);
  EXPECT_LE(selected.visited,8192U);
  EXPECT_TRUE(selected.truncated);
}

TEST(Temporal, ExpiryIsUnknownAndLateFreeCannotResurrectIt)
{
  surf::comms::TemporalVoxels t;
  surf_multirobot_msgs::msg::VoxelDelta d;
  d.map_epoch=1; d.resolution=.05; d.header.frame_id="map";
  auto apply=[&](uint64_t stamp,uint8_t state) {
    d.x={1};d.y={0};d.z={0};d.state={state};d.observation_time_ns={stamp};
    EXPECT_TRUE(t.filter(d)); return d.x.size();
  };
  EXPECT_EQ(apply(10,d.STATE_OCCUPIED_DYNAMIC),1U);
  EXPECT_EQ(apply(20,d.STATE_UNKNOWN),1U);
  EXPECT_EQ(apply(15,d.STATE_FREE),0U);
  auto snapshot=t.snapshot(d);
  ASSERT_EQ(snapshot.state.size(),1U);EXPECT_EQ(snapshot.state[0],d.STATE_UNKNOWN);
  EXPECT_EQ(apply(21,d.STATE_FREE),1U);
}

TEST(Debt, SpatialChangeFavorsTwentyAdjacentUpdatesOverSingleton)
{
  InformationDebt d;
  d.observe({100,0,0},1,1,0);
  for (int x=0;x<5;++x) for (int y=0;y<4;++y)
    d.observe({x,y,0},1,2+x*4+y,0);
  auto candidates=d.candidates(0);
  const auto debt=d.disturbance;
  d.score_spatial_change(candidates);
  auto ranked=InformationDebt::rank_candidates(candidates);
  EXPECT_LT(ranked.front()->coord.x,5);
  EXPECT_DOUBLE_EQ(d.entries.at({100,0,0}).spatial_bonus,0);
  EXPECT_EQ(d.disturbance,debt);
  // Even a one-voxel selection honors an old isolated update.
  d.entries.at({100,0,0}).created=-40;
  ranked=InformationDebt::rank_candidates(candidates,1,0,30,1);
  ASSERT_EQ(ranked.size(),1U);
  EXPECT_EQ(ranked.front()->coord,(Coord{100,0,0}));
}

TEST(Debt, SpatialChangeIgnoresDeliveredInflightAndUnknownNeighbors)
{
  InformationDebt d;
  d.observe({0,0,0},1,1,0);
  d.observe({1,0,0},1,2,0);
  d.observe({0,1,0},1,3,0);
  d.observe({0,0,1},5,4,0);
  d.mark_sent(d.entries.at({1,0,0}),1,0);
  d.ack(1);
  d.mark_sent(d.entries.at({0,1,0}),2,0);
  auto candidates=d.candidates(0);
  d.score_spatial_change(candidates);
  EXPECT_DOUBLE_EQ(d.entries.at({0,0,0}).spatial_bonus,0);
  // A small change in an already delivered map is still selected.
  d.observe({1,0,0},3,5,1);
  candidates=d.candidates(0);
  d.score_spatial_change(candidates);
  EXPECT_GT(d.entries.at({0,0,0}).spatial_bonus,0);
  d.score_spatial_change(candidates,0);
  EXPECT_DOUBLE_EQ(d.entries.at({0,0,0}).spatial_bonus,0);
}

TEST(Debt, ConnectedFocusReconsidersHotspotsAndOldSingletons)
{
  InformationDebt d;
  for (int x=0;x<10;++x) d.observe({x,0,0},1,x+1,0);
  d.advance(2,1,4);
  auto first=d.connected_candidates(1,d.candidates(1),1,100);
  ASSERT_EQ(first.entries.size(),1U);
  EXPECT_TRUE(first.truncated);
  for (int x=100;x<105;++x) for (int y=0;y<4;++y)
    d.observe({x,y,0},1,100+x*4+y,2);
  d.advance(4,1,4);
  auto candidates=d.candidates(1);
  d.score_spatial_change(candidates);
  auto ranked=InformationDebt::rank_candidates(candidates);
  auto hot=d.connected_candidates(1,ranked,1,100);
  ASSERT_EQ(hot.entries.size(),1U);
  EXPECT_GE(hot.entries.front()->coord.x,100);
  d.observe({200,0,0},1,1000,-40);
  d.advance(4,1,4);
  candidates=d.candidates(1);
  d.score_spatial_change(candidates);
  ranked=InformationDebt::rank_candidates(candidates,100,4,30,1);
  auto old=d.connected_candidates(1,ranked,1,100);
  ASSERT_EQ(old.entries.size(),1U);
  EXPECT_EQ(old.entries.front()->coord,(Coord{200,0,0}));
}

TEST(Debt, StableMapStillDeliversSingleChangedVoxel)
{
  InformationDebt d;
  for (int x=14;x<19;++x) for (int y=0;y<4;++y) {
    const Coord c{x,y,0};
    d.observe(c,1,1,0);
    d.mark_sent(d.entries.at(c),1,0);
  }
  d.ack(1);
  d.observe({16,1,0},3,2,1);
  d.advance(3,1,4);
  auto candidates=d.candidates(1);
  d.score_spatial_change(candidates);
  ASSERT_EQ(candidates.size(),1U);
  EXPECT_DOUBLE_EQ(candidates.front()->spatial_bonus,0);
  auto selected=d.connected_candidates(1,candidates,1,100);
  ASSERT_EQ(selected.entries.size(),1U);
  EXPECT_EQ(selected.entries.front()->coord,(Coord{16,1,0}));
  // Adjacent changes contribute across the 16-voxel region boundary.
  d.observe({15,1,0},3,2,3);
  d.score_spatial_change(candidates);
  EXPECT_GT(candidates.front()->spatial_bonus,0);
}
