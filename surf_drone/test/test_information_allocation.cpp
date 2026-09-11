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
  EXPECT_GT(information_priority(w,0,0,2), p); EXPECT_GT(information_priority(w,0,0,4), p);
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
