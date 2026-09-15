#include "surf_drone/information_debt.hpp"
#include <chrono>
#include <iostream>
#include <vector>
using namespace surf_drone;
int main() {
  std::array<PriorityWeights,2> weights{};
  for(int size : {10000,100000,1000000}) {
    InformationDebt d;
    for(int i=0;i<size;++i) d.observe({i%1000,(i/1000)%1000,i/1000000},1,i+1,0);
    auto start=std::chrono::steady_clock::now();
    auto full=d.cycle(2,2,1.5,4,weights,tf2::Vector3(0,0,0),.05,false);
    const double full_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
    std::vector<double> durations;
    for(int i=0;i<100;++i) {
      start=std::chrono::steady_clock::now();
      auto bounded=d.bounded_cycle(2,2,1.5,4,weights,tf2::Vector3(0,0,0),.05,false,4096);
      for(auto & c:bounded.candidates) c=InformationDebt::rank_candidates(std::move(c),4096);
      d.prefer_region(bounded.candidates[1]);
      durations.push_back(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count());
      if(bounded.scored_entries>4096 || bounded.pending_count[0]+bounded.pending_count[1]!=static_cast<unsigned>(size) ||
        std::abs(bounded.debt[1]-full.debt[1])>1e-3) return 1;
    }
    std::sort(durations.begin(),durations.end());
    std::cout<<size<<" entries: full traversal "<<full_ms<<" ms; bounded scoring+ranking median "<<durations[50]<<" ms p95 "<<durations[95]<<" ms\n"<<std::flush;
  }
}
