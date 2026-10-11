#include <stdexcept>
#include <iostream>
#include <random>
#include <functional>
#include <cassert>
#include "batch_partition.hpp"
using namespace egcs2017;
int main(){std::mt19937 gen(921);int checked=0;
 for(int n=1;n<=7;++n)for(int C=2;C<=4;++C)for(int repeat=0;repeat<15;++repeat){
  std::vector<std::vector<PartitionCost>> table(C,std::vector<PartitionCost>(1<<n));
  for(auto& car:table)for(auto& v:car){v.journey=gen()%200;v.waiting=gen()%100;v.valid=gen()%5!=0;}
  double budget=gen()%250;bool feasible=false;double best=1e100;std::vector<int> masks(C);
  std::function<void(int)> recurse=[&](int i){
   if(i==n){double w=0,j=0;for(int c=0;c<C;++c){auto v=table[c][masks[c]];if(!v.valid)return;w+=v.waiting;j+=v.journey;}if(w<=budget){feasible=true;best=std::min(best,j);}return;}
   for(int c=0;c<C;++c){masks[c]|=1<<i;recurse(i+1);masks[c]^=1<<i;}
  };recurse(0);
  auto result=partition_pareto(table,n,budget,10000000);
  assert(result.complete&&result.feasible==feasible);
  if(feasible){assert(result.journey==best);int unionmask=0;double w=0,j=0;for(int c=0;c<C;++c){assert(!(unionmask&result.masks[c]));unionmask|=result.masks[c];auto v=table[c][result.masks[c]];assert(v.valid);w+=v.waiting;j+=v.journey;}assert(unionmask==(1<<n)-1&&w==result.waiting&&j==result.journey&&w<=budget);}
  auto capped=partition_pareto(table,n,budget,0);if(capped.transitions)assert(!capped.complete);
  ++checked;
 }
 std::cout<<checked<<" exhaustive comparisons passed\n";
}
