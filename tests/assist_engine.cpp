#include <cassert>
#include <iostream>
#include "sim.hpp"
using namespace egcs2017;
int main(){
 std::map<int,std::pair<HallKey,long>> observed;
 for(int n : {1,20,100}) {
  Building b;b.floors=10;std::vector<Passenger> ps;
  for(int i=0;i<n;++i){Passenger p;p.origin=6;p.dest=(i%2)?0:2;ps.push_back(p);}
  SimOptions o;o.assist_age=0;o.check_invariants=true;
  Simulation sim(b,3,make_dispatcher("paper2017_ls_lk"),ps,nullptr,1,o);sim.run();
  if(n==1) observed=sim.auxiliary_targets(); else assert(observed==sim.auxiliary_targets());
 }
 assert(!observed.empty());
 for(int f : {0,6}) for(int m : {1,13,14,40}) {
  Building b;b.floors=10;b.capacity=13;
  std::vector<Passenger> ps;for(int i=0;i<m;++i){Passenger p;p.arrival=0;p.origin=f;p.dest=f?0:8;ps.push_back(p);}
  SimOptions o;o.check_invariants=true;o.assist_age=0;o.full_load_bypass=true;
  Simulation sim(b,3,make_dispatcher("paper2017_ls_lk"),ps,nullptr,1e5,o);sim.set_max_time(1);sim.run();
  Simulation clone(sim,make_dispatcher("paper2017_ls_lk"));sim.set_max_time(1e5);clone.set_max_time(1e5);sim.run();clone.run();
  assert(!sim.metrics().unserved&&!sim.metrics().unfinished);
  for(int i=0;i<m;++i){assert(sim.passengers()[i].boarded==clone.passengers()[i].boarded);assert(sim.passengers()[i].alighted==clone.passengers()[i].alighted);}
  std::cout<<f<<' '<<m<<' '<<sim.metrics().assist_issued<<' '<<sim.metrics().awt<<'\n';
 }
}
