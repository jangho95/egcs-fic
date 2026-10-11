#include <iostream>
#include <iomanip>
#include "sim.hpp"
using namespace egcs2017;
int main(){int C,n;while(std::cin>>C>>n){Building b;b.floors=10;b.capacity=C;std::vector<Passenger> ps;for(int i=0;i<n;++i){int f,m;std::cin>>f>>m;for(int j=0;j<m;++j){Passenger p;p.arrival=0;p.origin=f;p.dest=0;ps.push_back(p);}}SimOptions o;o.fixed_assignment=true;o.assigned_only_boarding=true;o.full_load_bypass=true;o.check_invariants=true;Simulation sim(b,1,make_dispatcher("paper2017_ls_lk"),ps,nullptr,100000,o);sim.run();auto v=sim.metrics();if(v.unserved||v.unfinished)return 2;double j=0,w=0;for(auto& p:sim.passengers()){j+=p.alighted;w+=p.boarded;}std::cout<<std::setprecision(17)<<j<<' '<<w<<'\n';}}
