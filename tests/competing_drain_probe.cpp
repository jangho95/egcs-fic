#include <iomanip>
#include <iostream>
#include <stdexcept>
#include "sim.hpp"
using namespace egcs2017;
int main() {
 try {
  int C,p,n,u,v;
  while(std::cin>>C>>p>>n>>u>>v) {
   if(C<4||p<3||p>=C||n<1||n>=p||u<1||v<1) throw std::runtime_error("invalid input");
   Building b; b.floors=6; b.capacity=C;
   const double branch=b.kin.travel(0,5)+b.doors.dwell(p,0)+1;
   double J[2]={},W[2]={};
   for(int with=0;with<2;++with) {
    std::vector<Passenger> people;
    for(int i=0;i<p;++i) { Passenger x; x.origin=5;x.dest=i<n?3:0;x.arrival=0;people.push_back(x); }
    for(int i=0;i<v;++i) { Passenger x; x.origin=2;x.dest=0;x.arrival=branch;people.push_back(x); }
    if(with) for(int i=0;i<u;++i) { Passenger x; x.origin=4;x.dest=0;x.arrival=branch;people.push_back(x); }
    SimOptions o; o.theta=0;o.fixed_assignment=true;o.assigned_only_boarding=true;
    o.full_load_bypass=true;o.check_invariants=true;
    for(int f:{5,4,2}) o.forced_assignment[{f,DOWN}]=0;
    Simulation s(b,1,make_dispatcher("paper2017_ls_lk"),people,nullptr,branch,o);
    s.run();
    if(s.cars()[0].load()!=p) throw std::runtime_error("branch load");
    auto cv=s.view(s.cars()[0],branch);
    if(!cv.moving || std::abs(b.pos(4)-cv.x)<cv.v*cv.v/(2*b.kin.a_max)) throw std::runtime_error("geometry");
    s.set_max_time(1e7);s.run();
    for(auto& x:s.passengers()) {
     if(x.alighted<0||x.boarded<0||x.car!=0) throw std::runtime_error("unserved");
     J[with]+=x.alighted-branch;
     if(x.origin!=5) W[with]+=x.boarded-branch;
    }
    if(!s.calls().empty()||!s.waiting().empty()) throw std::runtime_error("not drained");
   }
   std::cout<<std::setprecision(17)<<"{\"C\":"<<C<<",\"p\":"<<p<<",\"n\":"<<n<<",\"u\":"<<u<<",\"v\":"<<v
    <<",\"without\":"<<J[0]<<",\"with\":"<<J[1]<<",\"increment\":"<<J[1]-J[0]<<",\"waiting_increment\":"<<W[1]-W[0]<<"}\n";
  }
 } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 2;}
}
