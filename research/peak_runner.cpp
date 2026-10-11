#include "sequential_greedy.hpp"
#include "../src/traffic.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
using namespace egcs2017;
int main(int argc,char** argv){try{
 if(argc!=2)throw std::runtime_error("usage: peak_runner CSV");std::ofstream out(argv[1]);if(!out)throw std::runtime_error("cannot write CSV");out<<std::setprecision(17);
 out<<"profile,label,cars,rate,method,n,unserved,unfinished,awt,p95_wait,att,ajt,sim_end,drain_time\n";
 for(const std::string profile:{"up","downt"})for(int label=92000;label<92030;++label)for(int C:{2,4,6})for(int rate:{10,20,30}){
  Building b;b.floors=10;const auto spec=TrafficSpec::named(profile,10,rate,30,3600);const unsigned long seed=static_cast<unsigned long>(label)*1000003UL+10*101+C*7+rate*10;
  const auto passengers=profile_traffic(spec,seed);
  for(const std::string method:{"sequential_greedy","paper2017_ls","paper2017_ls_lk"}){Simulation sim(b,C,make_extended_dispatcher(method),passengers);sim.run();auto m=sim.metrics();
   out<<profile<<','<<label<<','<<C<<','<<rate<<','<<method<<','<<m.n<<','<<m.unserved<<','<<m.unfinished<<','<<m.awt<<','<<m.p95_wait<<','<<m.att<<','<<m.ajt<<','<<m.sim_end<<','<<m.drain_time<<'\n';out.flush();}
  if(label%5==0&&C==6&&rate==30)std::cout<<profile<<" label "<<label<<" completed\n"<<std::flush;
 }return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}}
