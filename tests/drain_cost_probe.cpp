#include <iostream>
#include <random>
#include "sim.hpp"
#include "drain_cost.hpp"
using namespace egcs2017;
int main(){try {
 std::mt19937 gen(84192); double err=0;int cases=0,moving=0,arriving=0,closed=0,skipped_open=0;
 for(int bypass=0;bypass<2;++bypass)for(int trial=0;trial<100;++trial){
  Building b;b.floors=3+gen()%7;b.capacity=2+gen()%12;
  std::vector<Passenger> people;
  for(int i=0,n=1+gen()%35;i<n;++i){Passenger p;p.arrival=1;p.origin=gen()%b.floors;
   do{p.dest=gen()%b.floors;}while(p.dest==p.origin);people.push_back(p);}
  SimOptions opt;opt.fixed_assignment=true;opt.assigned_only_boarding=true;opt.full_load_bypass=bypass;opt.check_invariants=true;
  for(auto& p:people)opt.forced_assignment[{p.origin,p.dir()}]=0;
  Simulation s(b,1,make_dispatcher("paper2017_ls_lk"),people,nullptr,1e6,opt);
  std::map<HallKey,std::deque<int>> queues;
  for(auto& p:s.passengers())queues[{p.origin,p.dir()}].push_back(p.dest);
  CarView cv;auto result=drain_cost(b,cv,queues,bypass);s.run();
  double w=0,j=0;
  for(auto& p:s.passengers()){if(p.boarded<0||p.alighted<0)throw std::runtime_error("unserved");w+=p.boarded-1;j+=p.alighted-1;}
  err=std::max({err,std::abs(w-result.waiting),std::abs(j-result.journey)});
  if(std::abs(w-result.waiting)>1e-7||std::abs(j-result.journey)>1e-7){
   std::cerr<<"case "<<cases<<" C "<<b.capacity<<" F "<<b.floors<<" W "<<w<<" / "<<result.waiting<<" J "<<j<<" / "<<result.journey<<'\n';return 2;}
  ++cases;
  double branch=2+gen()%180;
  Simulation paused(b,1,make_dispatcher("paper2017_ls_lk"),people,nullptr,branch,opt);paused.run();
  auto snapshot=paused.view(paused.cars()[0],branch);
  if(snapshot.doors_open){++skipped_open;continue;}
  snapshot.car_calls=paused.cars()[0].car_call_counts();
  std::map<HallKey,std::deque<int>> remain;
  std::vector<int> waiting_ids,onboard_ids=paused.cars()[0].onboard;
  for(auto& [k,ids]:paused.waiting())for(int i:ids){remain[k].push_back(paused.passengers()[i].dest);waiting_ids.push_back(i);}
  auto pred=drain_cost(b,snapshot,remain,bypass);
  paused.set_max_time(1e6);paused.run();
  double rw=0,rj=0;
  for(int i:waiting_ids){auto& ps=paused.passengers()[i];rw+=ps.boarded-branch;rj+=ps.alighted-branch;}
  for(int i:onboard_ids)rj+=paused.passengers()[i].alighted-branch;
  err=std::max({err,std::abs(rw-pred.waiting),std::abs(rj-pred.journey)});
  if(std::abs(rw-pred.waiting)>1e-7||std::abs(rj-pred.journey)>1e-7){
    std::cerr<<"paused mismatch trial "<<trial<<" moving "<<snapshot.moving<<" arriving "<<snapshot.arriving<<" W "<<rw<<" / "<<pred.waiting<<" J "<<rj<<" / "<<pred.journey<<'\n';return 2;}
  moving+=snapshot.moving;arriving+=snapshot.arriving;closed+=!snapshot.moving&&!snapshot.arriving;

 }
 std::cout<<"cases "<<cases<<" maximum error "<<err<<" moving "<<moving<<" arriving "<<arriving<<" closed "<<closed<<" skipped_open "<<skipped_open<<'\n';
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}}
