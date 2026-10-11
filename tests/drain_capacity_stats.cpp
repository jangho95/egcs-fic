#include <iostream>
#include <iomanip>
#include "drain_cost.hpp"
using namespace egcs2017;
int main(){try{
 Building b;
 while(std::cin>>b.floors>>b.capacity){
  std::cin>>b.kin.floor_height>>b.kin.v_max>>b.kin.a_max>>b.kin.settle>>b.doors.open>>b.doors.close>>b.doors.transfer;
  CarView c;int bypass,n;
  std::cin>>c.x>>c.v>>c.dir>>c.moving>>c.arriving>>c.committed>>c.release>>c.doors_open>>c.load>>bypass>>n;
  for(int i=0;i<n;++i){int f,count;std::cin>>f>>count;c.car_calls[f]=count;}
  std::map<HallKey,std::deque<int>> q;std::cin>>n;
  for(int i=0;i<n;++i){int f,d,count;std::cin>>f>>d>>count;for(int j=0;j<count;++j){int dest;std::cin>>dest;q[{f,d}].push_back(dest);}}
  if(!std::cin)throw std::runtime_error("incomplete drain input");
  auto r=drain_cost(b,c,q,bypass);
  std::cout<<std::setprecision(17)<<r.waiting<<' '<<r.journey<<' '<<r.end<<' '<<r.boarded<<' '<<r.alighted<<' '<<r.capacity_blocked_stops<<' '<<r.partial_stops<<'\n';
 }
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}}
