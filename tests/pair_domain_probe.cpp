#include "../research/forward_pair_features.hpp"
#include <iomanip>
#include <iostream>
#include <map>
using namespace egcs2017;
using namespace egcs2017::research;
struct Stat {long count=0;double cost=0;};
int main() {
 try {
  std::map<std::string,Stat> ff,fl;
  int F,C,n;long states=0;
  while(std::cin>>F>>C>>n) {
   Building b;b.floors=F;std::vector<CarView> cars(C);
   for(auto& c:cars) {
    int count;std::cin>>c.id>>c.x>>c.v>>c.dir>>c.moving>>c.arriving>>c.committed
      >>c.release>>c.doors_open>>c.load>>count;
    for(int k=0;k<count;++k){int f,q;std::cin>>f>>q;c.car_calls[f]=q;}
   }
   std::vector<HallKey> free(n);for(auto& k:free)std::cin>>k.floor>>k.dir;
   std::vector<std::vector<HallKey>> locked(C);
   for(auto& row:locked){int count;std::cin>>count;row.resize(count);for(auto& k:row)std::cin>>k.floor>>k.dir;}
   std::vector<std::vector<double>> u(n,std::vector<double>(C)),r=u;
   for(auto& row:u)for(auto& x:row)std::cin>>x;
   for(auto& row:r)for(auto& x:row)std::cin>>x;
   std::vector w(n,std::vector(n,std::vector<double>(C)));
   for(auto& row:w)for(auto& col:row)for(auto& x:col)std::cin>>x;
   if(!std::cin)throw std::runtime_error("truncated input");
   for(int c=0;c<C;++c) {
    auto feature=forward_features(b,cars[c],cars[c].dir);
    auto classify=[&](HallKey i,HallKey j) {
     if(!feature)return std::string("idle_or_geometry_outside");
     if(feature->contains(i)&&feature->contains(j))return std::string("forward_same_pair");
     const auto first=run_path(b,cars[c],{VCall{i,-1,1,0},VCall{j,-1,1,1}},1);
     if(first.order.empty())throw std::runtime_error("no pickup order");
     if(first.order[0]==1)std::swap(i,j);
     auto type=[&](HallKey k) {
      return std::string((k.floor-cars[c].committed)*cars[c].dir>=0?"ahead_":"behind_")+
       (k.dir==cars[c].dir?"same":"opposite");
     };
     return type(i)+"_then_"+type(j);
    };
    for(int i=0;i<n;++i) {
     for(int j=i+1;j<n;++j){auto& s=ff[classify(free[i],free[j])];++s.count;s.cost+=w[i][j][c];}
     for(auto l:locked[c]) {
      auto& s=fl[classify(free[i],l)];++s.count;
      CostOptions raw;s.cost+=pairwise(b,cars[c],free[i],l,u[i][c],unary(b,cars[c],l,raw));
     }
    }
   }
   ++states;
  }
  std::cout<<std::setprecision(17)<<"{\"states\":"<<states;
  auto emit=[&](const char* name,const auto& stats) {
   std::cout<<",\""<<name<<"\":{";bool first=true;
   for(auto& [key,s]:stats){if(!first)std::cout<<',';first=false;
    std::cout<<'"'<<key<<"\":{\"count\":"<<s.count<<",\"cost_sum\":"<<s.cost<<'}';}
   std::cout<<'}';
  };
  emit("free_free",ff);emit("free_locked",fl);std::cout<<"}\n";
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}
}
