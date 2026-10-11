#include "../research/cross_pair_features.hpp"
#include <iomanip>
#include <iostream>
using namespace egcs2017;
using namespace egcs2017::research;
int main() {
 try {
  long states=0,ff=0,fl=0,fast_ff=0,fast_fl=0,union_ff=0,union_fl=0,rows=0,fast_states=0;
  double pair_error=0,row_error=0,saved_error=0;int F,C,n;CostOptions raw;
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
   bool state_fast=false;
   for(int c=0;c<C;++c) {
    auto cross=cross_features(b,cars[c]);
    auto up=forward_features(b,cars[c],UP),down=forward_features(b,cars[c],DOWN);
    auto old=[&](HallKey a,HallKey z) {
      const auto& f=a.dir==UP?up:down;return f&&f->contains(a)&&f->contains(z);
    };
    std::vector<double> uf(n),ul;
    for(int i=0;i<n;++i)uf[i]=u[i][c];
    for(auto l:locked[c])ul.push_back(unary(b,cars[c],l,raw));
    auto aggregate=cross?cross->correction(free,locked[c],uf,ul):std::vector<double>(n,0);
    for(int i=0;i<n;++i) {
     for(int j=i+1;j<n;++j) {
      ++ff;bool hit=cross&&cross->contains(free[i],free[j]);union_ff+=hit||old(free[i],free[j]);
      if(hit) {
       ++fast_ff;state_fast=true;
       const double ref=pairwise(b,cars[c],free[i],free[j],uf[i],uf[j]);
       saved_error=std::max(saved_error,std::abs(ref-w[i][j][c]));
       pair_error=std::max(pair_error,std::abs(cross->pair(free[i],free[j],uf[i],uf[j])-ref));
      }
     }
     double expected=0,total=0;bool row_fast=false;
     for(size_t j=0;j<locked[c].size();++j) {
      ++fl;auto l=locked[c][j];bool hit=cross&&cross->contains(free[i],l);
      union_fl+=hit||old(free[i],l);
      const double ref=std::max(0.0,pairwise(b,cars[c],free[i],l,uf[i],ul[j]));total+=ref;
      if(hit) {
       ++fast_fl;state_fast=true;row_fast=true;expected+=ref;
       pair_error=std::max(pair_error,std::abs(cross->pair(free[i],l,uf[i],ul[j])-ref));
      }
     }
     saved_error=std::max(saved_error,std::abs(total-r[i][c]));
     row_error=std::max(row_error,std::abs(aggregate[i]-expected));rows+=row_fast;
    }
   }
   ++states;fast_states+=state_fast;
  }
  if(pair_error>1e-8||row_error>1e-8||saved_error>1e-8)throw std::runtime_error("saved cross mismatch");
  std::cout<<std::setprecision(17)<<"{\"states\":"<<states<<",\"free_free_pairs\":"<<ff
   <<",\"free_locked_pairs\":"<<fl<<",\"cross_free_free_pairs\":"<<fast_ff<<",\"cross_free_locked_pairs\":"<<fast_fl
   <<",\"union_free_free_pairs\":"<<union_ff<<",\"union_free_locked_pairs\":"<<union_fl
   <<",\"cross_rows\":"<<rows<<",\"cross_states\":"<<fast_states
   <<",\"maximum_pair_error\":"<<pair_error<<",\"maximum_row_error\":"<<row_error
   <<",\"maximum_saved_error\":"<<saved_error<<"}\n";
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}
}
