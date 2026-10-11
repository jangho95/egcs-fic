#include <iostream>
#include "batch_choice.hpp"
using namespace egcs2017;
int main(){int eligible=0;
 for(int p=1;p<13;++p)for(int f=2;f<9;++f)for(auto method:{"batch_expected","batch_route_expected","batch_route_mean","batch_global_diag","batch_global_expected","batch_global_mean"}){
 Building b;b.floors=10;CarView A;A.id=0;A.x=b.pos(9);A.committed=9;A.dir=DOWN;A.load=p;A.car_calls[0]=1;
 CarView B;B.id=1;B.x=0;B.committed=0;
 CallView c;c.key={f,DOWN};c.assigned=0;c.registered=0;c.waited=200;
 CallView d;d.key={1,DOWN};d.assigned=1;d.registered=0.1;d.waited=199.9;
 auto x=make_dispatcher(method),y=make_dispatcher(method);
 for(auto* disp:{x.get(),y.get()}){disp->observe_boarding(100,f,DOWN,6);disp->observe_boarding(150,1,DOWN,3);}
 DecisionTrace tx,ty;auto ax=x->decide(b,{A,B},{c,d},c.key,&tx);
 c.waiting=999;d.waiting=173;auto ay=y->decide(b,{A,B},{c,d},c.key,&ty);
 if(ax!=ay||tx.batch_eligible!=ty.batch_eligible||tx.batch_wait!=ty.batch_wait||tx.batch_journey!=ty.batch_journey)return 2;
 if(tx.batch_global_complete!=ty.batch_global_complete||tx.batch_global_proposed!=ty.batch_global_proposed||tx.batch_single_journey!=ty.batch_single_journey)return 4;
 if(std::string(method)=="batch_global_diag"){
  auto base=make_dispatcher("paper2017_ls_lk");DecisionTrace tb;
  if(ax!=base->decide(b,{A,B},{c,d},c.key,&tb)||tx.batch_changed)return 5;
 }
 eligible+=tx.batch_eligible+tx.batch_global_complete;
 }
 if(!eligible)return 3;
 std::cout<<"504 observation checks, eligible "<<eligible<<"; no waiting-count dependence\n";
}
