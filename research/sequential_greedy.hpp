#pragma once
#include "../src/dispatchers.hpp"
#include <algorithm>
#include <chrono>
#include <limits>

namespace egcs2017 {

class SequentialGreedyDispatcher : public Dispatcher {
public:
 std::string name() const override { return "sequential_greedy"; }
 std::map<HallKey,int> decide(const Building& b,const std::vector<CarView>& cars,
   const std::vector<CallView>& calls,const HallKey& nc,DecisionTrace* trace) override {
  auto start=std::chrono::steady_clock::now();
  std::map<HallKey,int> out;
  std::vector<CallView> pending;
  for(const auto& cv:calls) { out[cv.key]=cv.assigned; if(cv.assigned<0) pending.push_back(cv); }
  std::stable_sort(pending.begin(),pending.end(),[](const auto& a,const auto& z){return a.registered<z.registered;});
  CostOptions o; o.bonus=true; o.capacity_penalty=true; o.locked_aware=true;
  if(trace) {trace->method=name();trace->new_call=nc;for(const auto& c:cars)trace->cars.push_back(c.id);}
  for(const auto& target:pending) {
   std::vector<CallView> view;
   for(auto cv:calls) {
    cv.assigned=out.at(cv.key);
    if(cv.key==target.key){cv.close_to_service=false;view.push_back(cv);}
    else if(cv.assigned>=0){cv.close_to_service=true;view.push_back(cv);}
   }
   auto M=build_matrices(b,cars,view,target.key,o);
   if(M.N.size()!=1 || cars.empty()) throw std::logic_error("insertion greedy requires one pending call and cars");
   int best=0;
   for(int c=1;c<(int)cars.size();++c)
    if(M.u[0][c]<M.u[0][best]-1e-12 ||
       (std::abs(M.u[0][c]-M.u[0][best])<=1e-12 && cars[c].id<cars[best].id)) best=c;
   if(cars[best].load<std::max(1,b.capacity-o.capacity_margin))out[target.key]=cars[best].id;
   if(trace){trace->calls.push_back(target.key);trace->unary.push_back(M.u[0]);trace->unary_lk.push_back(M.u_lk[0]);
    trace->before[target.key]=-1;trace->after[target.key]=out[target.key];}
  }
  if(trace)trace->micros=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count();
  return out;
 }
};
inline std::unique_ptr<Dispatcher> make_extended_dispatcher(const std::string& method) {
 if(method=="sequential_greedy")return std::make_unique<SequentialGreedyDispatcher>();
 return make_dispatcher(method);
}
}
