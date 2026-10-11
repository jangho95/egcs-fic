#include "../research/sequential_greedy.hpp"
#include <random>
#include <iostream>
using namespace egcs2017;
int main(){
 std::mt19937 rng(942);SequentialGreedyDispatcher d;Building b;b.floors=8;
 for(int trial=0;trial<200;++trial){
  std::vector<CarView> cars(3);for(int c=0;c<3;++c){cars[c].id=c;cars[c].committed=rng()%8;cars[c].x=b.pos(cars[c].committed);cars[c].dir=0;}
  std::vector<CallView> calls;
  for(int i=0;i<4;++i){CallView cv;cv.key={i+1,1};cv.assigned=i<3?int(rng()%3):-1;cv.registered=i;calls.push_back(cv);}
  DecisionTrace t;auto a=d.decide(b,cars,calls,calls.back().key,&t);
  for(int i=0;i<3;++i)if(a.at(calls[i].key)!=calls[i].assigned)throw std::runtime_error("old assignment moved");
  CostOptions o;o.bonus=true;o.capacity_penalty=true;
  auto M=build_matrices(b,cars,calls,calls.back().key,o);
  std::vector<int>x;for(auto cv:calls)x.push_back(cv.assigned);int best=0;double value=1e100;
  for(int c=0;c<3;++c){x.back()=c;double v=g_of_public(x,M.u,M.w,3,o);if(v<value-1e-12){value=v;best=c;}}
  if(a.at(calls.back().key)!=best)throw std::runtime_error("does not minimize full quadratic insertion");
  auto cv=calls;for(auto& z:cv)z.assigned=a.at(z.key);
  if(d.decide(b,cars,cv,no_new_call(),nullptr)!=a)throw std::runtime_error("periodic decision moved existing assignment");
 }

 std::vector<CarView> cars(2);for(int c=0;c<2;++c){cars[c].id=c;cars[c].load=11;}
 CallView v;v.key={2,1};v.assigned=-1;
 if(d.decide(b,cars,{v},v.key,nullptr).at(v.key)!=-1)throw std::runtime_error("near-capacity handling");
 cars[0].load=cars[1].load=0;
 if(d.decide(b,cars,{v},v.key,nullptr).at(v.key)!=0)throw std::runtime_error("tie rule");
 std::cout<<"200 full-objective insertion checks; preservation, periodic no-op, capacity and tie checks passed\n";
}
