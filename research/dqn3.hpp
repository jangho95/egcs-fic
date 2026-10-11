#pragma once
#include "dqn2.hpp"
#include <numeric>

namespace egcs2017::research::dqn3 {
constexpr int inputs=122,hidden=32;
using Feature=std::array<float,inputs>;
using Candidates=std::vector<Feature>;
enum class Mode { WARMUP,TRAIN,EVALUATE,ROUTE_RULE };
struct RouteScore { double cost=0,old_delay=0,ride_delay=0; };
inline int probe_destination(const Building& b,const HallKey& key,int probe) {
 auto floors=destinations(b,key);
 if(floors.empty())throw std::runtime_error("empty directional destination set");
 const int rotation=(key.floor*2+(key.dir==UP))%4;
 const double fraction=(((probe+rotation)%4)+.5)/4.0;
 return floors[std::min(floors.size()-1,size_t(fraction*floors.size()))];
}
inline RouteScore route_score(const Building& b,const CarView& original,const HallKey& target,
 const std::map<HallKey,int>& partial) {
 const auto car=dqn2::button_view(original);RouteScore score;
 for(int probe=0;probe<4;++probe) {
  std::vector<VCall> old;
  for(const auto& [key,id]:partial)if(id==car.id&&!(key==target))
   old.push_back(VCall{key,probe_destination(b,key,probe),1,int(old.size())});
  auto before=run_path_full(b,car,old);auto next=old;
  next.push_back(VCall{target,probe_destination(b,target,probe),1,int(old.size())});
  auto after=run_path_full(b,car,next);
  double delay=0;for(size_t i=0;i<old.size();++i)delay+=after.pickup.at(i)-before.pickup.at(i);
  double ride=0;
  for(const auto& [floor,count]:car.car_calls) {
   (void)count;
   ride+=(after.alight.at(floor)-before.alight.at(floor))*car.load/std::max(size_t(1),car.car_calls.size());
  }
  score.cost+=(after.pickup.at(old.size())+delay)/4;
  score.old_delay+=delay/4;score.ride_delay+=ride/4;
 }
 if(!std::isfinite(score.cost)||!std::isfinite(score.ride_delay))throw std::runtime_error("nonfinite route score");
 return score;
}
inline Feature feature(const Building& b,const CallView& target,const CarView& car,
 const std::vector<CarView>& cars,const std::vector<CallView>& calls,
 const std::map<HallKey,int>& partial,int remaining,double now,const RouteScore& score,double minimum) {
 Feature x{};auto base=dqn2::feature(b,target,car,cars,calls,partial,remaining,now);
 std::copy(base.begin(),base.end(),x.begin());
 auto scaled=[](double v){return float(std::clamp(v/30.0,-20.0,20.0));};
 x[118]=scaled(score.cost);x[119]=scaled(score.cost-minimum);
 x[120]=scaled(score.old_delay);x[121]=scaled(score.ride_delay);
 return x;
}
inline size_t best_index(const std::vector<double>& values) {
 if(values.empty())throw std::runtime_error("empty Q action set");
 return size_t(std::max_element(values.begin(),values.end())-values.begin());
}
struct Network {
 static constexpr int b1=hidden*inputs,w2=b1+hidden,b2=w2+hidden*hidden,
  wv=b2+hidden,bv=wv+hidden,wa=bv+1,ba=wa+hidden,parameters=ba+1;
 std::array<double,parameters> weights{},adam_m{},adam_v{};long updates=0;
 struct Cache {std::array<double,hidden> a{},b{};double value=0,advantage=0;};
 struct Output {std::vector<double> q;std::vector<Cache> cache;double mean_value=0,mean_advantage=0;};
 explicit Network(unsigned long seed=0) {
  std::mt19937_64 rng(seed);
  std::normal_distribution<double> d1(0,std::sqrt(2.0/inputs)),d2(0,std::sqrt(2.0/hidden)),d3(0,std::sqrt(1.0/hidden));
  for(int k=0;k<b1;++k)weights[k]=d1(rng);
  for(int k=w2;k<b2;++k)weights[k]=d2(rng);
  for(int k=wv;k<bv;++k)weights[k]=d3(rng);
  for(int k=wa;k<ba;++k)weights[k]=d3(rng);
 }
 Cache encode(const Feature& x)const {
  Cache c;
  for(int h=0;h<hidden;++h) {
   double a=weights[b1+h];for(int j=0;j<inputs;++j)a+=weights[h*inputs+j]*x[j];c.a[h]=std::max(0.0,a);
  }
  for(int h=0;h<hidden;++h) {
   double v=weights[b2+h];for(int j=0;j<hidden;++j)v+=weights[w2+h*hidden+j]*c.a[j];c.b[h]=std::max(0.0,v);
  }
  c.value=weights[bv];c.advantage=weights[ba];
  for(int h=0;h<hidden;++h){c.value+=weights[wv+h]*c.b[h];c.advantage+=weights[wa+h]*c.b[h];}
  return c;
 }
 Output forward(const Candidates& xs)const {
  if(xs.empty())throw std::runtime_error("empty dueling state");
  Output out;out.cache.reserve(xs.size());out.q.reserve(xs.size());
  for(const auto& x:xs){out.cache.push_back(encode(x));out.mean_value+=out.cache.back().value/xs.size();out.mean_advantage+=out.cache.back().advantage/xs.size();}
  for(const auto& c:out.cache){double q=out.mean_value+c.advantage-out.mean_advantage;if(!std::isfinite(q))throw std::runtime_error("nonfinite dueling Q");out.q.push_back(q);}
  return out;
 }
 void backward_one(const Feature& x,const Cache& c,double dv,double da,std::array<double,parameters>& g)const {
  g[bv]+=dv;g[ba]+=da;std::array<double,hidden> first{};
  for(int h=0;h<hidden;++h) {
   g[wv+h]+=dv*c.b[h];g[wa+h]+=da*c.b[h];
   const double d=c.b[h]>0?dv*weights[wv+h]+da*weights[wa+h]:0;g[b2+h]+=d;
   for(int j=0;j<hidden;++j){g[w2+h*hidden+j]+=d*c.a[j];first[j]+=d*weights[w2+h*hidden+j];}
  }
  for(int h=0;h<hidden;++h) {
   double d=c.a[h]>0?first[h]:0;g[b1+h]+=d;
   for(int j=0;j<inputs;++j)g[h*inputs+j]+=d*x[j];
  }
 }
 void backward_q(const Candidates& xs,const Output& out,size_t selected,double derivative,
  std::array<double,parameters>& g)const {
  if(selected>=xs.size())throw std::runtime_error("invalid selected action");
  const double mean=derivative/xs.size();
  for(size_t i=0;i<xs.size();++i)backward_one(xs[i],out.cache[i],mean,(i==selected?derivative:0)-mean,g);
 }
 double backward_guide(const Candidates& xs,const Output& out,double weight,bool warm,
  std::array<double,parameters>& g)const {
  double costmean=0,error_mean=0,loss=0;std::vector<double> errors;
  for(const auto& x:xs)costmean+=x[118]/xs.size();
  for(size_t i=0;i<xs.size();++i){double e=out.cache[i].advantage-out.mean_advantage+xs[i][118]-costmean;errors.push_back(e);error_mean+=e/xs.size();loss+=.5*e*e/xs.size();}
  for(size_t i=0;i<xs.size();++i)
   backward_one(xs[i],out.cache[i],warm?weight*out.mean_value/xs.size():0,weight*(errors[i]-error_mean)/xs.size(),g);
  return loss+(warm?.5*out.mean_value*out.mean_value:0);
 }
 double adam(std::array<double,parameters>& gradient,double lr) {
  double norm=0;for(double v:gradient){if(!std::isfinite(v))throw std::runtime_error("nonfinite gradient");norm+=v*v;}norm=std::sqrt(norm);
  double scale=norm>10?10/norm:1;++updates;double c1=1-std::pow(.9,updates),c2=1-std::pow(.999,updates);
  for(int k=0;k<parameters;++k) {
   const double v=gradient[k]*scale;adam_m[k]=.9*adam_m[k]+.1*v;adam_v[k]=.999*adam_v[k]+.001*v*v;
   weights[k]-=lr*(adam_m[k]/c1)/(std::sqrt(adam_v[k]/c2)+1e-8);
   if(!std::isfinite(weights[k]))throw std::runtime_error("nonfinite weight");
  }
  return norm;
 }
 void save(const std::string& path)const {
  std::ofstream out(path,std::ios::binary);const std::uint64_t magic=0x4547435344335131ULL;const int n=parameters;
  out.write(reinterpret_cast<const char*>(&magic),sizeof magic);out.write(reinterpret_cast<const char*>(&n),sizeof n);out.write(reinterpret_cast<const char*>(weights.data()),sizeof weights);
  if(!out)throw std::runtime_error("checkpoint write failed");
 }
 void load(const std::string& path) {
  std::ifstream in(path,std::ios::binary);std::uint64_t magic=0;int n=0;
  in.read(reinterpret_cast<char*>(&magic),sizeof magic);in.read(reinterpret_cast<char*>(&n),sizeof n);
  if(magic!=0x4547435344335131ULL||n!=parameters)throw std::runtime_error("DQN3 checkpoint version mismatch");
  in.read(reinterpret_cast<char*>(weights.data()),sizeof weights);
  if(!in||in.peek()!=std::char_traits<char>::eof())throw std::runtime_error("invalid checkpoint length");
  for(double v:weights)if(!std::isfinite(v))throw std::runtime_error("nonfinite checkpoint");
 }
};
struct Transition {Candidates state,next;size_t selected=0;double reward=0,discount=1;bool terminal=false;};
struct Learner {
 Network online,target;std::mt19937_64 rng;std::vector<Transition> replay;
 size_t cursor=0;long transitions=0,terminals=0,same_time=0;double auxiliary=0,lr=.0005,last_loss=0,last_aux=0,last_gradient=0;
 explicit Learner(unsigned long seed,double guidance):online(seed),target(online),rng(seed^0xabe391778ULL),auxiliary(guidance){replay.reserve(50000);}
 double epsilon()const{return 1-.95*std::min(1.0,double(transitions)/400000);}
 void remember(Transition t) {
  if(replay.size()<50000)replay.push_back(std::move(t));else{replay[cursor]=std::move(t);cursor=(cursor+1)%50000;}
 }
 void teacher(Candidates xs) {
  ++transitions;remember(Transition{std::move(xs),{},0,0,1,true});
  if(transitions>=128&&transitions%4==0)update(true);
 }
 void add(Transition t) {
  ++transitions;terminals+=t.terminal;same_time+=!t.terminal&&t.discount==1;remember(std::move(t));
  if(transitions>=1000&&transitions%4==0)update(false);
 }
 void start_rl() {
  replay.clear();cursor=0;transitions=terminals=same_time=0;online.adam_m.fill(0);online.adam_v.fill(0);online.updates=0;
  target.weights=online.weights;last_loss=last_aux=last_gradient=0;
 }
 void update(bool warm) {
  std::array<double,Network::parameters> gradient{};std::uniform_int_distribution<size_t> pick(0,replay.size()-1);
  double loss=0,guide=0;
  for(int n=0;n<32;++n) {
   const auto& t=replay[pick(rng)];auto prediction=online.forward(t.state);
   if(warm)guide+=online.backward_guide(t.state,prediction,1.0/32,true,gradient);
   else {
    double y=t.reward;
    if(!t.terminal){auto selection=online.forward(t.next);auto evaluation=target.forward(t.next);y+=t.discount*evaluation.q.at(best_index(selection.q));}
    const double error=prediction.q.at(t.selected)-y;
    loss+=std::abs(error)<=1?.5*error*error:std::abs(error)-.5;
    online.backward_q(t.state,prediction,t.selected,std::clamp(error,-1.0,1.0)/32,gradient);
    if(auxiliary>0)guide+=online.backward_guide(t.state,prediction,auxiliary/32,false,gradient);
   }
  }
  last_loss=loss/32;last_aux=guide/32;last_gradient=online.adam(gradient,lr);
  if(!warm&&online.updates%1000==0)target.weights=online.weights;
 }
};
class Policy:public Dispatcher {
 Learner& learner;Mode mode;double now=0,cumulative=0,previous_time=0,previous_wait=0;Candidates previous;size_t chosen=0;bool pending=false;
 void flush(const Candidates& next,bool terminal) {
  if(!pending||mode!=Mode::TRAIN)return;
  const double dt=now-previous_time,dw=cumulative-previous_wait;
  if(dt<-1e-9||dw<-1e-6)throw std::runtime_error("learning clock decreased");
  const double reward=-std::max(0.0,dw)/100;
  learner.add(Transition{std::move(previous),next,chosen,reward,dt<=1e-9?1:.95,terminal});
  episode_reward+=reward;pending=false;
 }
public:
 double episode_reward=0,terminal_integral=0;bool terminal_seen=false;long selections=0,masked=0,no_candidates=0;
 Policy(Learner& l,Mode m):learner(l),mode(m){}
 std::string name()const override{return mode==Mode::ROUTE_RULE?"route_wait_rule":"route_guided_d3qn";}
 void observe_learning(double time,double integral,bool terminal)override {
  now=time;cumulative=integral;if(terminal){flush({},true);terminal_integral=integral;terminal_seen=true;}
 }
 std::map<HallKey,int> decide(const Building& b,const std::vector<CarView>& cars,
  const std::vector<CallView>& original,const HallKey&,DecisionTrace* trace)override {
  if(b.floors!=10||b.capacity!=13)throw std::runtime_error("DQN3 outside design domain");
  std::map<HallKey,int> out;std::vector<CallView> pending_calls,visible=original;
  for(auto& cv:visible) {
   out[cv.key]=cv.assigned;
   if(cv.assigned<0)pending_calls.push_back(cv);else cv.close_to_service=true;
  }
  std::sort(pending_calls.begin(),pending_calls.end(),[](const auto& a,const auto& b2){if(a.registered!=b2.registered)return a.registered<b2.registered;return a.key<b2.key;});
  for(size_t stage=0;stage<pending_calls.size();++stage) {
   const auto& call=pending_calls[stage];std::vector<size_t> eligible;std::vector<RouteScore> costs;
   double minimum=std::numeric_limits<double>::infinity();
   for(size_t c=0;c<cars.size();++c) {
    if(cars[c].id==call.forbidden_car)continue;
    if(cars[c].load>=b.capacity-2){++masked;continue;}
    eligible.push_back(c);costs.push_back(route_score(b,cars[c],call.key,out));minimum=std::min(minimum,costs.back().cost);
   }
   if(eligible.empty()){++no_candidates;continue;}
   Candidates xs;
   for(size_t c=0;c<eligible.size();++c)xs.push_back(feature(b,call,cars[eligible[c]],cars,visible,out,int(pending_calls.size()-stage),now,costs[c],minimum));
   flush(xs,false);size_t best=0;
   if(mode==Mode::WARMUP||mode==Mode::ROUTE_RULE) {
    for(size_t c=1;c<costs.size();++c)
     if(costs[c].cost<costs[best].cost-1e-12||(std::abs(costs[c].cost-costs[best].cost)<=1e-12&&cars[eligible[c]].id<cars[eligible[best]].id))best=c;
   } else {
    auto scores=learner.online.forward(xs).q;
    for(size_t c=1;c<scores.size();++c)
     if(scores[c]>scores[best]+1e-12||(std::abs(scores[c]-scores[best])<=1e-12&&cars[eligible[c]].id<cars[eligible[best]].id))best=c;
    if(mode==Mode::TRAIN&&std::uniform_real_distribution<double>(0,1)(learner.rng)<learner.epsilon())best=std::uniform_int_distribution<size_t>(0,xs.size()-1)(learner.rng);
   }
   out[call.key]=cars[eligible[best]].id;++selections;
   if(mode==Mode::WARMUP)learner.teacher(std::move(xs));
   if(mode==Mode::TRAIN){previous=std::move(xs);chosen=best;previous_time=now;previous_wait=cumulative;pending=true;}
  }
  if(trace){trace->method=name();trace->after=out;}return out;
 }
};
}
