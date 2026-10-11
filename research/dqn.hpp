#pragma once
#include "../src/dispatchers.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <random>
#include <stdexcept>
#include <vector>

namespace egcs2017::research::dqn {
constexpr int inputs=89,hidden=64;
using Feature=std::array<float,inputs>;
using Candidates=std::vector<Feature>;
inline double bounded(double x,double scale){return std::clamp(x/scale,0.0,1.0);}
inline int key_index(const HallKey& k){if(k.floor<0||k.floor>=10||(k.dir!=UP&&k.dir!=DOWN))throw std::runtime_error("DQN requires ten-floor directional keys");return k.floor*2+(k.dir==UP);}
inline Feature feature(const CallView& target,const CarView& car,const std::vector<CarView>& cars,
 const std::vector<CallView>& calls,const std::map<HallKey,int>& partial,int remaining,double now){
 Feature x{};int p=0;
 x[p++]=float(target.key.floor/9.0);x[p++]=float(target.key.dir);x[p++]=float(bounded(now-target.registered,120));
 x[p++]=target.assigned==car.id;x[p++]=float(remaining/18.0);
 x[p++]=float(car.x/31.5);x[p++]=float(car.v*car.dir/1.6);x[p++]=float(car.dir);x[p++]=car.moving;x[p++]=car.arriving;
 x[p++]=float(car.committed/9.0);x[p++]=float(bounded(car.release,30));x[p++]=car.doors_open;x[p++]=float(car.load/13.0);x[p++]=float(car.car_calls.size()/10.0);
 for(const auto& [floor,n]:car.car_calls){(void)n;if(floor<0||floor>=10)throw std::runtime_error("invalid lit button");x[p+floor]=1;}p+=10;
 for(const auto& [key,id]:partial)if(id==car.id)x[p+key_index(key)]=1;
 p+=20;
 for(const auto& cv:calls)if(cv.close_to_service&&cv.assigned==car.id)x[p+key_index(cv.key)]=1;
 p+=20;
 for(const auto& cv:calls)x[p+key_index(cv.key)]=1;
 p+=20;
 double pos=0,load=0,release=0,moving=0;
 for(const auto& cv:cars){pos+=cv.x/31.5;load+=cv.load/13.0;release+=bounded(cv.release,30);moving+=cv.moving;}
 const double C=double(cars.size());if(!C)throw std::runtime_error("no DQN candidates");
 x[p++]=float(pos/C);x[p++]=float(load/C);x[p++]=float(release/C);x[p++]=float(moving/C);
 if(p!=inputs)throw std::runtime_error("feature dimension error");
 for(float v:x)if(!std::isfinite(v))throw std::runtime_error("nonfinite DQN observation");
 return x;
}
struct Network {
 static constexpr int b1=hidden*inputs,w2=b1+hidden,b2=w2+hidden*hidden,w3=b2+hidden,b3=w3+hidden,parameters=b3+1;
 std::array<double,parameters> weights{},adam_m{},adam_v{};long updates=0;
 struct Cache{std::array<double,hidden> a{},b{};};
 explicit Network(unsigned long seed=0){std::mt19937_64 rng(seed);std::normal_distribution<double> d1(0,std::sqrt(2.0/inputs)),d2(0,std::sqrt(2.0/hidden)),d3(0,std::sqrt(1.0/hidden));
  for(int k=0;k<b1;++k)weights[k]=d1(rng);
  for(int k=w2;k<b2;++k)weights[k]=d2(rng);
  for(int k=w3;k<b3;++k)weights[k]=d3(rng);
 }
 double forward(const Feature& x,Cache* save=nullptr)const{
  Cache scratch;Cache& c=save?*save:scratch;
  for(int h=0;h<hidden;++h){double a=weights[b1+h];for(int j=0;j<inputs;++j)a+=weights[h*inputs+j]*x[j];c.a[h]=std::max(0.0,a);}
  for(int h=0;h<hidden;++h){double b=weights[b2+h];for(int j=0;j<hidden;++j)b+=weights[w2+h*hidden+j]*c.a[j];c.b[h]=std::max(0.0,b);}
  double q=weights[b3];for(int h=0;h<hidden;++h)q+=weights[w3+h]*c.b[h];
  if(!std::isfinite(q))throw std::runtime_error("nonfinite Q value");
  return q;
 }
 void backward(const Feature& x,const Cache& c,double derivative,std::array<double,parameters>& g)const{
  g[b3]+=derivative;std::array<double,hidden> da{};
  for(int h=0;h<hidden;++h){g[w3+h]+=derivative*c.b[h];double db=c.b[h]>0?derivative*weights[w3+h]:0;g[b2+h]+=db;
   for(int j=0;j<hidden;++j){g[w2+h*hidden+j]+=db*c.a[j];da[j]+=db*weights[w2+h*hidden+j];}}
  for(int h=0;h<hidden;++h){double d=c.a[h]>0?da[h]:0;g[b1+h]+=d;for(int j=0;j<inputs;++j)g[h*inputs+j]+=d*x[j];}
 }
 double adam(std::array<double,parameters>& g,double lr){double norm=0;for(double v:g){if(!std::isfinite(v))throw std::runtime_error("nonfinite gradient");norm+=v*v;}norm=std::sqrt(norm);
  double scale=norm>10?10/norm:1;++updates;double b1c=1-std::pow(0.9,updates),b2c=1-std::pow(0.999,updates);
  for(int k=0;k<parameters;++k){double v=g[k]*scale;adam_m[k]=.9*adam_m[k]+.1*v;adam_v[k]=.999*adam_v[k]+.001*v*v;
   weights[k]-=lr*(adam_m[k]/b1c)/(std::sqrt(adam_v[k]/b2c)+1e-8);if(!std::isfinite(weights[k]))throw std::runtime_error("nonfinite weight");}return norm;
 }
 void save(const std::string& path)const{std::ofstream out(path,std::ios::binary);if(!out)throw std::runtime_error("checkpoint write failed");const std::uint64_t magic=0x4547435344514e31ULL;const int n=parameters;
  out.write(reinterpret_cast<const char*>(&magic),sizeof magic);out.write(reinterpret_cast<const char*>(&n),sizeof n);out.write(reinterpret_cast<const char*>(weights.data()),sizeof weights);if(!out)throw std::runtime_error("checkpoint truncated");}
 void load(const std::string& path){std::ifstream in(path,std::ios::binary);std::uint64_t magic=0;int n=0;in.read(reinterpret_cast<char*>(&magic),sizeof magic);in.read(reinterpret_cast<char*>(&n),sizeof n);
  if(magic!=0x4547435344514e31ULL||n!=parameters)throw std::runtime_error("checkpoint version mismatch");
  in.read(reinterpret_cast<char*>(weights.data()),sizeof weights);
  if(!in||in.peek()!=std::char_traits<char>::eof())throw std::runtime_error("invalid checkpoint length");
  for(double v:weights)if(!std::isfinite(v))throw std::runtime_error("invalid checkpoint weight");}
};
struct Transition {Feature chosen{};Candidates next;double reward=0,discount=1;bool terminal=false;};
struct Learner {
 Network online,target;std::mt19937_64 rng;std::vector<Transition> replay;size_t cursor=0;long transitions=0,terminals=0,same_time=0;
 double lr,last_loss=0,last_gradient=0,max_abs_q=0,reward_sum=0;bool training=true;
 explicit Learner(unsigned long seed,double learning_rate=.001):online(seed),target(online),rng(seed^0x928abcd45ULL),lr(learning_rate){replay.reserve(50000);}
 double epsilon()const{return training?1.0-.95*std::min(1.0,double(transitions)/200000):0;}
 void add(Transition t){++transitions;terminals+=t.terminal;same_time+=!t.terminal&&t.discount==1;reward_sum+=t.reward;
  if(replay.size()<50000)replay.push_back(std::move(t));else{replay[cursor]=std::move(t);cursor=(cursor+1)%50000;}
  if(training&&transitions>=1000&&transitions%8==0&&online.updates<100000)update();
 }
 void update(){std::uniform_int_distribution<size_t> sample(0,replay.size()-1);std::array<double,Network::parameters> gradient{};double loss=0;
  for(int n=0;n<64;++n){const auto& t=replay[sample(rng)];double y=t.reward;
   if(!t.terminal){double best=-std::numeric_limits<double>::infinity();for(const auto& x:t.next)best=std::max(best,target.forward(x));if(!std::isfinite(best))throw std::runtime_error("empty next action set");y+=t.discount*best;}
   Network::Cache cache;double q=online.forward(t.chosen,&cache),e=q-y;max_abs_q=std::max(max_abs_q,std::abs(q));loss+=std::abs(e)<=1?.5*e*e:std::abs(e)-.5;
   online.backward(t.chosen,cache,std::clamp(e,-1.0,1.0)/64,gradient);
  }last_loss=loss/64;last_gradient=online.adam(gradient,lr);if(online.updates%1000==0)target.weights=online.weights;
 }
};
class Policy:public Dispatcher {
 Learner& learner;bool train;double now=0,cumulative=0,previous_time=0,previous_wait=0;Feature previous{};bool pending=false;
 void flush(const Candidates& next,bool terminal){if(!pending||!train)return;
  const double dt=now-previous_time,dw=cumulative-previous_wait;if(dt<-1e-9||dw<-1e-6)throw std::runtime_error("learning clock decreased");
  Transition t{previous,next,-std::max(0.0,dw)/1000,std::exp(-std::log(2.0)*std::max(0.0,dt)/600),terminal};episode_reward+=t.reward;
  learner.add(std::move(t));pending=false;
 }
public:
 double episode_reward=0,terminal_integral=0;long selections=0;bool terminal_seen=false;
 explicit Policy(Learner& l,bool training):learner(l),train(training){}
 std::string name()const override{return "dqn_shared_reassign";}
 void observe_learning(double time,double wait_integral,bool terminal)override{now=time;cumulative=wait_integral;if(terminal){flush({},true);terminal_integral=wait_integral;terminal_seen=true;}}
 std::map<HallKey,int> decide(const Building& b,const std::vector<CarView>& cars,const std::vector<CallView>& calls,const HallKey&,DecisionTrace* trace)override{
  if(b.floors!=10||b.capacity!=13||cars.empty())throw std::runtime_error("DQN design domain mismatch");
  std::map<HallKey,int> out;std::vector<CallView> free;
  for(const auto& cv:calls){if(cv.close_to_service&&cv.assigned>=0)out[cv.key]=cv.assigned;else{out[cv.key]=-1;free.push_back(cv);}}
  std::sort(free.begin(),free.end(),[](const auto& a,const auto& z){if(a.registered!=z.registered)return a.registered<z.registered;return a.key<z.key;});
  for(size_t i=0;i<free.size();++i){Candidates xs;std::vector<size_t> eligible;
   for(size_t c=0;c<cars.size();++c)if(cars[c].id!=free[i].forbidden_car){eligible.push_back(c);xs.push_back(feature(free[i],cars[c],cars,calls,out,int(free.size()-i),now));}
   if(xs.empty())throw std::runtime_error("no eligible DQN car");
   flush(xs,false);
   size_t best=0;double bestq=-std::numeric_limits<double>::infinity();
   for(size_t c=0;c<xs.size();++c){double q=learner.online.forward(xs[c]);if(q>bestq||(q==bestq&&cars[eligible[c]].id<cars[eligible[best]].id)){best=c;bestq=q;}}
   if(train&&std::uniform_real_distribution<double>(0,1)(learner.rng)<learner.epsilon())best=std::uniform_int_distribution<size_t>(0,xs.size()-1)(learner.rng);
   out[free[i].key]=cars[eligible[best]].id;if(train){previous=xs[best];previous_time=now;previous_wait=cumulative;pending=true;}++selections;
  }
  for(const auto& cv:free){int id=out.at(cv.key);auto car=std::find_if(cars.begin(),cars.end(),[&](const auto& c){return c.id==id;});if(car!=cars.end()&&car->load>=std::max(1,b.capacity-2))out[cv.key]=-1;}
  if(trace){trace->method=name();trace->after=out;}return out;
 }
};
}
