#include "../research/dqn.hpp"
#include "../src/traffic.hpp"
#include <filesystem>
#include <iostream>
using namespace egcs2017;
using namespace egcs2017::research::dqn;
void require(bool valid,const char* text){if(!valid)throw std::runtime_error(text);}
int main(){try{
 Network net(418);Feature x;std::mt19937_64 rng(97);for(float& f:x)f=float(std::uniform_real_distribution<double>(-.2,1)(rng));
 Network::Cache cache;net.forward(x,&cache);std::array<double,Network::parameters> g{};net.backward(x,cache,1,g);
 double max_error=0;
 for(int k=0;k<Network::parameters;k+=127){double original=net.weights[k];net.weights[k]=original+1e-6;double plus=net.forward(x);net.weights[k]=original-1e-6;double minus=net.forward(x);net.weights[k]=original;
  double numerical=(plus-minus)/2e-6;max_error=std::max(max_error,std::abs(numerical-g[k]));}
 require(max_error<1e-6,"numerical gradient disagreement");
 g.fill(0);g[Network::b3]=2;double before=net.weights[Network::b3];net.adam(g,.001);
 require(std::abs(net.weights[Network::b3]-(before-.001*2/(2+1e-8)))<1e-12,"Adam bias correction error");
 auto path=std::filesystem::temp_directory_path()/"egcs-dqn-checkpoint-test.bin";net.save(path.string());Network restored(9);restored.load(path.string());std::filesystem::remove(path);require(restored.forward(x)==net.forward(x),"checkpoint prediction changed");
 std::vector<CarView> cars(3);for(int i=0;i<3;++i){cars[i].id=i;cars[i].x=i*7;cars[i].committed=i*2;cars[i].load=i;cars[i].car_calls[5]=1;}
 CallView a{{3,UP},12,1,false,1},locked{{6,DOWN},5,2,true,1};std::vector<CallView> calls{a,locked};std::map<HallKey,int> partial{{a.key,-1},{locked.key,2}};
 auto fa=feature(a,cars[0],cars,calls,partial,1,20);auto changed=calls;changed[0].waiting=1000;changed[1].waiting=2048;changed[0].waited=123456;
 auto hidden=cars;for(auto& c:hidden)c.car_calls[5]=999;auto fb=feature(changed[0],hidden[0],hidden,changed,partial,1,20);require(fa==fb,"hidden counts entered observation");
 Learner learner(417);Policy eval(learner,false);eval.observe_learning(20,0,false);Building b;b.floors=10;
 auto choice=eval.decide(b,cars,calls,no_new_call(),nullptr);std::reverse(cars.begin(),cars.end());auto reordered=eval.decide(b,cars,calls,no_new_call(),nullptr);require(choice==reordered,"vehicle permutation changed choice");require(choice.at(locked.key)==2,"fixed call moved");
 learner.online.weights.fill(0);for(auto& c:cars)if(c.id==0)c.load=11;auto removed=eval.decide(b,cars,calls,no_new_call(),nullptr);require(removed.at(a.key)==-1&&removed.at(locked.key)==2,"capacity postprocessing mismatch");
 Learner training(1001);auto dispatcher=std::make_unique<Policy>(training,true);auto* observer=dispatcher.get();SimOptions options;options.check_invariants=true;
 auto passengers=interfloor_traffic(10,10,30,900,31203);Simulation sim(b,3,std::move(dispatcher),std::move(passengers),nullptr,14400,options);sim.run();auto m=sim.metrics();
 require(!m.unserved&&!m.unfinished,"check episode did not drain");require(observer->terminal_seen,"terminal reward missing");
 double wait=0;for(const auto& p:sim.passengers())wait+=p.boarded-p.arrival;
 require(std::abs(observer->terminal_integral-wait)<1e-7*std::max(1.0,wait),"reward integral differs from boarding timestamps");
 require(std::abs(observer->episode_reward+wait/1000)<1e-8*std::max(1.0,wait/1000),"transition rewards do not telescope");
 require(training.terminals==1,"terminal transition count wrong");require(training.same_time>0,"same-instant choices not linked");

 Learner targets(3);targets.online.weights.fill(0);targets.target.weights.fill(0);targets.target.weights[Network::b3]=123;
 targets.replay.push_back(Transition{x,{},-2,1,true});targets.update();require(std::abs(targets.last_loss-1.5)<1e-12,"terminal bootstrapping error");
 std::cout<<"{\"gradient_max_error\":"<<max_error<<",\"passengers\":"<<m.n<<",\"transitions\":"<<training.transitions<<",\"same_instant_transitions\":"<<training.same_time<<",\"reward_integral_error\":"<<std::abs(observer->terminal_integral-wait)<<",\"checks\":\"passed\"}\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
