#include "../research/dqn3.hpp"
#include "../src/traffic.hpp"
#include <filesystem>
#include <iostream>
using namespace egcs2017;
using namespace egcs2017::research::dqn3;
void require(bool valid,const char* text){if(!valid)throw std::runtime_error(text);}
int main(){try {
 Network net(419);Candidates xs(3);std::mt19937_64 rng(971);
 for(auto& x:xs)for(float& f:x)f=float(std::uniform_real_distribution<double>(-.2,1)(rng));
 auto prediction=net.forward(xs);std::array<double,Network::parameters> g{};
 net.backward_q(xs,prediction,1,1,g);double maximum=0;
 for(int k=0;k<Network::parameters;k+=43) {
  double original=net.weights[k];net.weights[k]=original+1e-6;double plus=net.forward(xs).q[1];net.weights[k]=original-1e-6;double minus=net.forward(xs).q[1];net.weights[k]=original;
  maximum=std::max(maximum,std::abs((plus-minus)/2e-6-g[k]));
 }
 require(maximum<1e-6,"dueling numerical gradient error");
 g.fill(0);net.backward_guide(xs,prediction,1,true,g);double guide_error=0;
 auto loss=[&]{std::array<double,Network::parameters> scratch{};return net.backward_guide(xs,net.forward(xs),1,true,scratch);};
 for(int k=0;k<Network::parameters;k+=47) {
  double original=net.weights[k];net.weights[k]=original+1e-6;double plus=loss();net.weights[k]=original-1e-6;double minus=loss();net.weights[k]=original;
  guide_error=std::max(guide_error,std::abs((plus-minus)/2e-6-g[k]));
 }
 require(guide_error<1e-6,"guide numerical gradient error");
 const auto q=net.forward(xs).q;net.weights[Network::ba]+=100;
 for(size_t i=0;i<xs.size();++i)require(std::abs(q[i]-net.forward(xs).q[i])<1e-12,"advantage centering error");
 auto checkpoint=std::filesystem::temp_directory_path()/"egcs-dqn3-check.bin";
 net.save(checkpoint.string());Network loaded(9);loaded.load(checkpoint.string());std::filesystem::remove(checkpoint);
 require(loaded.forward(xs).q==net.forward(xs).q,"checkpoint changed output");
 Learner double_test(3,0);double_test.online.weights.fill(0);double_test.target.weights.fill(0);
 double_test.online.weights[0]=double_test.online.weights[Network::w2]=double_test.online.weights[Network::wa]=1;
 double_test.target.weights[0]=double_test.target.weights[Network::w2]=1;double_test.target.weights[Network::wa]=-2;double_test.target.weights[Network::bv]=3;
 Candidates next(2);next[1][0]=1;double_test.replay.push_back(Transition{next,next,0,0,.95,false});double_test.update(false);
 require(std::abs(double_test.last_loss-1.9)<1e-12,"Double DQN selected with target network");
 Building b;b.floors=10;std::vector<CarView> cars(3);
 for(int i=0;i<3;++i){cars[i].id=i;cars[i].x=7*i;cars[i].committed=2*i;cars[i].load=i;cars[i].car_calls[5]=1;}
 CallView pending{{3,UP},12,-1,false,1},existing{{6,DOWN},5,2,false,1};
 std::vector<CallView> calls{pending,existing};std::map<HallKey,int> partial{{pending.key,-1},{existing.key,2}};
 auto score=route_score(b,cars[0],pending.key,partial);
 auto fa=feature(b,pending,cars[0],cars,calls,partial,1,20,score,score.cost);
 auto changed=cars;for(auto& c:changed)c.car_calls[5]=999;auto hidden=calls;hidden[0].waiting=1000;hidden[0].waited=777;
 auto hidden_score=route_score(b,changed[0],pending.key,partial);auto fb=feature(b,hidden[0],changed[0],changed,hidden,partial,1,20,hidden_score,score.cost);
 require(fa==fb&&score.cost==hidden_score.cost,"hidden counts entered features");
 Learner evaluator(7,.1);Policy eval(evaluator,Mode::EVALUATE);eval.observe_learning(20,0,false);
 auto a=eval.decide(b,cars,calls,no_new_call(),nullptr);std::reverse(cars.begin(),cars.end());auto reversed=eval.decide(b,cars,calls,no_new_call(),nullptr);
 require(a==reversed,"car permutation changed decision");require(a.at(existing.key)==2,"existing assignment changed");
 for(auto& car:cars)car.load=11;auto none=eval.decide(b,cars,calls,no_new_call(),nullptr);
 require(none.at(pending.key)==-1&&none.at(existing.key)==2,"capacity mask changed old assignment");
 Learner learner(1002,.1);auto policy=std::make_unique<Policy>(learner,Mode::TRAIN);auto* observer=policy.get();SimOptions options;options.check_invariants=true;
 auto ps=interfloor_traffic(10,10,30,900,31204);Simulation sim(b,3,std::move(policy),std::move(ps),nullptr,14400,options);sim.run();auto metrics=sim.metrics();
 require(!metrics.unserved&&!metrics.unfinished,"check simulation did not drain");double wait=0;for(const auto& p:sim.passengers())wait+=p.boarded-p.arrival;
 require(observer->terminal_seen&&learner.terminals==1,"terminal transition missing");
 require(std::abs(observer->episode_reward+wait/100)<1e-8,"reward integral mismatch");
 std::cout<<"{\"checks\":\"passed\",\"gradient_max_error\":"<<maximum<<",\"guide_gradient_max_error\":"<<guide_error<<",\"passengers\":"<<metrics.n<<",\"reward_error\":"<<std::abs(observer->episode_reward+wait/100)<<"}\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
