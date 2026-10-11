#include "dqn3.hpp"
#include "sequential_greedy.hpp"
#include "../src/traffic.hpp"
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
using namespace egcs2017;
using namespace egcs2017::research::dqn3;
namespace fs=std::filesystem;
struct Cell {int cars,rate;};
const std::array<Cell,9> cells{{{2,10},{2,20},{2,30},{4,10},{4,20},{4,30},{6,10},{6,20},{6,30}}};
struct Run {Metrics m;double seconds=0,reward=0,integral=0;long selections=0;};
unsigned long traffic_seed(long label,Cell c){return static_cast<unsigned long>(label)*1000003UL+10*101+c.cars*7+c.rate*10;}
Run episode(Learner& learner,Cell c,long label,Mode mode,bool invariants=false) {
 Building b;b.floors=10;auto dispatcher=std::make_unique<Policy>(learner,mode);auto* p=dispatcher.get();
 SimOptions o;o.theta=5;o.button_info=true;o.check_invariants=invariants;
 auto ps=interfloor_traffic(10,c.rate,30,3600,traffic_seed(label,c));auto start=std::chrono::steady_clock::now();
 Simulation sim(b,c.cars,std::move(dispatcher),std::move(ps),nullptr,14400,o);sim.run();
 Run r{sim.metrics(),std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count(),p->episode_reward,p->terminal_integral,p->selections};
 double wait=0;for(const auto& x:sim.passengers())wait+=x.boarded>=0?x.boarded-x.arrival:sim.now()-x.arrival;
 if(!p->terminal_seen||std::abs(wait-r.integral)>1e-7*std::max(1.0,wait))throw std::runtime_error("queue integral identity failed");
 if(mode==Mode::TRAIN&&std::abs(r.reward+wait/100)>1e-7*std::max(1.0,wait/100))throw std::runtime_error("transition reward identity failed");
 return r;
}
void header(std::ostream& out){out<<"stage,model_seed,episode,label,cars,rate,n,unserved,unfinished,awt,p95_wait,att,ajt,seconds,selections,transitions,updates,loss,gradient,max_abs_q,epsilon,reward,queue_integral,aux_loss\n";}
void row(std::ostream& out,const std::string& stage,long seed,int ep,long label,Cell c,const Run& r,const Learner& l) {
 const auto& m=r.m;
 out<<stage<<','<<seed<<','<<ep<<','<<label<<','<<c.cars<<','<<c.rate<<','<<m.n<<','<<m.unserved<<','<<m.unfinished<<','<<m.awt<<','<<m.p95_wait<<','<<m.att<<','<<m.ajt<<','<<r.seconds<<','<<r.selections<<','<<l.transitions<<','<<l.online.updates<<','<<l.last_loss<<','<<l.last_gradient<<",0,"<<(stage=="train"?l.epsilon():0)<<','<<r.reward<<','<<r.integral<<','<<l.last_aux<<'\n';out.flush();
}
double validate(Learner& learner,long seed,int checkpoint,std::ostream& log,double& seconds) {
 const auto weights=learner.online.weights;const auto updates=learner.online.updates;const auto transitions=learner.transitions;
 double score=0;bool failed=false;
 for(long label=97000;label<97010;++label)for(auto c:cells) {
  auto r=episode(learner,c,label,Mode::EVALUATE);seconds+=r.seconds;row(log,"validation",seed,checkpoint,label,c,r,learner);
  failed|=r.m.unserved||r.m.unfinished;score+=r.m.awt/90;
 }
 if(weights!=learner.online.weights||updates!=learner.online.updates||transitions!=learner.transitions)throw std::runtime_error("evaluation learned");
 return failed?std::numeric_limits<double>::infinity():score;
}
int main(int argc,char** argv){try {
 std::cout<<std::setprecision(17);
 if(argc==3&&std::string(argv[1])=="pilot") {
  fs::path dir=argv[2];fs::create_directories(dir);std::ofstream log(dir/"episodes.csv");log<<std::setprecision(17);header(log);Learner learner(85100,.1);
  double seconds=0;long failed=0;
  for(int ep=0;ep<36;++ep){auto r=episode(learner,cells[ep%9],96500+ep,Mode::TRAIN,true);seconds+=r.seconds;failed+=r.m.unserved||r.m.unfinished;row(log,"pilot",85100,ep+1,96500+ep,cells[ep%9],r,learner);}
  std::ofstream s(dir/"summary.json");s<<std::setprecision(17)<<"{\"episodes\":36,\"seconds\":"<<seconds<<",\"updates\":"<<learner.online.updates<<",\"transitions\":"<<learner.transitions<<",\"failed_episodes\":"<<failed<<"}\n";
  std::cout<<"Pilot: seconds="<<seconds<<" updates="<<learner.online.updates<<" failures="<<failed<<'\n';return failed?3:0;
 }
 if(argc==9&&std::string(argv[1])=="train") {
  long seed=std::stol(argv[2]),warm_block=std::stol(argv[3]),block=std::stol(argv[4]);double guidance=std::stod(argv[5]);int total=std::stoi(argv[6]),interval=std::stoi(argv[7]);fs::path dir=argv[8];
  if((guidance!=0&&guidance!=.1)||(total!=900&&total!=3600)||interval<=0)throw std::runtime_error("training outside frozen budget");
  fs::create_directories(dir);std::ofstream warm(dir/"warmup.csv"),log(dir/"episodes.csv"),validation(dir/"validation.csv"),curve(dir/"checkpoints.csv");
  warm<<std::setprecision(17);log<<std::setprecision(17);validation<<std::setprecision(17);curve<<std::setprecision(17);header(warm);header(log);header(validation);curve<<"episode,updates,transitions,validation_awt,best\n";
  Learner learner(seed,guidance);std::mt19937_64 order(seed^0xabcd983aULL);std::array<int,9> permutation;std::iota(permutation.begin(),permutation.end(),0);
  double warm_seconds=0,train_seconds=0,validation_seconds=0;long failed_warm=0,failed_train=0;
  for(int ep=0;ep<90;++ep) {
   if(ep%9==0)std::shuffle(permutation.begin(),permutation.end(),order);
   Cell c=cells[permutation[ep%9]];auto r=episode(learner,c,warm_block+ep,Mode::WARMUP);
   warm_seconds+=r.seconds;failed_warm+=r.m.unserved||r.m.unfinished;row(warm,"warmup",seed,ep+1,warm_block+ep,c,r,learner);
  }
  long warm_updates=learner.online.updates,warm_samples=learner.transitions;learner.start_rl();
  double best=validate(learner,seed,0,validation,validation_seconds);int best_ep=0;
  learner.online.save((dir/"best.bin").string());learner.online.save((dir/"checkpoint-0.bin").string());curve<<0<<",0,0,"<<best<<",1\n";curve.flush();
  std::cout<<"seed="<<seed<<" ep=0 val_awt="<<best<<" warm_updates="<<warm_updates<<'\n'<<std::flush;
  for(int ep=1;ep<=total;++ep) {
   if((ep-1)%9==0)std::shuffle(permutation.begin(),permutation.end(),order);
   Cell c=cells[permutation[(ep-1)%9]];auto r=episode(learner,c,block+ep-1,Mode::TRAIN);
   train_seconds+=r.seconds;failed_train+=r.m.unserved||r.m.unfinished;row(log,"train",seed,ep,block+ep-1,c,r,learner);
   if(ep%interval==0||ep==total) {
    double score=validate(learner,seed,ep,validation,validation_seconds);bool improved=score<best;
    if(improved){best=score;best_ep=ep;learner.online.save((dir/"best.bin").string());}
    learner.online.save((dir/("checkpoint-"+std::to_string(ep)+".bin")).string());curve<<ep<<','<<learner.online.updates<<','<<learner.transitions<<','<<score<<','<<improved<<'\n';curve.flush();
    std::cout<<"seed="<<seed<<" ep="<<ep<<" updates="<<learner.online.updates<<" val_awt="<<score<<" best_ep="<<best_ep<<'\n'<<std::flush;
   }
  }
  std::ofstream s(dir/"summary.json");s<<std::setprecision(17)<<"{\"seed\":"<<seed<<",\"auxiliary\":"<<guidance<<",\"episodes\":"<<total<<",\"updates\":"<<learner.online.updates<<",\"transitions\":"<<learner.transitions<<",\"selected_episode\":"<<best_ep<<",\"selected_validation_awt\":"<<best<<",\"warmup_episodes\":90,\"warmup_updates\":"<<warm_updates<<",\"warmup_samples\":"<<warm_samples<<",\"warmup_seconds\":"<<warm_seconds<<",\"training_seconds\":"<<train_seconds<<",\"validation_seconds\":"<<validation_seconds<<",\"failed_warmup_episodes\":"<<failed_warm<<",\"failed_training_episodes\":"<<failed_train<<"}\n";
  return std::isfinite(best)?0:3;
 }
 if(argc==5&&std::string(argv[1])=="test") {
  long seed=std::stol(argv[2]);Learner learner(seed,0);learner.online.load(argv[3]);std::ofstream log(argv[4]);log<<std::setprecision(17);header(log);
  for(long label=98000;label<98030;++label)for(auto c:cells){auto r=episode(learner,c,label,Mode::EVALUATE);row(log,"dqn3",seed,0,label,c,r,learner);}return 0;
 }
 if(argc==5&&std::string(argv[1])=="old2") {
  long seed=std::stol(argv[2]);egcs2017::research::dqn2::Learner learner(seed);learner.online.load(argv[3]);Learner dummy(0,0);std::ofstream log(argv[4]);log<<std::setprecision(17);header(log);
  for(long label=98000;label<98030;++label)for(auto c:cells) {
   Building b;b.floors=10;auto dispatcher=std::make_unique<egcs2017::research::dqn2::Policy>(learner,false);
   auto ps=interfloor_traffic(10,c.rate,30,3600,traffic_seed(label,c));Simulation sim(b,c.cars,std::move(dispatcher),std::move(ps));sim.run();row(log,"dqn2",seed,0,label,c,Run{sim.metrics()},dummy);
  }return 0;
 }
 if(argc==4&&std::string(argv[1])=="comparators") {
  std::string method=argv[2];if(method!="route_rule"&&method!="sequential_greedy"&&method!="paper2017_ls"&&method!="paper2017_ls_lk")throw std::runtime_error("unregistered comparator");
  Learner dummy(0,0);std::ofstream log(argv[3]);log<<std::setprecision(17);header(log);
  for(long label=98000;label<98030;++label)for(auto c:cells) {
   Run r;
   if(method=="route_rule")r=episode(dummy,c,label,Mode::ROUTE_RULE);
   else{Building b;b.floors=10;auto ps=interfloor_traffic(10,c.rate,30,3600,traffic_seed(label,c));Simulation sim(b,c.cars,make_extended_dispatcher(method),std::move(ps));sim.run();r.m=sim.metrics();}
   row(log,method,-1,0,label,c,r,dummy);
  }return 0;
 }
 throw std::runtime_error("usage: pilot DIR | train SEED WARM_LABEL TRAIN_LABEL AUX EPISODES INTERVAL DIR | test/old2 SEED MODEL CSV | comparators METHOD CSV");
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}}
