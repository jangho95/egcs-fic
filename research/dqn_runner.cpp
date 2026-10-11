#include "dqn.hpp"
#include "sequential_greedy.hpp"
#include "../src/traffic.hpp"
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
using namespace egcs2017;
using namespace egcs2017::research::dqn;
namespace fs=std::filesystem;
struct Cell{int cars,rate;};
const std::array<Cell,9> cells{{{2,10},{2,20},{2,30},{4,10},{4,20},{4,30},{6,10},{6,20},{6,30}}};
struct Run{Metrics m;double seconds=0,reward=0,integral=0;long selections=0;};
unsigned long traffic_seed(long label,Cell c){return static_cast<unsigned long>(label)*1000003UL+10*101+c.cars*7+c.rate*10;}
Run episode(Learner& learner,Cell cell,long label,bool train,bool invariants=false){
 Building b;b.floors=10;auto dispatcher=std::make_unique<Policy>(learner,train);auto* p=dispatcher.get();SimOptions o;o.theta=5;o.button_info=true;o.check_invariants=invariants;
 auto passengers=interfloor_traffic(10,cell.rate,30,3600,traffic_seed(label,cell));auto start=std::chrono::steady_clock::now();
 Simulation sim(b,cell.cars,std::move(dispatcher),std::move(passengers),nullptr,14400,o);sim.run();auto m=sim.metrics();
 Run r{m,std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count(),p->episode_reward,p->terminal_integral,p->selections};
 if(!p->terminal_seen)throw std::runtime_error("missing end-of-episode reward");
 double wait=0;for(const auto& ps:sim.passengers())wait+=ps.boarded>=0?ps.boarded-ps.arrival:sim.now()-ps.arrival;
 if(std::abs(wait-r.integral)>1e-7*std::max(1.0,wait))throw std::runtime_error("queue integral identity failed");
 if(train&&std::abs(r.reward+wait/1000)>1e-7*std::max(1.0,wait/1000))throw std::runtime_error("learning reward identity failed");
 return r;
}
void header(std::ostream& out){out<<"stage,model_seed,episode,label,cars,rate,n,unserved,unfinished,awt,p95_wait,att,ajt,seconds,selections,transitions,updates,loss,gradient,max_abs_q,epsilon,reward,queue_integral\n";}
void row(std::ostream& out,const std::string& stage,long model,int ep,long label,Cell cell,const Run& r,const Learner& learner){const auto& m=r.m;
 out<<stage<<','<<model<<','<<ep<<','<<label<<','<<cell.cars<<','<<cell.rate<<','<<m.n<<','<<m.unserved<<','<<m.unfinished<<','<<m.awt<<','<<m.p95_wait<<','<<m.att<<','<<m.ajt<<','<<r.seconds<<','<<r.selections<<','<<learner.transitions<<','<<learner.online.updates<<','<<learner.last_loss<<','<<learner.last_gradient<<','<<learner.max_abs_q<<','<<learner.epsilon()<<','<<r.reward<<','<<r.integral<<'\n';out.flush();
}
double validate(Learner& learner,long seed,int checkpoint,std::ostream& log){double sum=0;bool failed=false;
 for(auto cell:cells)for(int i=0;i<10;++i){auto r=episode(learner,cell,90000+i,false);row(log,"validation",seed,checkpoint,90000+i,cell,r,learner);failed|=r.m.unserved||r.m.unfinished;sum+=r.m.awt;}
 return failed?std::numeric_limits<double>::infinity():sum/90;
}
int main(int argc,char** argv){try{
 std::cout<<std::setprecision(17);
 if(argc==3&&std::string(argv[1])=="pilot"){
  fs::path dir=argv[2];fs::create_directories(dir);std::ofstream log(dir/"episodes.csv");log<<std::setprecision(17);header(log);Learner learner(80000);double sum=0;long failed=0;
  for(int ep=0;ep<36;++ep){Cell c=cells[ep%9];auto r=episode(learner,c,70000+ep,true,true);sum+=r.seconds;failed+=r.m.unserved||r.m.unfinished;row(log,"pilot",80000,ep+1,70000+ep,c,r,learner);
   std::cout<<"pilot "<<ep+1<<"/36 seconds="<<r.seconds<<" transitions="<<learner.transitions<<" updates="<<learner.online.updates<<'\n'<<std::flush;
  }
  learner.online.save((dir/"pilot.bin").string());std::ofstream s(dir/"summary.json");s<<std::setprecision(17)<<"{\"episodes\":36,\"seconds\":"<<sum<<",\"mean_episode_seconds\":"<<sum/36<<",\"transitions\":"<<learner.transitions<<",\"updates\":"<<learner.online.updates<<",\"failed_episodes\":"<<failed<<",\"last_loss\":"<<learner.last_loss<<",\"last_gradient\":"<<learner.last_gradient<<",\"max_abs_q\":"<<learner.max_abs_q<<",\"same_time_transitions\":"<<learner.same_time<<",\"terminal_transitions\":"<<learner.terminals<<"}\n";return 0;
 }
 if(argc==8&&std::string(argv[1])=="train"){
  const long seed=std::stol(argv[2]),block=std::stol(argv[3]);double lr=std::stod(argv[4]);const int episodes=std::stoi(argv[5]),interval=std::stoi(argv[6]);fs::path dir=argv[7];
  if(episodes<=0||episodes>1800||interval<=0||(lr!=.001&&lr!=.0003))throw std::runtime_error("training budget outside design");
  fs::create_directories(dir);std::ofstream log(dir/"episodes.csv"),val(dir/"validation.csv"),curve(dir/"checkpoints.csv");log<<std::setprecision(17);val<<std::setprecision(17);curve<<std::setprecision(17);header(log);header(val);curve<<"episode,updates,transitions,validation_awt,best\n";
  Learner learner(seed,lr);std::mt19937_64 order(seed^0xabcd983aULL);std::array<int,9> permutation;std::iota(permutation.begin(),permutation.end(),0);double best=std::numeric_limits<double>::infinity();int best_ep=-1,last=0;
  for(int ep=1;ep<=episodes&&learner.online.updates<100000;++ep){if((ep-1)%9==0)std::shuffle(permutation.begin(),permutation.end(),order);Cell c=cells[permutation[(ep-1)%9]];
   auto r=episode(learner,c,block+ep-1,true);row(log,"train",seed,ep,block+ep-1,c,r,learner);last=ep;
   if(ep%interval==0||ep==episodes||learner.online.updates>=100000){double score=validate(learner,seed,ep,val);bool improved=score<best;if(improved){best=score;best_ep=ep;learner.online.save((dir/"best.bin").string());}
    learner.online.save((dir/("checkpoint-"+std::to_string(ep)+".bin")).string());curve<<ep<<','<<learner.online.updates<<','<<learner.transitions<<','<<score<<','<<improved<<'\n';curve.flush();std::cout<<"seed="<<seed<<" ep="<<ep<<" updates="<<learner.online.updates<<" val_awt="<<score<<" best_ep="<<best_ep<<'\n'<<std::flush;
   }
  }
  std::ofstream s(dir/"summary.json");s<<std::setprecision(17)<<"{\"seed\":"<<seed<<",\"lr\":"<<lr<<",\"episodes\":"<<last<<",\"updates\":"<<learner.online.updates<<",\"transitions\":"<<learner.transitions<<",\"selected_episode\":"<<best_ep<<",\"selected_validation_awt\":";
  if(std::isfinite(best))s<<best;else s<<"null";s<<"}\n";return best_ep<0?3:0;
 }
 if(argc==5&&std::string(argv[1])=="test"){
  const long seed=std::stol(argv[2]);Learner learner(seed);learner.online.load(argv[3]);std::ofstream log(argv[4]);log<<std::setprecision(17);header(log);
  for(int label=91000;label<91030;++label)for(auto c:cells){auto r=episode(learner,c,label,false);row(log,"test",seed,0,label,c,r,learner);}
  return 0;
 }
 if(argc==4&&std::string(argv[1])=="comparators"){
  std::string method=argv[2];if(method!="sequential_greedy"&&method!="paper2017_ls"&&method!="paper2017_ls_lk")throw std::runtime_error("unregistered comparator");std::ofstream log(argv[3]);log<<std::setprecision(17);header(log);Learner dummy(0);
  for(int label=91000;label<91030;++label)for(auto c:cells){Building b;b.floors=10;auto ps=interfloor_traffic(10,c.rate,30,3600,traffic_seed(label,c));auto start=std::chrono::steady_clock::now();Simulation sim(b,c.cars,make_extended_dispatcher(method),std::move(ps));sim.run();
   Run r{sim.metrics(),std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()};row(log,method,-1,0,label,c,r,dummy);}return 0;
 }
 throw std::runtime_error("usage: dqn_runner pilot DIR | train SEED LABEL_BLOCK LR EPISODES VAL_INTERVAL DIR | test SEED MODEL CSV | comparators METHOD CSV");
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}}
