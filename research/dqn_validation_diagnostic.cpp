#include "dqn2.hpp"
#include "sequential_greedy.hpp"
#include "../src/traffic.hpp"
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <tuple>

using namespace egcs2017;
namespace dq=egcs2017::research::dqn2;
namespace fs=std::filesystem;
struct Cell { int cars,rate; };
const std::array<Cell,9> cells{{{2,10},{2,20},{2,30},{4,10},{4,20},{4,30},{6,10},{6,20},{6,30}}};
unsigned long traffic_seed(long label,Cell c) {
 return static_cast<unsigned long>(label)*1000003UL+10*101+c.cars*7+c.rate*10;
}
struct Counters { long decisions=0,selections=0,reference_matches=0,feature_checks=0; };

class DiagnosticPolicy:public Dispatcher {
 dq::Learner* learner;
 std::unique_ptr<dq::Policy> reference;
 std::ostream* stages;
 long seed,label; Cell cell; double now=0;
public:
 Counters count;
 DiagnosticPolicy(dq::Learner* l,std::ostream* log,long s,long lab,Cell c)
  :learner(l),stages(log),seed(s),label(lab),cell(c) {
  if(l) reference=std::make_unique<dq::Policy>(*l,false);
 }
 std::string name() const override { return learner?"dqn_validation_diagnostic":"raw_eta_reassign"; }
 void observe_learning(double time,double integral,bool terminal) override {
  now=time; if(reference) reference->observe_learning(time,integral,terminal);
 }
 std::map<HallKey,int> decide(const Building& b,const std::vector<CarView>& cars,
  const std::vector<CallView>& calls,const HallKey& nc,DecisionTrace* trace) override {
  ++count.decisions;
  std::map<HallKey,int> out; std::vector<CallView> free;
  for(const auto& cv:calls) {
   if(cv.close_to_service&&cv.assigned>=0) out[cv.key]=cv.assigned;
   else {out[cv.key]=-1;free.push_back(cv);}
  }
  std::sort(free.begin(),free.end(),[](const auto& a,const auto& z) {
   if(a.registered!=z.registered) return a.registered<z.registered;
   return a.key<z.key;
  });
  for(size_t i=0;i<free.size();++i) {
   std::vector<size_t> eligible;std::vector<double> eta,q;
   for(size_t c=0;c<cars.size();++c) if(cars[c].id!=free[i].forbidden_car) {
    eligible.push_back(c);eta.push_back(dq::raw_eta(b,cars[c],free[i].key));
    if(learner) {
     auto x=dq::feature(b,free[i],cars[c],cars,calls,out,int(free.size()-i),now);
     const double normalized=std::clamp(eta.back()/120.0,0.0,10.0);
     if(std::abs(x[89]-normalized)>1e-6) throw std::runtime_error("ETA feature mismatch");
     ++count.feature_checks; q.push_back(learner->online.forward(x));
    }
   }
   if(eligible.empty()) throw std::runtime_error("no eligible diagnostic car");
   size_t eta_best=0,best=0;
   for(size_t c=1;c<eligible.size();++c)
    if(eta[c]<eta[eta_best]||(eta[c]==eta[eta_best]&&cars[eligible[c]].id<cars[eligible[eta_best]].id)) eta_best=c;
   if(learner) {
    for(size_t c=1;c<eligible.size();++c)
     if(q[c]>q[best]||(q[c]==q[best]&&cars[eligible[c]].id<cars[eligible[best]].id)) best=c;
    for(size_t c=0;c<eligible.size();++c) {
     auto x=dq::feature(b,free[i],cars[eligible[c]],cars,calls,out,int(free.size()-i),now);
     if(std::abs(x[97]-std::clamp((eta[c]-eta[eta_best])/120.0,0.0,10.0))>1e-6)
      throw std::runtime_error("ETA excess feature mismatch");
    }
   } else best=eta_best;
   const int chosen=cars[eligible[best]].id;
   out[free[i].key]=chosen; ++count.selections;
   if(stages) {
    auto sorted=q;std::sort(sorted.begin(),sorted.end(),std::greater<double>());
    const bool has_gap=sorted.size()>=2;
    int eta_ties=0;for(double e:eta) eta_ties+=std::abs(e-eta[eta_best])<=1e-9;
    *stages<<seed<<','<<label<<','<<cell.cars<<','<<cell.rate<<','<<count.decisions<<','<<i<<','
     <<free[i].key.floor<<','<<free[i].key.dir<<','<<eligible.size()<<','<<chosen<<','
     <<cars[eligible[eta_best]].id<<','<<(eta[best]-eta[eta_best]<=1e-9)<<','
     <<std::max(0.0,eta[best]-eta[eta_best])<<','<<has_gap<<','
     <<(has_gap?sorted[0]-sorted[1]:0)<<','<<sorted[0]<<','
     <<(has_gap?sorted[1]:0)<<','<<eta_ties<<','
     <<(cars[eligible[best]].load>=std::max(1,b.capacity-2))<<'\n';
   }
  }
  for(const auto& cv:free) {
   int id=out.at(cv.key);
   auto car=std::find_if(cars.begin(),cars.end(),[&](const auto& c){return c.id==id;});
   if(car!=cars.end()&&car->load>=std::max(1,b.capacity-2)) out[cv.key]=-1;
  }
  if(reference) {
   if(out!=reference->decide(b,cars,calls,nc,nullptr)) throw std::runtime_error("frozen policy assignment mismatch");
   ++count.reference_matches;
  }
  if(trace){trace->method=name();trace->after=out;} return out;
 }
};

void episodes(const std::string& method,long seed,const fs::path& checkpoint,const fs::path& dir) {
 if(method!="dqn"&&method!="raw_eta"&&method!="insertion") throw std::runtime_error("invalid method");
 fs::create_directories(dir);
 const auto stem=method=="dqn"?method+"-"+std::to_string(seed):method;
 std::ofstream log(dir/(stem+"-episodes.csv")),stages;
 log<<std::setprecision(17)<<"method,model,label,cars,rate,n,unserved,unfinished,awt,p95_wait,decisions,selections,reference_matches,feature_checks\n";
 if(method=="dqn") {
  stages.open(dir/(stem+"-selections.csv"));
  stages<<std::setprecision(17)<<"model,label,cars,rate,decision,stage,floor,dir,eligible,chosen,eta_car,eta_optimal,eta_excess,has_gap,q_gap,q_best,q_second,eta_ties,postprocessed\n";
 }
 dq::Learner learner(static_cast<unsigned long>(seed));learner.training=false;
 if(method=="dqn") learner.online.load(checkpoint.string());
 const auto before=learner.online.weights;long total_matches=0,total_features=0;
 for(long label=95000;label<=95009;++label) for(auto cell:cells) {
  Building b;b.floors=10;SimOptions o;o.theta=5;o.button_info=true;o.check_invariants=true;
  std::unique_ptr<Dispatcher> dispatcher;DiagnosticPolicy* diagnostic=nullptr;
  if(method=="insertion") dispatcher=make_extended_dispatcher("sequential_greedy");
  else {
   auto p=std::make_unique<DiagnosticPolicy>(method=="dqn"?&learner:nullptr,method=="dqn"?&stages:nullptr,seed,label,cell);
   diagnostic=p.get();dispatcher=std::move(p);
  }
  auto ps=interfloor_traffic(10,cell.rate,30,3600,traffic_seed(label,cell));
  Simulation sim(b,cell.cars,std::move(dispatcher),std::move(ps),nullptr,14400,o);sim.run();auto m=sim.metrics();
  if(m.unserved||m.unfinished) throw std::runtime_error("diagnostic did not drain");
  Counters c;if(diagnostic)c=diagnostic->count;
  total_matches+=c.reference_matches;total_features+=c.feature_checks;
  log<<method<<','<<seed<<','<<label<<','<<cell.cars<<','<<cell.rate<<','<<m.n<<','<<m.unserved<<','<<m.unfinished<<','
   <<m.awt<<','<<m.p95_wait<<','<<m.decisions<<','<<c.selections<<','<<c.reference_matches<<','<<c.feature_checks<<'\n';
  if(learner.online.weights!=before||learner.online.updates||learner.transitions||!learner.replay.empty())
   throw std::runtime_error("diagnostic modified model or learned");
 }
 std::cout<<stem<<" complete: 90 validation streams, reference matches="<<total_matches<<", feature checks="<<total_features<<"\n";
}

std::vector<std::string> split(const std::string& line) {
 std::vector<std::string> fields;std::istringstream in(line);std::string s;
 while(std::getline(in,s,','))fields.push_back(s);
 return fields;
}
struct Aggregate {
 long n=0,strict=0,optimal=0,removed=0,ties=0;
 double excess=0;std::vector<double> gaps,excesses,qlevels;
 void add(const std::vector<std::string>& f) {
  ++n;strict+=f[9]==f[10];optimal+=std::stoi(f[11]);removed+=std::stoi(f[18]);ties+=std::stoi(f[17])>1;
  const double e=std::stod(f[12]);excess+=e;excesses.push_back(e);qlevels.push_back(std::stod(f[15]));
  if(std::stoi(f[13]))gaps.push_back(std::stod(f[14]));
 }
};
double percentile(std::vector<double> x,double p) {
 if(x.empty())throw std::runtime_error("empty quantile sample");
 size_t index=static_cast<size_t>(std::ceil(p*x.size()))-1;
 std::nth_element(x.begin(),x.begin()+index,x.end());return x[index];
}
void aggregate_json(std::ostream& out,const Aggregate& a) {
 out<<"{\"selections\":"<<a.n<<",\"strict_agreement\":"<<double(a.strict)/a.n
  <<",\"eta_optimal_agreement\":"<<double(a.optimal)/a.n<<",\"eta_tied_fraction\":"<<double(a.ties)/a.n
  <<",\"capacity_removed_fraction\":"<<double(a.removed)/a.n<<",\"mean_eta_excess_seconds\":"<<a.excess/a.n
  <<",\"eta_excess_p95_seconds\":"<<percentile(a.excesses,.95)<<",\"q_gap_count\":"<<a.gaps.size()
  <<",\"q_gap_p05\":"<<percentile(a.gaps,.05)<<",\"q_gap_p50\":"<<percentile(a.gaps,.5)
  <<",\"q_gap_p95\":"<<percentile(a.gaps,.95)<<",\"q_best_p50\":"<<percentile(a.qlevels,.5)<<'}';
}
void report(const fs::path& dir,const fs::path& texpath) {
 using Key=std::tuple<std::string,long,int,int,long>;
 std::map<Key,double> awt;std::map<std::pair<int,int>,Aggregate> bycell;
 std::map<long,Aggregate> bymodel;Aggregate all;
 long failed=0,reference_matches=0,feature_checks=0;
 for(auto method:{"insertion","raw_eta","dqn"}) {
  const bool is_dqn=std::string(method)=="dqn";
  for(long seed=is_dqn?84000:-1;seed<=(is_dqn?84004:-1);++seed) {
   auto stem=is_dqn?std::string(method)+"-"+std::to_string(seed):std::string(method);
   std::ifstream in(dir/(stem+"-episodes.csv"));if(!in)throw std::runtime_error("missing episode CSV");
   std::string line;std::getline(in,line);long count=0;
   while(std::getline(in,line)) {
    auto f=split(line);if(f.size()!=14)throw std::runtime_error("invalid episode row");
    const long label=std::stol(f[2]);const int cars=std::stoi(f[3]),rate=std::stoi(f[4]);
    if(label<95000||label>95009)throw std::runtime_error("diagnostic used nonvalidation label");
    if(!awt.emplace(Key{method,seed,cars,rate,label},std::stod(f[8])).second)throw std::runtime_error("duplicate episode");
    failed+=std::stoi(f[6])+std::stoi(f[7]);reference_matches+=std::stol(f[12]);feature_checks+=std::stol(f[13]);++count;
    if(is_dqn&&f[10]!=f[12])throw std::runtime_error("incomplete frozen policy matching");
   }
   if(count!=90)throw std::runtime_error("incomplete validation grid");
   if(is_dqn) {
    std::ifstream stages(dir/(stem+"-selections.csv"));if(!stages)throw std::runtime_error("missing selections");
    std::getline(stages,line);
    while(std::getline(stages,line)) {
     auto f=split(line);if(f.size()!=19||std::stol(f[0])!=seed||std::stol(f[1])<95000||std::stol(f[1])>95009)
      throw std::runtime_error("invalid selection row");
     bycell[{std::stoi(f[2]),std::stoi(f[3])}].add(f);bymodel[seed].add(f);all.add(f);
    }
   }
  }
 }
 if(failed)throw std::runtime_error("failed diagnostic episodes");
 auto mean=[&](const std::string& method,Cell c) {
  double sum=0;long n=0;for(long label=95000;label<=95009;++label)
   for(long seed=method=="dqn"?84000:-1;seed<=(method=="dqn"?84004:-1);++seed){sum+=awt.at(Key{method,seed,c.cars,c.rate,label});++n;}
  return sum/n;
 };
 std::ofstream json(dir/"summary.json");json<<std::setprecision(17)
  <<"{\"analysis\":\"post-hoc validation-only diagnostic\",\"labels\":[95000,95009],\"new_training\":false,\"test_streams_used\":false,"
  <<"\"runs\":630,\"failed_runs\":0,\"reference_assignment_matches\":"<<reference_matches<<",\"feature_checks\":"<<feature_checks<<",\"cells\":[";
 double insertion_mean=0,eta_mean=0,dqn_mean=0;bool first=true;
 for(auto c:cells) {
  double insertion=mean("insertion",c),eta=mean("raw_eta",c),dqn=mean("dqn",c);
  insertion_mean+=insertion/9;eta_mean+=eta/9;dqn_mean+=dqn/9;
  if(!first)json<<',';first=false;
  json<<"{\"cars\":"<<c.cars<<",\"rate\":"<<c.rate<<",\"insertion_awt\":"<<insertion<<",\"raw_eta_awt\":"<<eta<<",\"dqn_awt\":"<<dqn<<",\"choices\":";
  aggregate_json(json,bycell.at({c.cars,c.rate}));json<<'}';
 }
 json<<"],\"models\":[";first=true;
 for(const auto& [seed,a]:bymodel) {
  if(!first)json<<',';first=false;double m=0;
  for(auto c:cells)for(long label=95000;label<=95009;++label)m+=awt.at(Key{"dqn",seed,c.cars,c.rate,label})/90;
  json<<"{\"seed\":"<<seed<<",\"awt\":"<<m<<",\"choices\":";aggregate_json(json,a);json<<'}';
 }
 json<<"],\"insertion_awt\":"<<insertion_mean<<",\"raw_eta_awt\":"<<eta_mean<<",\"dqn_awt\":"<<dqn_mean<<",\"pooled_choices\":";
 aggregate_json(json,all);json<<"}\n";
 std::ofstream tex(texpath);tex<<std::fixed<<std::setprecision(3)
  <<"\\begingroup\\small\n\\renewcommand{\\thetable}{F\\arabic{table}}\\renewcommand{\\theHtable}{appendixF.\\arabic{table}}\\setcounter{table}{5}\n"
  <<"\\begin{longtable}{@{}rr rrr rr@{}}\n"
  <<"\\caption{Post-hoc validation diagnostic on labels 95000--95009. AWT in seconds equally weights streams and the five frozen DQN models. ETA agreement includes ties within $10^{-9}$ s; agreement and Q gaps pool DQN selection stages before capacity postprocessing. Gap units are the training reward units.}\\label{tab:dqn-diagnostic-cells}\\\\\n"
  <<"\\toprule Cars & Rate (\\%) & Insertion & Min ETA & DQN & ETA agree (\\%) & Q gap median \\\\ \\midrule\\endfirsthead\n"
  <<"\\toprule Cars & Rate (\\%) & Insertion & Min ETA & DQN & ETA agree (\\%) & Q gap median \\\\ \\midrule\\endhead\n";
 for(auto c:cells) {
  const auto& a=bycell.at({c.cars,c.rate});
  tex<<c.cars<<" & "<<c.rate<<" & "<<mean("insertion",c)<<" & "<<mean("raw_eta",c)<<" & "<<mean("dqn",c)
   <<" & "<<100.0*a.optimal/a.n<<" & "<<percentile(a.gaps,.5)<<" \\\\\n";
 }
 tex<<"\\bottomrule\n\\end{longtable}\n\\begin{longtable}{@{}rrrrr@{}}\n"
  <<"\\caption{Validation Q gap distributions by frozen model: largest minus second-largest eligible Q value, pooled over each model's selection stages. Percentiles are empirical inverse-CDF quantiles; stages are dependent and these are descriptive summaries.}\\label{tab:dqn-diagnostic-gaps}\\\\\n"
  <<"\\toprule Model & Stages with gap & P5 & Median & P95 \\\\ \\midrule\\endfirsthead\n"
  <<"\\toprule Model & Stages with gap & P5 & Median & P95 \\\\ \\midrule\\endhead\n";
 tex<<std::setprecision(6);
 for(const auto& [seed,a]:bymodel)tex<<seed<<" & "<<a.gaps.size()<<" & "<<percentile(a.gaps,.05)<<" & "<<percentile(a.gaps,.5)<<" & "<<percentile(a.gaps,.95)<<" \\\\\n";
 tex<<"\\bottomrule\n\\end{longtable}\n\\endgroup\n";
 std::cout<<std::setprecision(9)<<"validation AWT: insertion="<<insertion_mean<<", raw ETA="<<eta_mean<<", DQN="<<dqn_mean
  <<"; ETA-optimal agreement="<<double(all.optimal)/all.n<<", Q gap median="<<percentile(all.gaps,.5)<<'\n';
}
int main(int argc,char** argv) {try {
 if(argc==6&&std::string(argv[1])=="model") {episodes("dqn",std::stol(argv[2]),argv[3],argv[4]);
  if(std::string(argv[5])!="validation-only")throw std::runtime_error("explicit validation-only mode required");return 0;}
 if(argc==4&&std::string(argv[1])=="rule") {episodes(argv[2],-1,{},argv[3]);return 0;}
 if(argc==4&&std::string(argv[1])=="report") {report(argv[2],argv[3]);return 0;}
 throw std::runtime_error("usage: model SEED CHECKPOINT DIR validation-only | rule raw_eta/insertion DIR | report DIR TABLE_TEX");
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}}
