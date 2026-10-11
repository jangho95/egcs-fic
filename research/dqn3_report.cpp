#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <numeric>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <tuple>
#include <vector>
namespace fs=std::filesystem;
std::string read(const fs::path& p){std::ifstream in(p);if(!in)throw std::runtime_error("missing input "+p.string());return {std::istreambuf_iterator<char>(in),{}};}
double number(const std::string& text,const std::string& key) {
 std::regex pattern("\""+key+"\"\\s*:\\s*([-+0-9.eE]+)");std::smatch m;
 if(!std::regex_search(text,m,pattern))throw std::runtime_error("missing numeric JSON key "+key);
 double value=std::stod(m[1]);if(!std::isfinite(value))throw std::runtime_error("nonfinite summary");return value;
}
std::vector<std::string> split(const std::string& s){std::vector<std::string> f;std::istringstream in(s);std::string v;while(std::getline(in,v,','))f.push_back(v);return f;}
using Values=std::array<double,4>;
using Key=std::tuple<int,int,long>;
struct Sum {Values total{};int n=0;void add(const Values& v){for(size_t i=0;i<4;++i)total[i]+=v[i];++n;}Values mean()const{if(!n)throw std::runtime_error("empty average");auto v=total;for(double& x:v)x/=n;return v;}};
const std::array<std::pair<int,int>,9> cells{{{2,10},{2,20},{2,30},{4,10},{4,20},{4,30},{6,10},{6,20},{6,30}}};
struct Contrast {double mean=0,low=0,high=0;};
Contrast interval(const std::vector<double>& x) {
 if(x.size()!=30)throw std::runtime_error("contrast requires 30 labels");
 double mean=std::accumulate(x.begin(),x.end(),0.0)/x.size(),variance=0;
 for(double v:x)variance+=(v-mean)*(v-mean)/(x.size()-1);
 double margin=2.045229642132703*std::sqrt(variance/x.size());return {mean,mean-margin,mean+margin};
}
void report(const fs::path& root,const fs::path& texfile) {
 std::map<std::string,std::map<Key,Sum>> policy;
 std::map<std::string,Values> individual;std::map<Key,int> passenger_n;int files=0,runs=0,failures=0;
 for(const auto& entry:fs::directory_iterator(root/"test")) {
  if(entry.path().extension()!=".csv")continue;
  std::string stem=entry.path().stem().string(),method=stem;
  if(stem.starts_with("dqn3-"))method="dqn3";
  if(stem.starts_with("dqn2-"))method="dqn2";
  if(stem.starts_with("warm-"))method="warm";
  if(method!="dqn3"&&method!="dqn2"&&method!="warm"&&method!="route_rule"&&method!="sequential_greedy"&&method!="paper2017_ls"&&method!="paper2017_ls_lk")throw std::runtime_error("unexpected test CSV");
  std::ifstream in(entry.path());std::string line;std::getline(in,line);int count=0;Sum overall;std::set<Key> unique;
  while(std::getline(in,line)) {
   auto f=split(line);if(f.size()!=24)throw std::runtime_error("invalid test row");
   long label=std::stol(f[3]);int cars=std::stoi(f[4]),rate=std::stoi(f[5]),n=std::stoi(f[6]);
   if(label<98000||label>=98030||std::find(cells.begin(),cells.end(),std::pair{cars,rate})==cells.end())throw std::runtime_error("test outside fixed domain");
   Key key{cars,rate,label};if(!unique.insert(key).second)throw std::runtime_error("duplicate stream");
   if(passenger_n.contains(key)&&passenger_n.at(key)!=n)throw std::runtime_error("unpaired passenger stream");passenger_n[key]=n;
   failures+=std::stoi(f[7])+std::stoi(f[8]);Values v{std::stod(f[9]),std::stod(f[10]),std::stod(f[11]),std::stod(f[12])};
   policy[method][key].add(v);overall.add(v);++count;++runs;
  }
  if(count!=270)throw std::runtime_error("incomplete policy evaluation");individual[stem]=overall.mean();++files;
 }
 if(files!=19||runs!=5130||failures)throw std::runtime_error("incomplete or undrained test grid");
 for(const auto& [name,grid]:policy) {
  if(grid.size()!=270)throw std::runtime_error("incomplete pooled grid");
  for(const auto& [key,total]:grid){(void)key;int expected=(name=="dqn3"||name=="dqn2"||name=="warm")?5:1;if(total.n!=expected)throw std::runtime_error("missing model in stream");}
 }
 auto cell_mean=[&](const std::string& method,int cars,int rate) {
  Sum s;for(long label=98000;label<98030;++label)s.add(policy.at(method).at(Key{cars,rate,label}).mean());return s.mean();
 };
 std::map<std::string,Values> levels;
 for(const auto& [name,grid]:policy){Sum s;for(const auto& [key,total]:grid){(void)key;s.add(total.mean());}levels[name]=s.mean();}
 std::map<std::string,Contrast> differences;
 for(auto comparator:{"dqn2","warm","route_rule","sequential_greedy","paper2017_ls_lk"}) {
  std::vector<double> labels;
  for(long label=98000;label<98030;++label) {
   double delta=0;for(auto [cars,rate]:cells)delta+=(policy.at("dqn3").at(Key{cars,rate,label}).mean()[0]-policy.at(comparator).at(Key{cars,rate,label}).mean()[0])/9;
   labels.push_back(delta);
  }
  differences[comparator]=interval(labels);
 }
 std::ofstream json(root/"test/summary.json");json<<std::setprecision(17)<<"{\"test_labels\":[98000,98029],\"runs\":"<<runs<<",\"failed_runs\":0,\"levels\":{";
 bool first=true;
 for(const auto& [name,v]:levels) {
  if(!first)json<<',';first=false;
  json<<'"'<<name<<"\":{\"awt\":"<<v[0]<<",\"p95_wait\":"<<v[1]<<",\"att\":"<<v[2]<<",\"ajt\":"<<v[3]<<'}';
 }
 json<<"},\"contrasts_dqn3_minus\":{";first=true;
 for(const auto& [name,d]:differences) {
  if(!first)json<<',';first=false;json<<'"'<<name<<"\":{\"mean\":"<<d.mean<<",\"low\":"<<d.low<<",\"high\":"<<d.high<<'}';
 }
 json<<"},\"cells\":[";first=true;
 for(auto [cars,rate]:cells) {
  if(!first)json<<',';first=false;json<<"{\"cars\":"<<cars<<",\"rate\":"<<rate;
  for(const auto& [name,v]:levels){(void)v;json<<",\""<<name<<"_awt\":"<<cell_mean(name,cars,rate)[0];}json<<'}';
 }
 json<<"],\"training_models\":[";first=true;
 for(long seed=86000;seed<=86004;++seed){if(!first)json<<',';first=false;json<<read(root/("model-"+std::to_string(seed))/"summary.json");}
 json<<"],\"tuning_candidates\":["<<read(root/"tuning/aux-0/summary.json")<<','<<read(root/"tuning/aux-0.1/summary.json")<<"]}\n";
 std::ofstream tex(texfile);tex<<std::fixed<<std::setprecision(3)
  <<"\\begingroup\\small\n\\renewcommand{\\thetable}{F\\arabic{table}}\\renewcommand{\\theHtable}{appendixF.\\arabic{table}}\\setcounter{table}{7}\n"
  <<"\\begin{longtable}{@{}lrrrr@{}}\n\\caption{Route-guided Dueling Double DQN on new labels 98000--98029: seconds, nine equally weighted cells and 30 paired streams per cell. Warmup-only evaluates the same five initializations before reinforcement learning; the frozen second-study models are reevaluated on these streams. Model means are summaries, not ensemble policies.}\\label{tab:dqn3-levels}\\\\\n"
  <<"\\toprule Policy & AWT & P95 wait & ATT & AJT \\\\ \\midrule\\endfirsthead\n\\toprule Policy (continued) & AWT & P95 wait & ATT & AJT \\\\ \\midrule\\endhead\n";
 auto row=[&](const std::string& name,const Values& v){tex<<name;for(double x:v)tex<<" & "<<x;tex<<" \\\\\n";};
 for(long seed=86000;seed<=86004;++seed)row("Guided D3QN seed "+std::to_string(seed),individual.at("dqn3-"+std::to_string(seed)));
 row("Guided D3QN five-model mean",levels.at("dqn3"));row("Warmup-only five-model mean",levels.at("warm"));row("Frozen DQN2 five-model mean",levels.at("dqn2"));
 row("Route rule",levels.at("route_rule"));row("Sequential insertion",levels.at("sequential_greedy"));row("Uncorrected search",levels.at("paper2017_ls"));row("Corrected search",levels.at("paper2017_ls_lk"));
 tex<<"\\bottomrule\n\\end{longtable}\n\\begin{longtable}{@{}rr rrrrr@{}}\n"
  <<"\\caption{AWT by cell in the additional development study, using the same new test streams for every policy. All learned policies equally average their five models.}\\label{tab:dqn3-cells}\\\\\n"
  <<"\\toprule Cars & Rate (\\%) & DQN2 & Guided D3QN & Warmup only & Route rule & Insertion \\\\ \\midrule\\endfirsthead\n"
  <<"\\toprule Cars & Rate (\\%) & DQN2 & Guided D3QN & Warmup only & Route rule & Insertion \\\\ \\midrule\\endhead\n";
 for(auto [cars,rate]:cells) {
  tex<<cars<<" & "<<rate;
  for(auto name:{"dqn2","dqn3","warm","route_rule","sequential_greedy"})tex<<" & "<<cell_mean(name,cars,rate)[0];tex<<" \\\\\n";
 }
 tex<<"\\bottomrule\n\\end{longtable}\n\\begin{longtable}{@{}rrrrrrr@{}}\n"
  <<"\\caption{Actual budgets and validation selection. Each initialization completes 90 warmup and 3,600 reinforcement-learning episodes. Recorded seconds are per-process episode totals; validation is separate.}\\label{tab:dqn3-training}\\\\\n"
  <<"\\toprule Seed & Warm updates & RL updates & Selected episode & Warm (s) & RL (s) & Valid. (s) \\\\ \\midrule\\endfirsthead\n"
  <<"\\toprule Seed & Warm updates & RL updates & Selected episode & Warm (s) & RL (s) & Valid. (s) \\\\ \\midrule\\endhead\n";
 for(long seed=86000;seed<=86004;++seed) {
  const auto s=read(root/("model-"+std::to_string(seed))/"summary.json");
  tex<<seed<<" & "<<long(number(s,"warmup_updates"))<<" & "<<long(number(s,"updates"))<<" & "<<long(number(s,"selected_episode"))<<" & "<<number(s,"warmup_seconds")<<" & "<<number(s,"training_seconds")<<" & "<<number(s,"validation_seconds")<<" \\\\\n";
 }
 tex<<"\\bottomrule\n\\end{longtable}\n\\endgroup\n";
 std::cout<<std::setprecision(9)<<"AWT: DQN2="<<levels.at("dqn2")[0]<<" guidedD3QN="<<levels.at("dqn3")[0]<<" warmupOnly="<<levels.at("warm")[0]<<" routeRule="<<levels.at("route_rule")[0]<<" insertion="<<levels.at("sequential_greedy")[0]<<" corrected="<<levels.at("paper2017_ls_lk")[0]<<'\n';
}
int main(int argc,char** argv){try {
 if(argc==4&&std::string(argv[1])=="choose") {
  const auto a=read(argv[2]),b=read(argv[3]);std::cout<<std::setprecision(2)<<(number(a,"selected_validation_awt")<=number(b,"selected_validation_awt")?number(a,"auxiliary"):number(b,"auxiliary"))<<'\n';return 0;
 }
 if(argc==4&&std::string(argv[1])=="report"){report(argv[2],argv[3]);return 0;}
 throw std::runtime_error("usage: choose SUMMARY0 SUMMARY1 | report RESULT_DIR TABLE_TEX");
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}}
