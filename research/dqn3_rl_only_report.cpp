#define main dqn3_primary_report_main
#include "dqn3_report.cpp"
#undef main

void choose_rl(const fs::path& root) {
 fs::create_directories(root/"rl-only-test");
 std::ofstream chosen(root/"rl-only-test/selected-models.tsv");
 chosen<<std::setprecision(17)<<"seed\tepisode\tvalidation_awt\tpath\n";
 for(long seed=86000;seed<=86004;++seed) {
  fs::path folder=root/("model-"+std::to_string(seed));
  std::ifstream in(folder/"checkpoints.csv");std::string line;
  std::getline(in,line);int best_episode=-1,count=0;double best=INFINITY;
  while(std::getline(in,line)) {
   auto f=split(line);if(f.size()!=5)throw std::runtime_error("invalid checkpoint row");
   int ep=std::stoi(f[0]);double awt=std::stod(f[3]);
   if(ep==0)continue;
   if(ep%180||ep>3600||!std::isfinite(awt))throw std::runtime_error("invalid RL checkpoint");
   ++count;if(awt<best){best=awt;best_episode=ep;}
  }
  if(count!=20||best_episode<0)throw std::runtime_error("RL training incomplete");
  chosen<<seed<<'\t'<<best_episode<<'\t'<<best<<'\t'<<(folder/("checkpoint-"+std::to_string(best_episode)+".bin")).string()<<'\n';
 }
}
void report_rl(const fs::path& root,const fs::path& texfile) {
 std::map<std::string,std::map<Key,Sum>> grid;
 std::map<std::string,Values> individual;std::map<Key,int> passengers;
 auto load=[&](const fs::path& p,const std::string& method) {
  std::ifstream in(p);if(!in)throw std::runtime_error("missing CSV");
  std::string line;std::getline(in,line);Sum total;std::set<Key> keys;
  while(std::getline(in,line)) {
   auto f=split(line);if(f.size()!=24)throw std::runtime_error("bad row");
   long label=std::stol(f[3]);int cars=std::stoi(f[4]),rate=std::stoi(f[5]),n=std::stoi(f[6]);Key key{cars,rate,label};
   if(label<98000||label>=98030||std::find(cells.begin(),cells.end(),std::pair{cars,rate})==cells.end()||!keys.insert(key).second)throw std::runtime_error("invalid test key");
   if(std::stoi(f[7])||std::stoi(f[8]))throw std::runtime_error("undrained run");
   if(passengers.contains(key)&&passengers.at(key)!=n)throw std::runtime_error("unpaired stream");passengers[key]=n;
   Values v{std::stod(f[9]),std::stod(f[10]),std::stod(f[11]),std::stod(f[12])};
   grid[method][key].add(v);total.add(v);
  }
  if(keys.size()!=270)throw std::runtime_error("incomplete file");individual[p.stem().string()]=total.mean();
 };
 for(long seed=86000;seed<=86004;++seed) {
  load(root/"rl-only-test"/("rl-"+std::to_string(seed)+".csv"),"rl_only");
  load(root/"test"/("warm-"+std::to_string(seed)+".csv"),"warm");
  load(root/"test"/("dqn3-"+std::to_string(seed)+".csv"),"primary");
  load(root/"test"/("dqn2-"+std::to_string(seed-2000)+".csv"),"dqn2");
 }
 for(auto method:{"route_rule","sequential_greedy","paper2017_ls_lk"})load(root/"test"/(std::string(method)+".csv"),method);
 std::map<std::string,Values> levels;
 for(const auto& [name,g]:grid) {
  Sum total;for(const auto& [key,s]:g){(void)key;int expected=(name=="rl_only"||name=="warm"||name=="primary"||name=="dqn2")?5:1;if(s.n!=expected)throw std::runtime_error("missing model");total.add(s.mean());}levels[name]=total.mean();
 }
 std::map<std::string,Contrast> contrasts;
 for(auto method:{"dqn2","warm","primary","route_rule","sequential_greedy","paper2017_ls_lk"}) {
  std::vector<double> values;
  for(long label=98000;label<98030;++label){double d=0;for(auto [cars,rate]:cells)d+=(grid.at("rl_only").at(Key{cars,rate,label}).mean()[0]-grid.at(method).at(Key{cars,rate,label}).mean()[0])/9;values.push_back(d);}
  contrasts[method]=interval(values);
 }
 std::map<long,int> episodes;std::ifstream selected(root/"rl-only-test/selected-models.tsv");std::string line;std::getline(selected,line);
 while(std::getline(selected,line)){std::istringstream in(line);long seed;int ep;double awt;in>>seed>>ep>>awt;episodes[seed]=ep;}
 if(episodes.size()!=5)throw std::runtime_error("missing selections");
 std::ofstream out(root/"rl-only-test/summary.json");out<<std::setprecision(17)<<"{\"selection_added_after_validation\":true,\"test_labels\":[98000,98029],\"additional_runs\":1350,\"failed_runs\":0,\"models\":[";
 bool first=true;for(auto [seed,ep]:episodes){if(!first)out<<',';first=false;auto v=individual.at("rl-"+std::to_string(seed));out<<"{\"seed\":"<<seed<<",\"selected_episode\":"<<ep<<",\"awt\":"<<v[0]<<",\"p95_wait\":"<<v[1]<<",\"att\":"<<v[2]<<",\"ajt\":"<<v[3]<<'}';}
 out<<"],\"levels\":{";first=true;for(auto [name,v]:levels){if(!first)out<<',';first=false;out<<'"'<<name<<"\":{\"awt\":"<<v[0]<<",\"p95_wait\":"<<v[1]<<",\"att\":"<<v[2]<<",\"ajt\":"<<v[3]<<'}';}
 out<<"},\"contrasts_rl_only_minus\":{";first=true;for(auto [name,d]:contrasts){if(!first)out<<',';first=false;out<<'"'<<name<<"\":{\"mean\":"<<d.mean<<",\"low\":"<<d.low<<",\"high\":"<<d.high<<'}';}
 out<<"},\"cells\":[";first=true;
 for(auto [cars,rate]:cells){if(!first)out<<',';first=false;out<<"{\"cars\":"<<cars<<",\"rate\":"<<rate;for(auto [name,g]:grid){Sum s;for(long label=98000;label<98030;++label)s.add(g.at(Key{cars,rate,label}).mean());auto v=s.mean();out<<",\""<<name<<"_awt\":"<<v[0];}out<<'}';}out<<"]}\n";
 std::ofstream tex(texfile);tex<<std::fixed<<std::setprecision(3)<<"\\begingroup\\small\n\\renewcommand{\\thetable}{F\\arabic{table}}\\renewcommand{\\theHtable}{appendixF.\\arabic{table}}\\setcounter{table}{10}\n"
  <<"\\begin{longtable}{@{}lrrrrr@{}}\n\\caption{Additional RL-only sensitivity analysis on labels 98000--98029. Its selection rule was added after examining validation curves and before test evaluation; episode 0 is excluded only here. The primary selection and all learning budgets are unchanged. All five-model means are summaries, not ensembles.}\\label{tab:dqn3-rl-only}\\\\\n"
  <<"\\toprule Policy & RL episode & AWT & P95 wait & ATT & AJT \\\\ \\midrule\\endfirsthead\n"
  <<"\\toprule Policy & RL episode & AWT & P95 wait & ATT & AJT \\\\ \\midrule\\endhead\n";
 auto row=[&](const std::string& name,const std::string& ep,const Values& v){tex<<name<<" & "<<ep;for(double x:v)tex<<" & "<<x;tex<<" \\\\\n";};
 for(auto [seed,ep]:episodes)row("RL-only seed "+std::to_string(seed),std::to_string(ep),individual.at("rl-"+std::to_string(seed)));
 row("RL-only five-model mean","--",levels.at("rl_only"));row("Warmup-only mean","0",levels.at("warm"));row("Frozen DQN2 mean","--",levels.at("dqn2"));row("Route rule","--",levels.at("route_rule"));row("Sequential insertion","--",levels.at("sequential_greedy"));row("Corrected search","--",levels.at("paper2017_ls_lk"));
 tex<<"\\bottomrule\n\\end{longtable}\n\\endgroup\n";
 std::cout<<std::setprecision(9)<<"RL-only AWT="<<levels.at("rl_only")[0]<<" frozenDQN2="<<levels.at("dqn2")[0]<<" warm="<<levels.at("warm")[0]<<'\n';
}
int main(int argc,char** argv){try{if(argc==3&&std::string(argv[1])=="choose"){choose_rl(argv[2]);return 0;}if(argc==4&&std::string(argv[1])=="report"){report_rl(argv[2],argv[3]);return 0;}throw std::runtime_error("usage: choose ROOT | report ROOT TEX");}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}}
