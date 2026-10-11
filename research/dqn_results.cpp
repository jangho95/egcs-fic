#include "json.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <map>
#include <numeric>
#include <sstream>
#include <tuple>
using Key=std::tuple<int,int,int>;
struct Row{int n=0,unserved=0,unfinished=0;double awt=0,p95=0,att=0,ajt=0;};
records::Json json_file(const std::string& p){std::ifstream in(p);if(!in)throw std::runtime_error("missing "+p);return records::parse_json({std::istreambuf_iterator<char>(in),{}});}
std::map<Key,Row> read(const std::string& p){std::ifstream in(p);if(!in)throw std::runtime_error("missing "+p);std::string line;std::getline(in,line);std::map<Key,Row> out;
 while(std::getline(in,line)){if(line.empty())continue;std::istringstream s(line);std::vector<std::string> v;std::string t;while(std::getline(s,t,','))v.push_back(t);if(v.size()!=23)throw std::runtime_error("invalid results CSV");
  Key k{std::stoi(v[3]),std::stoi(v[4]),std::stoi(v[5])};Row r{std::stoi(v[6]),std::stoi(v[7]),std::stoi(v[8]),std::stod(v[9]),std::stod(v[10]),std::stod(v[11]),std::stod(v[12])};
  if(!out.emplace(k,r).second)throw std::runtime_error("duplicate result");
 }if(out.size()!=270)throw std::runtime_error("incomplete nine-cell test");return out;
}
int main(int argc,char** argv){try{std::cout<<std::setprecision(17);
 if(argc==4&&std::string(argv[1])=="--choose"){
  auto a=json_file(argv[2]),b=json_file(argv[3]);if(a.at("seed").integer()!=81000||b.at("seed").integer()!=81000||a.at("selected_episode").integer()<0||b.at("selected_episode").integer()<0)throw std::runtime_error("no valid tuned model");
  std::cout<<(b.at("selected_validation_awt").number<a.at("selected_validation_awt").number?b.at("lr").number:a.at("lr").number)<<'\n';return 0;
 }
 if(argc==10&&std::string(argv[1])=="--analyze"){
  std::array<std::map<Key,Row>,8> data;for(int i=0;i<8;++i)data[i]=read(argv[i+2]);
  long failures=0;for(const auto& [key,r]:data[0])for(const auto& d:data){const auto& q=d.at(key);if(q.n!=r.n||q.n<1)throw std::runtime_error("different passenger stream");failures+=q.unserved||q.unfinished;}
  if(failures){std::cout<<"{\"failed_runs\":"<<failures<<",\"complete_comparison\":false}\n";return 3;}
  const std::array<const char*,8> names{"dqn_82000","dqn_82001","dqn_82002","dqn_82003","dqn_82004","sequential_greedy","paper2017_ls","paper2017_ls_lk"};
  std::array<double,8> means{};std::cout<<"{\"cells\":9,\"test_labels\":[91000,91029],\"training_models\":5,\"runs\":2160,\"failed_runs\":0,\"policy_means\":[";
  for(int i=0;i<8;++i){double awt=0,p95=0,att=0,ajt=0;for(const auto& [key,r]:data[i]){(void)key;awt+=r.awt;p95+=r.p95;att+=r.att;ajt+=r.ajt;}means[i]=awt/270;
   if(i)std::cout<<',';
   std::cout<<"{\"name\":\""<<names[i]<<"\",\"awt\":"<<awt/270<<",\"p95_wait\":"<<p95/270<<",\"att\":"<<att/270<<",\"ajt\":"<<ajt/270<<'}';
  }
  double ensemble_awt=0;for(int i=0;i<5;++i)ensemble_awt+=means[i]/5;
  std::array<double,30> differences{};std::cout<<"],\"dqn_mean_awt\":"<<ensemble_awt<<",\"seed_differences\":[";
  for(int seed=91000;seed<91030;++seed){double d=0;for(int c:{2,4,6})for(int rate:{10,20,30}){Key key{seed,c,rate};double avg=0;for(int i=0;i<5;++i)avg+=data[i].at(key).awt/5;d+=data[7].at(key).awt-avg;}
   differences[seed-91000]=d/9;if(seed!=91000)std::cout<<',';std::cout<<d/9;
  }
  const double mean=std::accumulate(differences.begin(),differences.end(),0.0)/30;double variance=0;for(double d:differences)variance+=(d-mean)*(d-mean);const double se=std::sqrt(variance/29/30),t=2.045229642132703;
  std::cout<<"],\"primary_proposed_minus_dqn\":{\"mean\":"<<mean<<",\"standard_error\":"<<se<<",\"df\":29,\"ci95\":["<<mean-t*se<<','<<mean+t*se<<"],\"scope\":\"conditional on five learned models, paired traffic labels averaged equally across nine cells\"},\"dqn_model_awt_range\":["<<*std::min_element(means.begin(),means.begin()+5)<<','<<*std::max_element(means.begin(),means.begin()+5)<<"],\"cell_means\":[";
  bool first=true;for(int c:{2,4,6})for(int rate:{10,20,30}){if(!first)std::cout<<',';first=false;std::cout<<"{\"cars\":"<<c<<",\"rate\":"<<rate<<",\"awt\":[";
   for(int i=0;i<8;++i){double sum=0;for(int seed=91000;seed<91030;++seed)sum+=data[i].at(Key{seed,c,rate}).awt;if(i)std::cout<<',';std::cout<<sum/30;}std::cout<<"]}";
  }std::cout<<"]}\n";return 0;
 }
 throw std::runtime_error("usage: dqn_results --choose tuning-summary1 tuning-summary2 | --analyze five-dqn-csv three-comparator-csv");
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}}
