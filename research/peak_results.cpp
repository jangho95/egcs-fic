#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <numeric>
#include <sstream>
#include <tuple>
#include <vector>
struct Row{int n,unserved,unfinished;double awt,p95,att,ajt;};
using Key=std::tuple<std::string,int,int,int,std::string>;
int main(int argc,char** argv){try{
 if(argc!=2)throw std::runtime_error("usage: peak_results CSV");std::ifstream in(argv[1]);if(!in)throw std::runtime_error("missing CSV");std::string line;std::getline(in,line);std::map<Key,Row> data;long failures=0;
 while(std::getline(in,line)){std::istringstream s(line);std::vector<std::string> v;std::string t;while(std::getline(s,t,','))v.push_back(t);if(v.size()!=14)throw std::runtime_error("bad CSV");
  Key key{v[0],std::stoi(v[1]),std::stoi(v[2]),std::stoi(v[3]),v[4]};Row row{std::stoi(v[5]),std::stoi(v[6]),std::stoi(v[7]),std::stod(v[8]),std::stod(v[9]),std::stod(v[10]),std::stod(v[11])};if(!data.emplace(key,row).second)throw std::runtime_error("duplicate result");failures+=row.unserved||row.unfinished;
 }
 if(data.size()!=1620)throw std::runtime_error("incomplete experiment");std::cout<<std::setprecision(17)<<"{\"runs\":1620,\"failed_runs\":"<<failures;
 if(failures){std::cout<<",\"complete_comparison\":false}\n";return 3;}std::cout<<",\"profiles\":[";bool first=true;
 for(std::string profile:{"up","downt"}){if(!first)std::cout<<',';first=false;std::cout<<"{\"profile\":\""<<profile<<"\",\"policy_means\":[";
  bool mp=true;for(std::string method:{"sequential_greedy","paper2017_ls","paper2017_ls_lk"}){double awt=0,p95=0,att=0,ajt=0;
   for(int seed=92000;seed<92030;++seed)for(int c:{2,4,6})for(int r:{10,20,30}){auto x=data.at(Key{profile,seed,c,r,method});awt+=x.awt;p95+=x.p95;att+=x.att;ajt+=x.ajt;auto original=data.at(Key{profile,seed,c,r,"paper2017_ls"});if(x.n!=original.n)throw std::runtime_error("passenger stream mismatch");}
   if(!mp)std::cout<<',';mp=false;std::cout<<"{\"method\":\""<<method<<"\",\"awt\":"<<awt/270<<",\"p95_wait\":"<<p95/270<<",\"att\":"<<att/270<<",\"ajt\":"<<ajt/270<<'}';
  }
  std::array<double,30> differences{};for(int seed=92000;seed<92030;++seed){double d=0;for(int c:{2,4,6})for(int r:{10,20,30})d+=data.at(Key{profile,seed,c,r,"paper2017_ls_lk"}).awt-data.at(Key{profile,seed,c,r,"paper2017_ls"}).awt;differences[seed-92000]=d/9;}
  double mean=std::accumulate(differences.begin(),differences.end(),0.0)/30,ss=0;for(double x:differences)ss+=(x-mean)*(x-mean);double se=std::sqrt(ss/29/30),width=2.045229642132703*se;
  std::cout<<"],\"corrected_minus_uncorrected_search\":{\"mean\":"<<mean<<",\"ci95\":["<<mean-width<<','<<mean+width<<"],\"df\":29},\"cell_awt\":[";bool fc=true;
  for(int c:{2,4,6})for(int r:{10,20,30}){if(!fc)std::cout<<',';fc=false;std::cout<<"{\"cars\":"<<c<<",\"rate\":"<<r<<",\"values\":[";bool fm=true;for(std::string method:{"sequential_greedy","paper2017_ls","paper2017_ls_lk"}){double sum=0;for(int seed=92000;seed<92030;++seed)sum+=data.at(Key{profile,seed,c,r,method}).awt;if(!fm)std::cout<<',';fm=false;std::cout<<sum/30;}std::cout<<"]}";}
  std::cout<<"]}";
 }std::cout<<"]}\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}}
