#include "json.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <sstream>

using records::Json;
struct Counts {long total=0,enumerated=0,enumerated_positive=0,excluded=0,excluded_positive=0;};
bool too_large(int C,int n,long limit=20000){long states=1;for(int i=0;i<n;++i){if(states>limit/C)return true;states*=C;}return false;}
std::string read(std::istream& in){return {std::istreambuf_iterator<char>(in),std::istreambuf_iterator<char>()};}
Counts count(const Json& root){Counts out;const int C=root.at("meta").at("cars").integer();if(C<1)throw std::runtime_error("invalid car count");
 for(const auto& d:root.at("decisions").array){if(d.at("method").string!="paper2017_ls_lk")throw std::runtime_error("wrong trace method");
  const auto& u=d.at("unary").array;const auto& r=d.at("unary_lk").array;
  if(u.size()!=r.size())throw std::runtime_error("matrix row mismatch");
  if(u.empty())continue;
  bool positive=false;
  for(size_t i=0;i<u.size();++i){if(u[i].array.size()!=static_cast<size_t>(C)||r[i].array.size()!=static_cast<size_t>(C))throw std::runtime_error("matrix width mismatch");
   for(const auto& v:r[i].array){if(v.kind!=Json::NUMBER||v.number<0)throw std::runtime_error("negative/non-numeric correction");positive|=v.number>0;}}
  ++out.total;if(too_large(C,static_cast<int>(u.size()))){++out.excluded;out.excluded_positive+=positive;}
  else{++out.enumerated;out.enumerated_positive+=positive;}
 }return out;
}
int main(int argc,char** argv){try{
 if(argc>1&&std::string(argv[1])=="--selftest"){
  if(too_large(2,14)||!too_large(2,15)||too_large(3,9)||!too_large(3,10)||!too_large(6,1000))throw std::runtime_error("enumeration threshold");
  auto j=records::parse_json("{\"zero\":0e-5,\"tiny\":1e-100,\"text\":\"escaped \\\"word\\\"\",\"skip\":[[1,2],{\"a\":true}]}",{"skip"});
  if(j.at("zero").number!=0||j.at("tiny").number<=0||j.contains("skip"))throw std::runtime_error("number/skip parsing");
  bool rejected=false;try{records::parse_json("[01]");}catch(...){rejected=true;}if(!rejected)throw std::runtime_error("invalid JSON accepted");
  std::cout<<"excluded-counter selftest passed\n";return 0;
 }
 if(argc==4&&std::string(argv[1])=="--aggregate"){
  std::ifstream csv(argv[2]),audit(argv[3]);if(!csv||!audit)throw std::runtime_error("missing aggregate input");
  auto old=records::parse_json(read(audit));const auto& original=old.at("summary").at("overall");
  Counts all;std::map<int,Counts> cars;std::string line;std::getline(csv,line);int files=0;
  while(std::getline(csv,line)){if(line.empty())continue;std::istringstream is(line);std::vector<long> fields;std::string token;while(std::getline(is,token,','))fields.push_back(std::stol(token));
   if(fields.size()!=9)throw std::runtime_error("bad count row");
   Counts a{fields[4],fields[5],fields[6],fields[7],fields[8]};
   for(auto* dest:{&all,&cars[static_cast<int>(fields[1])]}){dest->total+=a.total;dest->enumerated+=a.enumerated;dest->enumerated_positive+=a.enumerated_positive;dest->excluded+=a.excluded;dest->excluded_positive+=a.excluded_positive;}++files;
  }
  if(files!=144||all.enumerated!=original.at("decisions").number||all.enumerated_positive!=original.at("with_locked_terms").number||all.excluded!=old.at("summary").at("skipped_too_large").number)throw std::runtime_error("archival summary mismatch");
  long hits=static_cast<long>(original.at("optimal_sets_disjoint").number);long denom=all.enumerated_positive+all.excluded_positive;
  std::cout<<std::setprecision(17)<<"{\"files\":"<<files<<",\"audit_cells\":48,\"traffic_seeds\":[0,1,2],\"enumeration_limit\":20000,\"total_nonempty\":"<<all.total<<",\"enumerated\":"<<all.enumerated<<",\"enumerated_positive\":"<<all.enumerated_positive<<",\"excluded\":"<<all.excluded<<",\"excluded_positive\":"<<all.excluded_positive<<",\"excluded_zero\":"<<all.excluded-all.excluded_positive<<",\"disjoint_enumerated_positive\":"<<hits<<",\"denominator\":"<<denom<<",\"lower_percent\":"<<100.0*hits/denom<<",\"upper_percent\":"<<100.0*(hits+all.excluded_positive)/denom<<",\"by_cars\":[";
  bool first=true;for(const auto& [C,a]:cars){if(!first)std::cout<<',';first=false;std::cout<<"{\"cars\":"<<C<<",\"excluded\":"<<a.excluded<<",\"excluded_positive\":"<<a.excluded_positive<<'}';}std::cout<<"]}\n";return 0;
 }
 auto j=records::parse_json(read(std::cin),{"pair","events","samples"});auto a=count(j);const auto& m=j.at("meta");
 std::cout<<m.at("floors").integer()<<','<<m.at("cars").integer()<<','<<m.at("rate").integer()<<','<<m.at("seed").integer()<<','<<a.total<<','<<a.enumerated<<','<<a.enumerated_positive<<','<<a.excluded<<','<<a.excluded_positive<<'\n';
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}}
