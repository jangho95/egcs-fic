#include "json.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <string>
#include <vector>
using records::Json;
Json file(const std::string& path){std::ifstream in(path);if(!in)throw std::runtime_error("missing "+path);return records::parse_json({std::istreambuf_iterator<char>(in),{}});}
void scalar(const Json& x){if(x.kind==Json::BOOLEAN)std::cout<<x.boolean;else if(x.kind==Json::NUMBER)std::cout<<x.number;else throw std::runtime_error("invalid protocol value");std::cout<<' ';}
void values(const Json& x){if(x.kind==Json::ARRAY){for(const auto& v:x.array)values(v);}else scalar(x);}
int main(int argc,char** argv){try{
 std::cout<<std::setprecision(17);
 if(argc==3&&std::string(argv[1])=="--manifest"){
  const auto m=file(argv[2]);if(m.at("sources").array.size()!=75||!m.contains("finished"))throw std::runtime_error("incomplete saved grid");
  for(const auto& s:m.at("sources").array)std::cout<<s.at("path").string<<'\n';
  return 0;
 }
 if(argc==2&&std::string(argv[1])=="--tokens"){
  std::string line;while(std::getline(std::cin,line)){if(line.empty())continue;const auto d=records::parse_json(line);
   scalar(d.at("floors"));scalar(d.at("cars"));std::cout<<d.at("calls").array.size()<<' ';
   for(const auto& car:d.at("car_views").array){for(const auto& k:{"id","x","v","dir","moving","arriving","committed","release","doors_open","load"})scalar(car.at(k));
    std::cout<<car.at("car_calls").array.size()<<' ';values(car.at("car_calls"));}
   for(const auto& call:d.at("calls").array){scalar(call.at(0));scalar(call.at(1));}
   for(const auto& locked:d.at("locked").array){std::cout<<locked.array.size()<<' ';values(locked);}
   values(d.at("u_raw"));values(d.at("u_lk"));values(d.at("w"));values(d.at("new_call"));
   for(const auto& call:d.at("calls").array){scalar(call.at(2));scalar(call.at(3));scalar(call.at(4));}std::cout<<'\n';
  }return 0;
 }
 if(argc>3&&std::string(argv[1])=="--aggregate"){
  const auto m=file(argv[2]);if(argc!=78)throw std::runtime_error("expected 75 cell results");std::vector<Json> rows;
  long states=0,locked=0,empty=0,removed=0;double saved=0;
  for(int k=3;k<argc;++k){auto j=file(argv[k]);const auto& s=m.at("sources").at(k-3);
   if(j.at("states").integer()!=s.at("rows").integer()||j.at("rounds").integer()!=30)throw std::runtime_error("replay count mismatch");
   states+=j.at("states").integer();locked+=j.at("locked_states").integer();empty+=j.at("empty_free_states").integer();removed+=j.at("states_with_capacity_removal").integer();saved=std::max(saved,j.at("maximum_saved_input_error").number);rows.push_back(std::move(j));
  }
  std::cout<<"{\"cells\":75,\"states\":"<<states<<",\"locked_states\":"<<locked<<",\"empty_free_states\":"<<empty<<",\"states_with_capacity_removal\":"<<removed<<",\"maximum_saved_input_error\":"<<saved<<",\"rounds\":30,\"variants\":[";
  for(int k=0;k<3;++k){if(k)std::cout<<',';long greedy=0,final=0,returned=0,faster=0;double matrix=0,cost=0,pooled=0,equal=0,ratio=0,lo=1e300,hi=0;
   for(const auto& r:rows){const auto& v=r.at("variants").at(k);greedy+=v.at("greedy_changed").integer();final+=v.at("final_changed").integer();returned+=v.at("return_changed").integer();matrix=std::max(matrix,v.at("maximum_matrix_error").number);cost=std::max(cost,v.at("maximum_original_cost_difference").number);
    double t=v.at("full_mean_us").number,x=t/r.at("variants").at(0).at("full_mean_us").number;pooled+=t*r.at("states").number;equal+=t;ratio+=x;lo=std::min(lo,x);hi=std::max(hi,x);faster+=x<1;}
   if(greedy||final||returned||matrix>1e-7||cost>1e-7)throw std::runtime_error("dispatcher equivalence failed");
   std::cout<<"{\"name\":\""<<rows[0].at("variants").at(k).at("name").string<<"\",\"greedy_changed\":"<<greedy<<",\"final_changed\":"<<final<<",\"return_changed\":"<<returned<<",\"maximum_matrix_error\":"<<matrix<<",\"maximum_original_cost_difference\":"<<cost<<",\"pooled_decision_mean_us\":"<<pooled/states<<",\"equal_cell_mean_us\":"<<equal/75<<",\"mean_cell_ratio_to_cached\":"<<ratio/75<<",\"cell_ratio_range\":["<<lo<<','<<hi<<"],\"cells_faster_than_cached\":"<<faster<<'}';
  }std::cout<<"]}\n";return 0;
 }
 throw std::runtime_error("usage: timing_records --manifest manifest | --tokens | --aggregate manifest results...");
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}}
