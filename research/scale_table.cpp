#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
int main(int argc,char** argv){try{
 if(argc!=3)throw std::runtime_error("usage: scale_table CSV TEX");std::ifstream in(argv[1]);std::ofstream out(argv[2]);if(!in||!out)throw std::runtime_error("missing table file");
 out<<"\\begingroup\\small\\setlength{\\tabcolsep}{4pt}\n\\begin{longtable}{@{}rrrlrrrr@{}}\n\\caption{Constructed larger-system states, with 12 cars and 12 fixed calls each. FF and FL denote supported fractions of free/free and free/fixed pairs. Times are complete-adapter means over six alternating repeats, not field or traffic measurements.}\\label{tab:scale}\\\\\n\\toprule $F$ & $n$ & $v_{\\max}$ & Direction & FF (\\%) & FL (\\%) & Cached (ms) & Combined (ms) \\\\ \\midrule\\endfirsthead\n\\toprule $F$ & $n$ & $v_{\\max}$ & Direction & FF (\\%) & FL (\\%) & Cached (ms) & Combined (ms) \\\\ \\midrule\\endhead\n";
 std::string line;std::getline(in,line);int rows=0;
 while(std::getline(in,line)){std::istringstream s(line);std::vector<std::string> v;std::string t;while(std::getline(s,t,','))v.push_back(t);if(v.size()!=17)throw std::runtime_error("wrong scale row");int direction=std::stoi(v[5]);
  out<<v[0]<<" & "<<v[2]<<" & "<<std::fixed<<std::setprecision(1)<<std::stod(v[4])<<" & "<<(direction==0?"Idle":direction==1?"Up":"Down")<<" & "<<100*std::stod(v[6])/std::stod(v[7])<<" & "<<100*std::stod(v[8])/std::stod(v[9])<<" & "<<std::setprecision(3)<<std::stod(v[15])/1000<<" & "<<std::stod(v[16])/1000<<" \\\\\n";++rows;
 }if(rows!=12)throw std::runtime_error("incomplete scale states");out<<"\\bottomrule\\end{longtable}\\endgroup\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}}
