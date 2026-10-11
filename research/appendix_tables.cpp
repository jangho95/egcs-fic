#include "json.hpp"
#include <fstream>
#include <iomanip>
#include <iterator>
#include <iostream>
#include <string>
records::Json read(const std::string& p){std::ifstream in(p);if(!in)throw std::runtime_error("missing input");return records::parse_json({std::istreambuf_iterator<char>(in),{}});}
void end(std::ostream& out){out<<"\\bottomrule\n\\end{longtable}\n";}
int main(int argc,char** argv){try{
 if(argc!=4)throw std::runtime_error("usage: appendix_tables DQN_JSON PEAK_JSON OUTPUT_DIR");auto d=read(argv[1]),p=read(argv[2]);std::string dir=argv[3];
 std::ofstream out(dir+"/dqn-results-table.tex");out<<std::fixed<<std::setprecision(3)<<"\\begingroup\\small\n\\renewcommand{\\thetable}{F\\arabic{table}}\\renewcommand{\\theHtable}{appendixF.\\arabic{table}}\\setcounter{table}{0}\n\\begin{longtable}{@{}lrrrr@{}}\n\\caption{Common-engine test outcomes in seconds: nine equally weighted cells and 30 labels. DQN rows refer to validation-selected models; the five-model mean is an average across models, not one new policy.}\\label{tab:dqn-levels}\\\\\n\\toprule Policy & AWT & P95 wait & ATT & AJT \\\\ \\midrule\\endfirsthead\n\\toprule Policy (continued) & AWT & P95 wait & ATT & AJT \\\\ \\midrule\\endhead\n";
 double awt=0,p95=0,att=0,ajt=0;
 for(int i=0;i<8;++i){const auto& r=d.at("policy_means").at(i);std::string name=i<5?"DQN seed "+std::to_string(82000+i):i==5?"Sequential insertion":i==6?"Uncorrected search":"Corrected search";
  if(i<5){awt+=r.at("awt").number/5;p95+=r.at("p95_wait").number/5;att+=r.at("att").number/5;ajt+=r.at("ajt").number/5;}
  out<<name<<" & "<<r.at("awt").number<<" & "<<r.at("p95_wait").number<<" & "<<r.at("att").number<<" & "<<r.at("ajt").number<<" \\\\\n";
  if(i==4)out<<"DQN five-model mean & "<<awt<<" & "<<p95<<" & "<<att<<" & "<<ajt<<" \\\\\n";
 }end(out);
 out<<"\\begin{longtable}{@{}rr rrr r@{}}\n\\caption{Cell-level AWT means: DQN averages five learned models and 30 test labels; each comparator averages the same 30 labels. These cells are descriptive summaries.}\\label{tab:dqn-cells}\\\\\n\\toprule Cars & Rate (\\%) & Insertion & Uncorrected & Corrected & DQN mean \\\\ \\midrule\\endfirsthead\n\\toprule Cars & Rate (\\%) & Insertion & Uncorrected & Corrected & DQN mean \\\\ \\midrule\\endhead\n";
 for(const auto& c:d.at("cell_means").array){double mean=0;for(int i=0;i<5;++i)mean+=c.at("awt").at(i).number/5;out<<c.at("cars").integer()<<" & "<<c.at("rate").integer();for(int i=5;i<8;++i)out<<" & "<<c.at("awt").at(i).number;out<<" & "<<mean<<" \\\\\n";}end(out);out<<"\\endgroup\n";
 std::ofstream peak(dir+"/peak-results-table.tex");peak<<std::fixed<<std::setprecision(3)<<"\\begingroup\\small\n\\renewcommand{\\thetable}{G\\arabic{table}}\\renewcommand{\\theHtable}{appendixG.\\arabic{table}}\\setcounter{table}{0}\n\\begin{longtable}{@{}llrrrr@{}}\n\\caption{Fixed-policy peak-profile outcomes in seconds: nine equally weighted cells and 30 paired labels per profile. Each policy serves the same passenger streams.}\\label{tab:peak-levels}\\\\\n\\toprule Profile & Policy & AWT & P95 wait & ATT & AJT \\\\ \\midrule\\endfirsthead\n\\toprule Profile & Policy & AWT & P95 wait & ATT & AJT \\\\ \\midrule\\endhead\n";
 for(const auto& profile:p.at("profiles").array){int i=0;for(const auto& r:profile.at("policy_means").array){std::string name=i==0?"Insertion":i==1?"Uncorrected search":"Corrected search";++i;peak<<(profile.at("profile").string=="up"?"Up":"Down")<<" & "<<name<<" & "<<r.at("awt").number<<" & "<<r.at("p95_wait").number<<" & "<<r.at("att").number<<" & "<<r.at("ajt").number<<" \\\\\n";}}end(peak);
 peak<<"\\begin{longtable}{@{}lrr rrr@{}}\n\\caption{Peak-profile cell AWT means over 30 labels.}\\label{tab:peak-cells}\\\\\n\\toprule Profile & Cars & Rate (\\%) & Insertion & Uncorrected & Corrected \\\\ \\midrule\\endfirsthead\n\\toprule Profile & Cars & Rate (\\%) & Insertion & Uncorrected & Corrected \\\\ \\midrule\\endhead\n";
 for(const auto& profile:p.at("profiles").array)for(const auto& c:profile.at("cell_awt").array){peak<<(profile.at("profile").string=="up"?"Up":"Down")<<" & "<<c.at("cars").integer()<<" & "<<c.at("rate").integer();for(const auto& v:c.at("values").array)peak<<" & "<<v.number;peak<<" \\\\\n";}end(peak);peak<<"\\endgroup\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}}
