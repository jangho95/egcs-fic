#include "json.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <string>
records::Json read(const std::string& path){std::ifstream in(path);if(!in)throw std::runtime_error("missing "+path);return records::parse_json({std::istreambuf_iterator<char>(in),{}});}
int main(int argc,char** argv){try{
 if(argc!=4)throw std::runtime_error("usage: report DQN2-RESULTS TABLE-TEX BUDGET-JSON");
 std::string root=argv[1];auto test=read(root+"/test/summary.json");const auto& policies=test.at("policy_means").array;
 std::ofstream table(argv[2]),budget(argv[3]);table<<std::fixed<<std::setprecision(3);budget<<std::setprecision(17);
 table<<"\\begingroup\\small\n\\renewcommand{\\thetable}{F\\arabic{table}}\\renewcommand{\\theHtable}{appendixF.\\arabic{table}}\\setcounter{table}{2}\n";
 table<<"\\begin{longtable}{@{}lrrrr@{}}\n\\caption{Follow-up DQN test outcomes in seconds: nine equally weighted cells and 30 new labels. P95 averages within-run quantiles. The five-model mean is a summary, not an ensemble policy.}\\label{tab:dqn2-levels}\\\\\n\\toprule Policy & AWT & P95 wait & ATT & AJT \\\\ \\midrule\\endfirsthead\n\\toprule Policy (continued) & AWT & P95 wait & ATT & AJT \\\\ \\midrule\\endhead\n";
 double mean_awt=0,mean_p95=0,mean_att=0,mean_ajt=0;
 for(int i=0;i<8;++i){const auto& row=policies.at(i);if(i<5)table<<"DQN seed "<<84000+i;else table<<(i==5?"Sequential insertion":i==6?"Uncorrected search":"Corrected search");
  table<<" & "<<row.at("awt").number<<" & "<<row.at("p95_wait").number<<" & "<<row.at("att").number<<" & "<<row.at("ajt").number<<" \\\\\n";
  if(i<5){mean_awt+=row.at("awt").number/5;mean_p95+=row.at("p95_wait").number/5;mean_att+=row.at("att").number/5;mean_ajt+=row.at("ajt").number/5;}
  if(i==4)table<<"DQN five-model mean & "<<mean_awt<<" & "<<mean_p95<<" & "<<mean_att<<" & "<<mean_ajt<<" \\\\\n";
 }
 table<<"\\bottomrule\n\\end{longtable}\n\\begin{longtable}{@{}rr rrr r@{}}\n\\caption{Follow-up cell-level AWT means on labels 96000--96029. Each comparator and all five learned models use the same streams.}\\label{tab:dqn2-cells}\\\\\n\\toprule Cars & Rate (\\%) & Insertion & Uncorrected & Corrected & DQN mean \\\\ \\midrule\\endfirsthead\n\\toprule Cars & Rate (\\%) & Insertion & Uncorrected & Corrected & DQN mean \\\\ \\midrule\\endhead\n";
 for(const auto& cell:test.at("cell_means").array){const auto& values=cell.at("awt").array;double dqn=0;for(int i=0;i<5;++i)dqn+=values.at(i).number/5;table<<cell.at("cars").integer()<<" & "<<cell.at("rate").integer()<<" & "<<values.at(5).number<<" & "<<values.at(6).number<<" & "<<values.at(7).number<<" & "<<dqn<<" \\\\\n";}
 table<<"\\bottomrule\n\\end{longtable}\n\\begin{longtable}{@{}rrrrrr@{}}\n\\caption{Completed follow-up training budgets and validation selection. Times are measured per-process episode totals, not inference latency or total wall time.}\\label{tab:dqn2-training}\\\\\n\\toprule Seed & Episodes & Updates & Selected episode & Training (s) & Validation (s) \\\\ \\midrule\\endfirsthead\n\\toprule Seed & Episodes & Updates & Selected episode & Training (s) & Validation (s) \\\\ \\midrule\\endhead\n";
 budget<<"{\"training_models\":[";for(int i=0;i<5;++i){auto model=read(root+"/model-"+std::to_string(84000+i)+"/summary.json");if(i)budget<<',';
  budget<<"{\"seed\":"<<model.at("seed").integer()<<",\"episodes\":"<<model.at("episodes").integer()<<",\"updates\":"<<model.at("updates").integer()<<",\"transitions\":"<<model.at("transitions").integer()<<",\"selected_episode\":"<<model.at("selected_episode").integer()<<",\"selected_validation_awt\":"<<model.at("selected_validation_awt").number<<",\"training_seconds\":"<<model.at("training_seconds").number<<",\"validation_seconds\":"<<model.at("validation_seconds").number<<",\"failed_training_episodes\":"<<model.at("failed_training_episodes").integer()<<'}';
  table<<model.at("seed").integer()<<" & "<<model.at("episodes").integer()<<" & "<<model.at("updates").integer()<<" & "<<model.at("selected_episode").integer()<<" & "<<model.at("training_seconds").number<<" & "<<model.at("validation_seconds").number<<" \\\\\n";
 }
 budget<<"]}\n";table<<"\\bottomrule\n\\end{longtable}\n\\endgroup\n";return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 2;}}
