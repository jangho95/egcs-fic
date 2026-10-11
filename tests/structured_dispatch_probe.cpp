#include "../research/dispatch_audit_io.hpp"
#include <array>
#include <chrono>
#include <iomanip>

using namespace egcs2017;
using namespace egcs2017::research;
using namespace egcs2017::research::audit;

int main(int argc,char** argv) {
    try {
        int rounds=12;if(argc==2)rounds=std::stoi(argv[1]);
        if(rounds<0||rounds%6)throw std::runtime_error("need a multiple of six timing rounds");
        std::vector<Input> inputs;
        while(true){Input s;if(!read(s))break;inputs.push_back(std::move(s));}
        if(inputs.empty())throw std::runtime_error("no input states");
        const auto opt=locked_ls_options();SubmodularDispatcher original(opt,"paper2017_ls_lk");
        const std::array<PairKernel,3> kernels{PairKernel::Cached,PairKernel::Forward,PairKernel::Combined};
        const std::array<const char*,3> names{"cached","forward","combined"};
        struct Summary {long greedy=0,final=0,returned=0;double matrix=0,cost=0;};
        std::array<Summary,3> stats;
        long locked=0,empty=0,removed=0,removed_labels=0;
        double saved_error=0;
        for(const auto& s:inputs) {
            const auto M=build_matrices(s.b,s.cars,s.calls,s.nc,opt);
            for(size_t i=0;i<M.N.size();++i)for(size_t c=0;c<s.cars.size();++c) {
                saved_error=std::max(saved_error,std::abs(M.u_raw[i][c]-s.u[i][c]));
                saved_error=std::max(saved_error,std::abs(M.u_lk[i][c]-s.r[i][c]));
                for(size_t j=0;j<M.N.size();++j)
                    saved_error=std::max(saved_error,std::abs(M.w[i][j][c]-s.w[i][j][c]));
            }
            DecisionTrace trace;
            const auto after=original.decide(s.b,s.cars,s.calls,s.nc,&trace);
            const int C=static_cast<int>(s.cars.size());
            auto g=greedy_public(M.u,M.w,C,opt),f=g;local_search_public(f,M.u,M.w,C,opt);
            const double cost=g_of_public(f,M.u,M.w,C,opt);
            for(const auto& [i,c,gain]:trace.order) {
                (void)gain;if(g[i]!=c)throw std::runtime_error("greedy/production disagreement");
            }
            if(std::abs(cost-trace.g_final)>1e-9)throw std::runtime_error("LS/production disagreement");
            locked+=std::any_of(M.locked.begin(),M.locked.end(),[](const auto& x){return !x.empty();});
            empty+=M.N.empty();bool any=false;
            for(const auto& call:M.N){bool r=after.at(call.key)<0;any|=r;removed_labels+=r;}
            removed+=any;
            for(int k=0;k<3;++k) {
                const auto d=structured_decide(s.b,s.cars,s.calls,s.nc,kernels[k]);
                stats[k].greedy+=d.greedy!=g;stats[k].final+=d.final!=f;stats[k].returned+=d.after!=after;
                stats[k].matrix=std::max(stats[k].matrix,matrix_error(M,d.matrices));
                stats[k].cost=std::max(stats[k].cost,std::abs(g_of_public(d.final,M.u,M.w,C,opt)-cost));
                for(const auto& call:s.calls)if(call.close_to_service&&!(call.key==s.nc))
                    if(d.after.at(call.key)!=call.assigned)throw std::runtime_error("moved fixed call");
            }
        }
        if(saved_error>1e-7)throw std::runtime_error("saved coefficients changed");
        for(const auto& s:stats)if(s.matrix>1e-7)throw std::runtime_error("structural formula changed cost");
        std::vector<std::array<double,3>> times(rounds);
        auto timed=[&](const Input& s,int k) {
            auto start=std::chrono::steady_clock::now();
            auto d=structured_decide(s.b,s.cars,s.calls,s.nc,kernels[k]);
            double t=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count();
            if(d.after.size()!=s.calls.size())throw std::runtime_error("timed decision lost calls");
            return t;
        };

        const int orders[6][3]={{0,1,2},{2,1,0},{1,2,0},{0,2,1},{2,0,1},{1,0,2}};
        for(int r=0;r<rounds;++r)for(size_t i=0;i<inputs.size();++i)
            for(int j=0;j<3;++j){int k=orders[(r+2*(i%3))%6][j];times[r][k]+=timed(inputs[i],k);}
        std::cout<<std::setprecision(17)<<"{\"states\":"<<inputs.size()<<",\"locked_states\":"<<locked
            <<",\"empty_free_states\":"<<empty<<",\"states_with_capacity_removal\":"<<removed
            <<",\"capacity_removed_labels\":"<<removed_labels<<",\"maximum_saved_input_error\":"<<saved_error
            <<",\"rounds\":"<<rounds<<",\"variants\":[";
        for(int k=0;k<3;++k){if(k)std::cout<<',';double us=0;for(auto t:times)us+=t[k];
            if(rounds)us/=rounds*inputs.size();
            const auto& s=stats[k];
            std::cout<<"{\"name\":\""<<names[k]<<"\",\"greedy_changed\":"<<s.greedy
              <<",\"final_changed\":"<<s.final<<",\"return_changed\":"<<s.returned
              <<",\"maximum_matrix_error\":"<<s.matrix<<",\"maximum_original_cost_difference\":"<<s.cost
              <<",\"full_mean_us\":"<<us<<'}';}
        std::cout<<"],\"round_totals_us\":[";
        for(int r=0;r<rounds;++r){if(r)std::cout<<',';std::cout<<'['<<times[r][0]<<','<<times[r][1]<<','<<times[r][2]<<']';}
        std::cout<<"]}\n";
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}
}
