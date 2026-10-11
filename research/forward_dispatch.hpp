#pragma once
#include "forward_pair_batch.hpp"
#include "dispatchers.hpp"

namespace egcs2017::research {

inline CostOptions locked_ls_options() {
    CostOptions o; o.bonus=true; o.capacity_penalty=true;
    o.locked_aware=true; o.search=CostOptions::LS; return o;
}

inline Matrices forward_matrices(const Building& b,const std::vector<CarView>& cars,
        const std::vector<CallView>& calls,HallKey nc,bool hybrid) {
    const auto opt=locked_ls_options();
    Matrices M;
    for(const auto& call:calls) if(!call.close_to_service || call.key==nc) M.N.push_back(call);
    std::stable_sort(M.N.begin(),M.N.end(),[](const auto& x,const auto& y){return x.registered<y.registered;});
    const int n=static_cast<int>(M.N.size()),C=static_cast<int>(cars.size());
    M.locked.resize(C);
    for(const auto& call:calls) if(call.close_to_service && !(call.key==nc) && call.assigned>=0)
        for(int c=0;c<C;++c) if(cars[c].id==call.assigned) M.locked[c].push_back(call.key);
    M.u.assign(n,std::vector<double>(C));M.u_raw=M.u;M.adj=M.u;M.u_lk=M.u;
    M.w.assign(n,std::vector<std::vector<double>>(n,std::vector<double>(C)));
    std::vector<HallKey> keys;
    for(const auto& call:M.N) keys.push_back(call.key);
    for(int c=0;c<C;++c) {
        auto terms=pair_batch(b,cars[c],keys,M.locked[c],hybrid);
        for(int i=0;i<n;++i) {

            double adjusted=unary(b,cars[c],keys[i],opt);
            if(M.N[i].forbidden_car==cars[c].id) adjusted+=1e6;
            M.u_raw[i][c]=terms.raw[i];M.adj[i][c]=adjusted-terms.raw[i];
            M.u_lk[i][c]=terms.correction[i];
            M.u[i][c]=adjusted+terms.correction[i];
            for(int j=0;j<n;++j) M.w[i][j][c]=terms.pairs[i][j];
        }
    }
    return M;
}

struct ForwardDecision {
    Matrices matrices;
    std::vector<int> greedy,final;
    int iterations=0,removed=0;
    double greedy_cost=0,final_cost=0;
    std::map<HallKey,int> after;
};

inline ForwardDecision forward_decide(const Building& b,const std::vector<CarView>& cars,
        const std::vector<CallView>& calls,HallKey nc,bool hybrid) {
    const auto opt=locked_ls_options();
    ForwardDecision out;
    for(const auto& call:calls) out.after[call.key]=call.assigned;
    out.matrices=forward_matrices(b,cars,calls,nc,hybrid);
    const auto& M=out.matrices;
    const int C=static_cast<int>(cars.size()),n=static_cast<int>(M.N.size());
    if(!n) return out;
    out.greedy=greedy_public(M.u,M.w,C,opt); out.final=out.greedy;
    out.greedy_cost=g_of_public(out.greedy,M.u,M.w,C,opt);
    out.iterations=local_search_public(out.final,M.u,M.w,C,opt);
    out.final_cost=g_of_public(out.final,M.u,M.w,C,opt);
    for(int i=0;i<n;++i) {
        const auto& car=cars[out.final[i]];
        const bool removed=car.load>=std::max(1,b.capacity-opt.capacity_margin);
        out.after[M.N[i].key]=removed?-1:car.id;
        out.removed+=removed;
    }
    return out;
}
}
