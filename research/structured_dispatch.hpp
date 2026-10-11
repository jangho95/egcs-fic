#pragma once
#include "forward_dispatch.hpp"
#include "cross_pair_features.hpp"

namespace egcs2017::research {

enum class PairKernel {Cached,Forward,Combined};

inline PairBatch combined_pair_batch(const Building& b,const CarView& car,
        const std::vector<HallKey>& free,const std::vector<HallKey>& locked) {
    const int n=static_cast<int>(free.size());
    PairBatch out;
    out.raw.resize(n);out.correction.assign(n,0);
    out.pairs.assign(n,std::vector<double>(n));
    if(!n)return out;
    CostOptions raw;
    for(int i=0;i<n;++i)out.raw[i]=unary(b,car,free[i],raw);
    std::vector<double> locked_raw;
    for(auto k:locked)locked_raw.push_back(unary(b,car,k,raw));
    std::optional<ForwardPairFeatures> up,down;
    for(int d:{UP,DOWN}) {
        int nf=0,nl=0;
        auto ahead=[&](HallKey k){return k.dir==d&&(k.floor-car.committed)*d>=0;};
        for(auto k:free)nf+=ahead(k);
        for(auto k:locked)nl+=ahead(k);
        if(nf>0&&nf+nl>=2)(d==UP?up:down)=forward_features(b,car,d);
    }
    std::optional<CrossPairFeatures> cross;
    if(car.dir) {
        int nf=0,ns=0,lf=0,ls=0;
        auto first=[&](HallKey k){return k.dir==car.dir&&(k.floor-car.committed)*car.dir>=0;};
        for(auto k:free){nf+=first(k);ns+=k.dir==-car.dir;}
        for(auto k:locked){lf+=first(k);ls+=k.dir==-car.dir;}
        if((nf&&ns)||(nf&&ls)||(ns&&lf))cross=cross_features(b,car);
    }
    auto forward=[&](HallKey a,HallKey z)->const ForwardPairFeatures* {
        const auto& f=a.dir==UP?up:down;
        return f&&f->contains(a)&&f->contains(z)?&*f:nullptr;
    };
    for(int i=0;i<n;++i)for(int j=i+1;j<n;++j) {
        const auto* f=forward(free[i],free[j]);
        const double w=f?f->pair(free[i],free[j]):
            cross&&cross->contains(free[i],free[j])?
                cross->pair(free[i],free[j],out.raw[i],out.raw[j]):
                pairwise(b,car,free[i],free[j],out.raw[i],out.raw[j]);
        out.pairs[i][j]=out.pairs[j][i]=nonnegative_pair(w);
    }
    for(const auto* f:{up?&*up:nullptr,down?&*down:nullptr})if(f) {
        std::vector<HallKey> ns,ls;std::vector<int> ids;
        for(int i=0;i<n;++i)if(f->contains(free[i])){ns.push_back(free[i]);ids.push_back(i);}
        for(auto k:locked)if(f->contains(k))ls.push_back(k);
        if(ns.empty()||ls.empty())continue;
        const auto row=f->correction(ns,ls);
        for(size_t i=0;i<ids.size();++i)out.correction[ids[i]]+=nonnegative_pair(row[i]);
    }
    if(cross&&!locked.empty()) {
        const auto row=cross->correction(free,locked,out.raw,locked_raw);
        for(int i=0;i<n;++i)out.correction[i]+=nonnegative_pair(row[i]);
    }
    for(int i=0;i<n;++i)for(size_t j=0;j<locked.size();++j)
        if(!forward(free[i],locked[j])&&!(cross&&cross->contains(free[i],locked[j])))
            out.correction[i]+=nonnegative_pair(pairwise(b,car,free[i],locked[j],out.raw[i],locked_raw[j]));
    return out;
}

inline Matrices structured_matrices(const Building& b,const std::vector<CarView>& cars,
        const std::vector<CallView>& calls,HallKey nc,PairKernel kernel) {
    const auto opt=locked_ls_options();
    Matrices M;
    for(const auto& call:calls)if(!call.close_to_service||call.key==nc)M.N.push_back(call);
    std::stable_sort(M.N.begin(),M.N.end(),[](const auto& x,const auto& y){return x.registered<y.registered;});
    const int n=static_cast<int>(M.N.size()),C=static_cast<int>(cars.size());
    M.locked.resize(C);
    for(const auto& call:calls)if(call.close_to_service&&!(call.key==nc)&&call.assigned>=0)
        for(int c=0;c<C;++c)if(cars[c].id==call.assigned)M.locked[c].push_back(call.key);
    M.u.assign(n,std::vector<double>(C));M.u_raw=M.u;M.adj=M.u;M.u_lk=M.u;
    M.w.assign(n,std::vector<std::vector<double>>(n,std::vector<double>(C)));
    std::vector<HallKey> keys;
    for(const auto& call:M.N)keys.push_back(call.key);
    for(int c=0;c<C;++c) {
        const auto terms=kernel==PairKernel::Combined?combined_pair_batch(b,cars[c],keys,M.locked[c]):
            pair_batch(b,cars[c],keys,M.locked[c],kernel==PairKernel::Forward);
        for(int i=0;i<n;++i) {
            double adjusted=unary(b,cars[c],keys[i],opt);
            if(M.N[i].forbidden_car==cars[c].id)adjusted+=1e6;
            M.u_raw[i][c]=terms.raw[i];M.adj[i][c]=adjusted-terms.raw[i];
            M.u_lk[i][c]=terms.correction[i];M.u[i][c]=adjusted+terms.correction[i];
            for(int j=0;j<n;++j)M.w[i][j][c]=terms.pairs[i][j];
        }
    }
    return M;
}

inline ForwardDecision structured_decide(const Building& b,const std::vector<CarView>& cars,
        const std::vector<CallView>& calls,HallKey nc,PairKernel kernel) {
    const auto opt=locked_ls_options();
    ForwardDecision out;
    for(const auto& call:calls)out.after[call.key]=call.assigned;
    out.matrices=structured_matrices(b,cars,calls,nc,kernel);
    const auto& M=out.matrices;
    const int C=static_cast<int>(cars.size()),n=static_cast<int>(M.N.size());
    if(!n)return out;
    out.greedy=greedy_public(M.u,M.w,C,opt);out.final=out.greedy;
    out.greedy_cost=g_of_public(out.greedy,M.u,M.w,C,opt);
    out.iterations=local_search_public(out.final,M.u,M.w,C,opt);
    out.final_cost=g_of_public(out.final,M.u,M.w,C,opt);
    for(int i=0;i<n;++i) {
        const auto& car=cars[out.final[i]];
        const bool removed=car.load>=std::max(1,b.capacity-opt.capacity_margin);
        out.after[M.N[i].key]=removed?-1:car.id;out.removed+=removed;
    }
    return out;
}
}
