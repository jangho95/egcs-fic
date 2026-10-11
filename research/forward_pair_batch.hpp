#pragma once
#include "forward_pair_features.hpp"

namespace egcs2017::research {

struct PairBatch {
    std::vector<double> raw, correction;
    std::vector<std::vector<double>> pairs;
};

inline double nonnegative_pair(double x) {
    if(!std::isfinite(x) || x < -1e-6)
        throw std::runtime_error("invalid pair coefficient");
    return std::max(0.0,x);
}

inline PairBatch pair_batch(const Building& b, const CarView& car,
        const std::vector<HallKey>& free, const std::vector<HallKey>& locked,
        bool hybrid) {
    const int n=static_cast<int>(free.size());
    PairBatch out;
    out.raw.resize(n); out.correction.assign(n,0);
    out.pairs.assign(n,std::vector<double>(n,0));
    if(n==0) return out;
    CostOptions raw;
    for(int i=0;i<n;++i) out.raw[i]=unary(b,car,free[i],raw);
    std::vector<double> locked_raw;
    for(auto k:locked) locked_raw.push_back(unary(b,car,k,raw));
    std::optional<ForwardPairFeatures> up,down;
    if(hybrid) {
        for(int direction:{UP,DOWN}) {

            int nf=0,nl=0;
            auto ahead=[&](HallKey k) {
                return k.dir==direction && (k.floor-car.committed)*direction>=0;
            };
            for(auto k:free) nf+=ahead(k);
            for(auto k:locked) nl+=ahead(k);
            if(nf>0 && nf+nl>=2) {
                auto f=forward_features(b,car,direction);
                (direction==UP?up:down)=std::move(f);
            }
        }
    }
    auto match=[&](HallKey x,HallKey y)->const ForwardPairFeatures* {
        const auto& f=x.dir==UP?up:down;
        return f && f->contains(x) && f->contains(y)?&*f:nullptr;
    };
    for(int i=0;i<n;++i) for(int j=i+1;j<n;++j) {
        const auto* f=match(free[i],free[j]);
        const double v=f ? f->pair(free[i],free[j])
            :pairwise(b,car,free[i],free[j],out.raw[i],out.raw[j]);
        out.pairs[i][j]=out.pairs[j][i]=nonnegative_pair(v);
    }
    for(const auto* f:{up?&*up:nullptr,down?&*down:nullptr}) if(f) {
        std::vector<HallKey> ns,ls; std::vector<int> ids;
        for(int i=0;i<n;++i) if(f->contains(free[i])) {ns.push_back(free[i]);ids.push_back(i);}
        for(auto k:locked) if(f->contains(k)) ls.push_back(k);
        if(ns.empty() || ls.empty()) continue;
        const auto rows=f->correction(ns,ls);
        for(size_t i=0;i<ids.size();++i) out.correction[ids[i]]+=nonnegative_pair(rows[i]);
    }
    for(int i=0;i<n;++i) for(size_t j=0;j<locked.size();++j)
        if(!match(free[i],locked[j])) out.correction[i]+=nonnegative_pair(
            pairwise(b,car,free[i],locked[j],out.raw[i],locked_raw[j]));
    return out;
}
}
