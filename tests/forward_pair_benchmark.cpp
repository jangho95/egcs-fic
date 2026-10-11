#include "../research/forward_pair_batch.hpp"
#include <chrono>
#include <iomanip>
#include <iostream>
#include <numeric>

using namespace egcs2017;
using namespace egcs2017::research;

struct Snapshot {
    Building building;
    std::vector<CarView> cars;
    std::vector<HallKey> free;
    std::vector<std::vector<HallKey>> locked;
    std::vector<std::vector<double>> saved_u,saved_r;
    std::vector<std::vector<std::vector<double>>> saved_w;
};

bool read(Snapshot& s) {
    int F,C,n;
    if(!(std::cin>>F>>C>>n)) return false;
    s.building.floors=F; s.cars.resize(C);s.free.resize(n);s.locked.resize(C);
    for(auto& c:s.cars) {
        int count;
        std::cin>>c.id>>c.x>>c.v>>c.dir>>c.moving>>c.arriving>>c.committed
                >>c.release>>c.doors_open>>c.load>>count;
        for(int j=0;j<count;++j) {int f,q;std::cin>>f>>q;c.car_calls[f]=q;}
    }
    for(auto& k:s.free) std::cin>>k.floor>>k.dir;
    for(auto& ls:s.locked) {
        int count;std::cin>>count;ls.resize(count);
        for(auto& k:ls) std::cin>>k.floor>>k.dir;
    }
    s.saved_u.assign(n,std::vector<double>(C));s.saved_r=s.saved_u;
    for(auto& row:s.saved_u) for(auto& x:row) std::cin>>x;
    for(auto& row:s.saved_r) for(auto& x:row) std::cin>>x;
    s.saved_w.assign(n,std::vector<std::vector<double>>(n,std::vector<double>(C)));
    for(auto& row:s.saved_w) for(auto& col:row) for(auto& x:col) std::cin>>x;
    if(!std::cin) throw std::runtime_error("truncated snapshot");
    return true;
}

using Output=std::vector<PairBatch>;
Output calculate(const Snapshot& s,bool hybrid) {
    Output out;
    for(size_t c=0;c<s.cars.size();++c)
        out.push_back(pair_batch(s.building,s.cars[c],s.free,s.locked[c],hybrid));
    return out;
}

double verify(const Snapshot& s,const Output& out) {
    double error=0;
    auto check=[&](double a,double b) {
        if(!std::isfinite(a)) throw std::runtime_error("nonfinite coefficient");
        error=std::max(error,std::abs(a-b));
    };
    for(size_t c=0;c<s.cars.size();++c) for(size_t i=0;i<s.free.size();++i) {
        check(out[c].raw[i],s.saved_u[i][c]);
        check(out[c].correction[i],s.saved_r[i][c]);
        for(size_t j=0;j<s.free.size();++j) check(out[c].pairs[i][j],s.saved_w[i][j][c]);
    }
    if(error>1e-7) throw std::runtime_error("coefficient mismatch");
    return error;
}

double quantile(std::vector<double> x,double q) {
    std::sort(x.begin(),x.end());
    const double p=q*(x.size()-1);
    const size_t i=static_cast<size_t>(p),j=std::min(i+1,x.size()-1);
    return x[i]+(p-i)*(x[j]-x[i]);
}

int main() {
    try {
        std::vector<Snapshot> inputs;
        while(true) {Snapshot s;if(!read(s)) break;inputs.push_back(std::move(s));}
        if(inputs.empty()) throw std::runtime_error("no snapshots");
        double max_reference=0,max_hybrid=0;

        for(const auto& s:inputs) {
            max_reference=std::max(max_reference,verify(s,calculate(s,false)));
            max_hybrid=std::max(max_hybrid,verify(s,calculate(s,true)));
        }
        constexpr int rounds=8;
        std::vector<double> base_times(inputs.size(),0),fast_times(inputs.size(),0);
        std::vector<double> base_totals(rounds,0),fast_totals(rounds,0);
        auto timed=[&](const Snapshot& s,bool hybrid) {
            auto start=std::chrono::steady_clock::now();
            auto out=calculate(s,hybrid);
            const double us=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count();
            verify(s,out);
            return us;
        };
        for(int r=0;r<rounds;++r) for(size_t i=0;i<inputs.size();++i) {
            double a,b;
            if((r+i)%2==0) {a=timed(inputs[i],false);b=timed(inputs[i],true);}
            else {b=timed(inputs[i],true);a=timed(inputs[i],false);}
            base_times[i]+=a/rounds; fast_times[i]+=b/rounds;
            base_totals[r]+=a;fast_totals[r]+=b;
        }
        const double base=std::accumulate(base_times.begin(),base_times.end(),0.0)/inputs.size();
        const double fast=std::accumulate(fast_times.begin(),fast_times.end(),0.0)/inputs.size();
        std::cout<<std::setprecision(17)<<"{\"states\":"<<inputs.size()<<",\"rounds\":"<<rounds
            <<",\"reference_mean_us\":"<<base<<",\"hybrid_mean_us\":"<<fast
            <<",\"hybrid_over_reference\":"<<fast/base
            <<",\"reference_state_p95_us\":"<<quantile(base_times,.95)
            <<",\"hybrid_state_p95_us\":"<<quantile(fast_times,.95)
            <<",\"maximum_reference_error\":"<<max_reference
            <<",\"maximum_hybrid_error\":"<<max_hybrid<<",\"rounds_total_us\":[";
        for(int r=0;r<rounds;++r) {
            if(r) std::cout<<',';
            std::cout<<"{\"reference\":"<<base_totals[r]<<",\"hybrid\":"<<fast_totals[r]<<'}';
        }
        std::cout<<"]}\n";
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 2;}
}
