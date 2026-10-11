#include "../research/forward_dispatch.hpp"
#include <chrono>
#include <iomanip>
#include <iostream>

using namespace egcs2017;
using namespace egcs2017::research;

struct Input {
    Building b;
    std::vector<CarView> cars;
    std::vector<CallView> calls;
    HallKey nc;
    std::vector<std::vector<double>> u,r;
    std::vector<std::vector<std::vector<double>>> w;
};

bool read(Input& s) {
    int C,n;
    if(!(std::cin>>s.b.floors>>C>>n)) return false;
    s.cars.resize(C);
    for(auto& c:s.cars) {
        int count;
        std::cin>>c.id>>c.x>>c.v>>c.dir>>c.moving>>c.arriving>>c.committed
                >>c.release>>c.doors_open>>c.load>>count;
        for(int j=0;j<count;++j) {int f,q;std::cin>>f>>q;c.car_calls[f]=q;}
    }
    s.calls.resize(n);
    for(auto& c:s.calls) std::cin>>c.key.floor>>c.key.dir;
    for(int i=0;i<C;++i) {
        int count;std::cin>>count;
        for(int j=0;j<count;++j) {
            CallView c;std::cin>>c.key.floor>>c.key.dir;
            c.close_to_service=true;c.assigned=s.cars[i].id;c.registered=-1;
            s.calls.push_back(c);
        }
    }
    s.u.assign(n,std::vector<double>(C));s.r=s.u;
    for(auto& row:s.u) for(auto& x:row) std::cin>>x;
    for(auto& row:s.r) for(auto& x:row) std::cin>>x;
    s.w.assign(n,std::vector<std::vector<double>>(n,std::vector<double>(C)));
    for(auto& row:s.w) for(auto& col:row) for(auto& x:col) std::cin>>x;
    std::cin>>s.nc.floor>>s.nc.dir;
    for(int i=0;i<n;++i) std::cin>>s.calls[i].registered>>s.calls[i].assigned>>s.calls[i].waiting;
    if(!std::cin) throw std::runtime_error("truncated saved dispatch input");
    return true;
}

double matrix_error(const Matrices& x,const Matrices& y) {
    double e=0;
    if(x.N.size()!=y.N.size()) throw std::runtime_error("call count changed");
    auto check=[&](double a,double b) {
        if(!std::isfinite(a)||!std::isfinite(b)) throw std::runtime_error("nonfinite cost");
        e=std::max(e,std::abs(a-b));
    };
    for(size_t i=0;i<x.N.size();++i) {
        if(!(x.N[i].key==y.N[i].key)) throw std::runtime_error("call order changed");
        for(size_t c=0;c<x.u[i].size();++c) {
            check(x.u[i][c],y.u[i][c]);check(x.u_raw[i][c],y.u_raw[i][c]);
            check(x.u_lk[i][c],y.u_lk[i][c]);check(x.adj[i][c],y.adj[i][c]);
            for(size_t j=0;j<x.N.size();++j) check(x.w[i][j][c],y.w[i][j][c]);
        }
    }
    return e;
}

int main() {
    try {
        std::vector<Input> inputs;
        while(true) {Input s;if(!read(s)) break;inputs.push_back(std::move(s));}
        if(inputs.empty()) throw std::runtime_error("no input states");
        const auto opt=locked_ls_options(); SubmodularDispatcher original(opt,"paper2017_ls_lk");
        long locked=0,removed=0,empty=0,capacity_calls=0;
        long cached_greedy=0,cached_final=0,cached_after=0,fast_greedy=0,fast_final=0,fast_after=0;
        double raw_error=0,cached_error=0,fast_error=0,max_final_cost_difference=0;
        for(const auto& s:inputs) {
            const auto M=build_matrices(s.b,s.cars,s.calls,s.nc,opt);
            for(size_t i=0;i<M.N.size();++i) for(size_t c=0;c<s.cars.size();++c) {
                raw_error=std::max(raw_error,std::abs(M.u_raw[i][c]-s.u[i][c]));
                raw_error=std::max(raw_error,std::abs(M.u_lk[i][c]-s.r[i][c]));
                for(size_t j=0;j<M.N.size();++j) raw_error=std::max(raw_error,std::abs(M.w[i][j][c]-s.w[i][j][c]));
            }
            DecisionTrace trace;
            const auto after=original.decide(s.b,s.cars,s.calls,s.nc,&trace);
            auto g=greedy_public(M.u,M.w,static_cast<int>(s.cars.size()),opt),f=g;
            local_search_public(f,M.u,M.w,static_cast<int>(s.cars.size()),opt);
            for(const auto& [i,c,gain]:trace.order) {
                (void)gain;
                if(g[i]!=c) throw std::runtime_error("public greedy disagrees with actual trace");
            }
            const double ref_cost=g_of_public(f,M.u,M.w,static_cast<int>(s.cars.size()),opt);
            if(std::abs(ref_cost-trace.g_final)>1e-9) throw std::runtime_error("public LS disagrees with actual trace");
            const auto cached=forward_decide(s.b,s.cars,s.calls,s.nc,false);
            const auto fast=forward_decide(s.b,s.cars,s.calls,s.nc,true);
            cached_error=std::max(cached_error,matrix_error(M,cached.matrices));
            fast_error=std::max(fast_error,matrix_error(M,fast.matrices));
            cached_greedy+=cached.greedy!=g;cached_final+=cached.final!=f;cached_after+=cached.after!=after;
            fast_greedy+=fast.greedy!=g;fast_final+=fast.final!=f;fast_after+=fast.after!=after;
            max_final_cost_difference=std::max(max_final_cost_difference,std::abs(
                g_of_public(fast.final,M.u,M.w,static_cast<int>(s.cars.size()),opt)-ref_cost));
            empty+=M.N.empty();

            locked+=std::any_of(M.locked.begin(),M.locked.end(),
                               [](const auto& row){return !row.empty();});
            bool any_removed=false;
            for(const auto& c:M.N) {bool r=after.at(c.key)<0;any_removed|=r;capacity_calls+=r;}
            removed+=any_removed;
            for(const auto& c:s.calls) if(c.close_to_service && !(c.key==s.nc)) {
                if(cached.after.at(c.key)!=c.assigned || fast.after.at(c.key)!=c.assigned)
                    throw std::runtime_error("locked call moved");
            }
        }
        if(raw_error>1e-7 || fast_error>1e-7 || cached_error>1e-7)
            throw std::runtime_error("input/matrix mismatch");

        constexpr int rounds=8;
        std::vector<double> base(rounds,0),fast(rounds,0);
        auto timed=[&](const Input& s,bool hybrid) {
            auto start=std::chrono::steady_clock::now();
            auto result=forward_decide(s.b,s.cars,s.calls,s.nc,hybrid);
            double us=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count();

            if(result.after.size()!=s.calls.size()) throw std::runtime_error("lost call in timed decision");
            return us;
        };
        for(int r=0;r<rounds;++r) for(size_t i=0;i<inputs.size();++i) {
            if((r+i)%2==0) {base[r]+=timed(inputs[i],false);fast[r]+=timed(inputs[i],true);}
            else {fast[r]+=timed(inputs[i],true);base[r]+=timed(inputs[i],false);}
        }
        double mean_base=0,mean_fast=0;
        for(int r=0;r<rounds;++r) {mean_base+=base[r]/rounds/inputs.size();mean_fast+=fast[r]/rounds/inputs.size();}
        std::cout<<std::setprecision(17)<<"{\"states\":"<<inputs.size()<<",\"locked_states\":"<<locked
          <<",\"empty_free_states\":"<<empty<<",\"states_with_capacity_removal\":"<<removed
          <<",\"capacity_removed_labels\":"<<capacity_calls
          <<",\"cached_greedy_changed\":"<<cached_greedy<<",\"cached_final_changed\":"<<cached_final
          <<",\"cached_return_changed\":"<<cached_after
          <<",\"hybrid_greedy_changed\":"<<fast_greedy<<",\"hybrid_final_changed\":"<<fast_final
          <<",\"hybrid_return_changed\":"<<fast_after
          <<",\"maximum_saved_input_error\":"<<raw_error<<",\"maximum_cached_matrix_error\":"<<cached_error
          <<",\"maximum_hybrid_matrix_error\":"<<fast_error
          <<",\"maximum_final_cost_difference_on_original\":"<<max_final_cost_difference
          <<",\"rounds\":"<<rounds<<",\"cached_full_mean_us\":"<<mean_base
          <<",\"hybrid_full_mean_us\":"<<mean_fast<<",\"hybrid_over_cached\":"<<mean_fast/mean_base
          <<",\"round_totals_us\":[";
        for(int r=0;r<rounds;++r) {
            if(r) std::cout<<',';
            std::cout<<"{\"cached\":"<<base[r]<<",\"hybrid\":"<<fast[r]<<'}';
        }
        std::cout<<"]}\n";
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 2;}
}
