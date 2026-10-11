#include "../research/forward_pair_features.hpp"
#include <chrono>
#include <iomanip>
#include <iostream>
#include <stdexcept>
using namespace egcs2017;
using namespace egcs2017::research;

int main() {
    try {
        long states=0, car_states=0, ff=0, fl=0, fast_ff=0, fast_fl=0, corrected_rows=0;
        long rows_with_fast=0, rows_entirely_fast=0, states_with_fast=0;
        double error=0, saved_error=0, sum_error=0;
        CostOptions raw; int F,C,n;
        while(std::cin>>F>>C>>n) {
            Building b; b.floors=F;
            std::vector<CarView> cars(C);
            for(auto& c:cars) {
                int count;
                std::cin>>c.id>>c.x>>c.v>>c.dir>>c.moving>>c.arriving>>c.committed
                        >>c.release>>c.doors_open>>c.load>>count;
                for(int j=0;j<count;++j) {int f,q;std::cin>>f>>q;c.car_calls[f]=q;}
            }
            std::vector<HallKey> free(n);
            for(auto& k:free) std::cin>>k.floor>>k.dir;
            std::vector<std::vector<HallKey>> locked(C);
            for(auto& ls:locked) {
                int count;std::cin>>count;ls.resize(count);
                for(auto& k:ls) std::cin>>k.floor>>k.dir;
            }
            std::vector<std::vector<double>> saved_u(n,std::vector<double>(C));
            for(auto& row:saved_u) for(auto& x:row) std::cin>>x;
            std::vector<std::vector<double>> saved_r(n,std::vector<double>(C));
            for(auto& row:saved_r) for(auto& x:row) std::cin>>x;
            std::vector<std::vector<std::vector<double>>> saved_w(n,std::vector<std::vector<double>>(n,std::vector<double>(C)));
            for(auto& row:saved_w) for(auto& col:row) for(auto& x:col) std::cin>>x;
            if(!std::cin) throw std::runtime_error("truncated snapshot input");
            bool any_fast=false;
            for(int c=0;c<C;++c) {
                ++car_states;
                auto up=forward_features(b,cars[c],UP), down=forward_features(b,cars[c],DOWN);
                auto match=[&](HallKey a,HallKey z)->const ForwardPairFeatures* {
                    const auto& f=a.dir==UP?up:down;
                    return f && f->contains(a) && f->contains(z) && !(a==z)?&*f:nullptr;
                };
                for(int i=0;i<n;++i) {
                    saved_error=std::max(saved_error,std::abs(unary(b,cars[c],free[i],raw)-saved_u[i][c]));
                    for(int j=i+1;j<n;++j) {
                        ++ff;
                        const double ref=pairwise(b,cars[c],free[i],free[j],saved_u[i][c],saved_u[j][c]);
                        saved_error=std::max(saved_error,std::abs(ref-saved_w[i][j][c]));
                        if(auto f=match(free[i],free[j])) {
                            ++fast_ff;any_fast=true;
                            error=std::max(error,std::abs(f->pair(free[i],free[j])-ref));
                        }
                    }
                }

                std::vector<double> aggregate(n,0), refsum(n,0);
                for(const auto* f:{up?&*up:nullptr,down?&*down:nullptr}) if(f) {
                    std::vector<HallKey> ns,ls;std::vector<int> ids;
                    for(int i=0;i<n;++i) if(f->contains(free[i])) {ns.push_back(free[i]);ids.push_back(i);}
                    for(auto k:locked[c]) if(f->contains(k)) ls.push_back(k);
                    auto r=f->correction(ns,ls);
                    for(size_t i=0;i<ids.size();++i) aggregate[ids[i]]+=r[i];
                }
                for(int i=0;i<n;++i) {
                    int covered=0;
                    for(auto k:locked[c]) {
                        ++fl;
                        const double v=pairwise(b,cars[c],free[i],k,saved_u[i][c],unary(b,cars[c],k,raw));
                        refsum[i]+=std::max(0.0,v);
                        if(auto f=match(free[i],k)) {
                            ++fast_fl;++covered;any_fast=true;
                            error=std::max(error,std::abs(f->pair(free[i],k)-v));
                        } else aggregate[i]+=std::max(0.0,v);
                    }
                    if(!locked[c].empty()) {
                        ++corrected_rows;rows_with_fast+=covered>0;
                        rows_entirely_fast+=covered==int(locked[c].size());
                    }
                    sum_error=std::max(sum_error,std::abs(aggregate[i]-refsum[i]));
                    saved_error=std::max(saved_error,std::abs(refsum[i]-saved_r[i][c]));
                }
            }
            ++states;states_with_fast+=any_fast;
        }
        if(error>1e-7 || sum_error>1e-7 || saved_error>1e-6)
            throw std::runtime_error("saved-state coefficient mismatch: "+std::to_string(error)+","+std::to_string(sum_error)+","+std::to_string(saved_error));
        std::cout<<std::setprecision(17)<<"{\"states\":"<<states<<",\"car_states\":"<<car_states
          <<",\"free_free_pairs\":"<<ff<<",\"free_locked_pairs\":"<<fl
          <<",\"fast_free_free_pairs\":"<<fast_ff<<",\"fast_free_locked_pairs\":"<<fast_fl
          <<",\"nonempty_correction_rows\":"<<corrected_rows<<",\"rows_with_fast_part\":"<<rows_with_fast
          <<",\"rows_entirely_fast\":"<<rows_entirely_fast<<",\"states_with_fast_part\":"<<states_with_fast
          <<",\"maximum_pair_error\":"<<error<<",\"maximum_row_error\":"<<sum_error
          <<",\"maximum_saved_coefficient_error\":"<<saved_error<<"}\n";
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 2;}
}
