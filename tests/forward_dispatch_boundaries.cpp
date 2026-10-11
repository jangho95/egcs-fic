#include "../research/forward_dispatch.hpp"
#include <iomanip>
#include <iostream>
#include <limits>

using namespace egcs2017;
using namespace egcs2017::research;

int main() {
    try {
        const auto opt=locked_ls_options(); SubmodularDispatcher original(opt,"paper2017_ls_lk");
        int checks=0;
        auto equal=[&](const Building& b,const std::vector<CarView>& cars,
                       const std::vector<CallView>& calls,HallKey nc) {
            auto ref=original.decide(b,cars,calls,nc,nullptr);
            auto plain=forward_decide(b,cars,calls,nc,false);
            auto fast=forward_decide(b,cars,calls,nc,true);
            if(ref!=plain.after || ref!=fast.after) throw std::runtime_error("contract case changed");
            ++checks;return fast;
        };
        Building b;b.floors=8;
        std::vector<CarView> cars(2);cars[0].id=2;cars[1].id=9;
        for(auto& c:cars) {c.dir=UP;c.load=11;}
        CallView lock{{1,UP},0,2,true},free{{3,UP},1,-1,false},fresh{{5,UP},2,-1,true};
        auto full=equal(b,cars,{lock,free,fresh},fresh.key);
        if(full.removed!=2 || full.after.at(lock.key)!=2 || full.after.at(fresh.key)!=-1)
            throw std::runtime_error("capacity/new-call lock exception not exercised");
        auto all_locked=equal(b,cars,{lock},no_new_call());
        if(!all_locked.matrices.N.empty() || all_locked.after.at(lock.key)!=2)
            throw std::runtime_error("all-locked early return changed");
        for(auto& c:cars) c.load=0;
        fresh.close_to_service=false;fresh.forbidden_car=2;
        auto forbidden=equal(b,cars,{lock,fresh},fresh.key);
        if(forbidden.after.at(fresh.key)!=9) throw std::runtime_error("forbidden label ignored");
        CallView opposite{{3,DOWN},3,-1,false};
        equal(b,cars,{lock,free,opposite,fresh},fresh.key);
        b.kin.floor_height=1;
        equal(b,cars,{lock,free,fresh},fresh.key);

        b=Building{};b.floors=8;cars=std::vector<CarView>(2);
        cars[0].id=0;cars[1].id=1;cars[0].dir=cars[1].dir=UP;
        lock=CallView{{2,UP},0,0,true};fresh=CallView{{5,UP},1,-1,false};
        CostOptions raw;
        const double wi=unary(b,cars[0],lock.key,raw),wj=unary(b,cars[0],fresh.key,raw);
        const double legacy_pair=pairwise(b,cars[0],lock.key,fresh.key,wi,wj);
        auto feature=forward_features(b,cars[0],UP);
        const double feature_pair=feature->pair(lock.key,fresh.key);
        const double center=legacy_pair-1e-12;
        int boundary=0,cached_changes=0,hybrid_changes=0;
        double max_cost_loss=0;
        struct Witness {double release,u0,u1,fu0;int ref,fast;};
        std::vector<Witness> witnesses;
        for(int offset=-64;offset<=64;++offset) {
            double t=center;
            for(int k=0;k<std::abs(offset);++k)
                t=std::nextafter(t,offset<0?-std::numeric_limits<double>::infinity():std::numeric_limits<double>::infinity());
            cars[1].release=t;
            auto ref=original.decide(b,cars,{lock,fresh},fresh.key,nullptr);
            auto cached=forward_decide(b,cars,{lock,fresh},fresh.key,false);
            auto fast=forward_decide(b,cars,{lock,fresh},fresh.key,true);
            ++boundary;cached_changes+=cached.after!=ref;
            if(fast.after!=ref) {
                ++hybrid_changes;
                const auto M=build_matrices(b,cars,{lock,fresh},fresh.key,opt);
                double loss=M.u[0][fast.after.at(fresh.key)]-M.u[0][ref.at(fresh.key)];
                max_cost_loss=std::max(max_cost_loss,std::abs(loss));
                witnesses.push_back({t,M.u[0][0],M.u[0][1],fast.matrices.u[0][0],ref.at(fresh.key),fast.after.at(fresh.key)});
            }
        }
        std::cout<<std::setprecision(17)<<"{\"contract_cases\":"<<checks<<",\"capacity_removed\":"<<full.removed
          <<",\"boundary_cases\":"<<boundary<<",\"boundary_cached_changed\":"<<cached_changes
          <<",\"boundary_hybrid_changed\":"<<hybrid_changes<<",\"legacy_pair\":"<<legacy_pair
          <<",\"feature_pair\":"<<feature_pair<<",\"maximum_original_cost_difference\":"<<max_cost_loss
          <<",\"witnesses\":[";
        for(size_t k=0;k<witnesses.size();++k) {
            const auto& w=witnesses[k];if(k)std::cout<<',';
            std::cout<<"{\"release_B\":"<<w.release<<",\"legacy_u_A\":"<<w.u0<<",\"legacy_u_B\":"<<w.u1
              <<",\"hybrid_u_A\":"<<w.fu0<<",\"legacy_car\":"<<w.ref<<",\"hybrid_car\":"<<w.fast<<'}';
        }
        std::cout<<"]}\n";
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 2;}
}
