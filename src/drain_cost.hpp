#pragma once
#include <deque>
#include <numeric>
#include "path.hpp"
namespace egcs2017 {
struct DrainCost { double waiting=0,journey=0,end=0; int boarded=0,alighted=0,capacity_blocked_stops=0,partial_stops=0; };
inline DrainCost drain_cost(const Building& b,const CarView& car,
        std::map<HallKey,std::deque<int>> pending,bool full_bypass=false) {
    if(car.doors_open) throw std::invalid_argument("open-door drain not yet supported");
    if(b.capacity<1) throw std::invalid_argument("invalid capacity");
    auto on=car.car_calls;
    int load=0,N=0; for(auto [f,n]:on) {
        if(f<0||f>=b.floors||n<1) throw std::invalid_argument("invalid onboard counts");
        load+=n;
    }
    if(load!=car.load||load>b.capacity) throw std::invalid_argument("inconsistent load");
    N=load;
    for(auto& [k,q]:pending) {
        if(k.floor<0||k.floor>=b.floors||(k.dir!=UP&&k.dir!=DOWN)||q.empty())
            throw std::invalid_argument("invalid hall queue");
        for(int d:q) if(d<0||d>=b.floors||(d-k.floor)*k.dir<=0) throw std::invalid_argument("invalid destination");
        N+=q.size();
    }
    DrainCost out; double t=0;int pos=car.committed,dir=car.dir;
    auto keys=[&](bool at_rest) {
        std::vector<HallKey> result;
        for(auto& [k,q]:pending) if(!(load>=b.capacity&&(full_bypass||(at_rest&&k.floor==pos)))) result.push_back(k);
        return result;
    };
    auto ahead=[&](int d) {
        for(auto [f,n]:on) if((f-pos)*d>0) return true;
        for(auto& [k,q]:pending) if((k.floor-pos)*d>0) return true;
        return false;
    };
    auto board=[&](int d) {
        auto it=pending.find({pos,d});int count=0;
        if(it==pending.end()) return count;
        auto& q=it->second;
        while(load<b.capacity&&!q.empty()) {
            ++on[q.front()];q.pop_front();++load;++count;
            out.waiting+=t;++out.boarded;
        }
        if(!q.empty()) {++out.capacity_blocked_stops;if(count>0)++out.partial_stops;}
        if(q.empty()) pending.erase(it);
        return count;
    };
    auto stop=[&]() {
        int off=0;auto it=on.find(pos);
        if(it!=on.end()) {off=it->second;on.erase(it);load-=off;out.journey+=off*t;out.alighted+=off;}
        int got=board(dir);
        if(!ahead(dir)&&pending.count({pos,-dir})) {dir=-dir;got+=board(dir);}
        t+=b.doors.dwell(got,off);
    };
    if(car.moving) {
        auto m=next_move(pos,dir,on,keys(false));
        int target=(m.kind==Move::LEG||m.kind==Move::TAIL_OPP)?m.target:pos;
        t=b.kin.time_from_state(std::abs(b.pos(target)-car.x),car.v);
        pos=target;if(m.kind==Move::TAIL_OPP)dir=-dir;
        stop();
    } else {
        t=car.release;
        if(car.arriving)stop();
    }
    int guard=16*(N+1)*(b.floors+1);
    while(!on.empty()||!pending.empty()) {
        if(--guard<0)throw std::runtime_error("drain failed to terminate");
        auto ks=keys(true);
        if(!dir)dir=initial_direction(pos,on,ks);
        auto m=next_move(pos,dir,on,ks);
        if(m.kind==Move::TAIL_BEHIND){dir=m.dir;continue;}
        if(m.kind==Move::HALT)throw std::runtime_error("drain halted with passengers");
        t+=b.kin.travel(pos,m.target);pos=m.target;
        if(m.kind==Move::TAIL_OPP)dir=-dir;
        stop();
    }
    if(out.alighted!=N)throw std::runtime_error("drain conservation failure");
    out.end=t;return out;
}
}
