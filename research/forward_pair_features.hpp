#pragma once
#include "cost.hpp"
#include <algorithm>
#include <optional>
#include <stdexcept>

namespace egcs2017::research {

struct ForwardPairFeatures {
    int floors = 0, direction = 0, start = 0;

    std::vector<double> a, u, v;

    int orient(int f) const { return direction == UP ? f : floors-1-f; }
    bool contains(HallKey k) const {
        const int f = orient(k.floor);
        return k.dir == direction && f >= start && f < floors-1;
    }
    double pair(HallKey x, HallKey y) const {
        if (!contains(x) || !contains(y) || x == y)
            throw std::invalid_argument("outside distinct forward-call domain");
        int i=orient(x.floor), j=orient(y.floor);
        if(i>j) std::swap(i,j);
        return a[i]+u[i]*v[j];
    }

    std::vector<double> correction(const std::vector<HallKey>& free,
                                   const std::vector<HallKey>& locked) const {
        std::vector<bool> is_locked(floors, false), is_free(floors, false);
        double upper_v=0; int upper_n=0;
        for(auto k:locked) {
            if(!contains(k) || is_locked[orient(k.floor)])
                throw std::invalid_argument("invalid or duplicate locked call");
            const int f=orient(k.floor); is_locked[f]=true;
            upper_v+=v[f]; ++upper_n;
        }
        for(auto k:free) {
            if(!contains(k) || is_locked[orient(k.floor)] || is_free[orient(k.floor)])
                throw std::invalid_argument("invalid, repeated or overlapping free call");
            is_free[orient(k.floor)]=true;
        }
        double lower_a=0, lower_u=0;
        std::vector<double> rows(floors,0), out;
        for(int f=start; f<floors-1; ++f) {
            if(is_locked[f]) { upper_v-=v[f]; --upper_n; }
            rows[f]=lower_a+lower_u*v[f]+upper_n*a[f]+u[f]*upper_v;
            if(is_locked[f]) { lower_a+=a[f]; lower_u+=u[f]; }
        }
        for(auto k:free) out.push_back(rows[orient(k.floor)]);
        return out;
    }
};

inline std::optional<ForwardPairFeatures> forward_features(
        const Building& building, const CarView& car, int direction) {
    if(direction != UP && direction != DOWN) return std::nullopt;
    const auto& kin=building.kin;

    if(kin.floor_height < kin.v_max*kin.v_max/kin.a_max) return std::nullopt;
    if(car.dir != direction && car.dir != 0) return std::nullopt;
    if(car.moving && car.dir != direction) return std::nullopt;

    if(car.moving) {
        const double d=(building.pos(car.committed)-car.x)*direction;
        if(car.v<0 || car.v>kin.v_max || d<0 || d+1e-6<kin.stop_distance(car.v))
            return std::nullopt;
    }
    ForwardPairFeatures out;
    out.floors=building.floors; out.direction=direction;
    out.start=out.orient(car.committed);
    const int F=out.floors, s=out.start;
    if(s<0 || s>=F) return std::nullopt;
    std::vector<int> mandatory(F,0), prefix(F,0);
    for(auto [f,n]:car.car_calls) {
        const int z=out.orient(f);
        if(z<0 || z>=F || n<=0) return std::nullopt;

        if(car.dir==0 && z<s) return std::nullopt;
        mandatory[z]=1;
    }
    for(int f=0;f<F;++f) prefix[f]=mandatory[f]+(f?prefix[f-1]:0);
    const double gamma=kin.floor_height/kin.v_max;
    const double kappa=kin.v_max/kin.a_max+kin.settle;
    const double tau=building.doors.transfer;
    const double overhead=kappa+building.doors.open+building.doors.close;
    const double q=overhead+tau;
    auto original_floor=[&](int f){return direction==UP?f:F-1-f;};
    auto first_leg=[&](int f) {
        const int floor=original_floor(f);
        return car.moving ? kin.time_from_state(std::abs(building.pos(floor)-car.x),car.v)
                          : kin.travel(car.committed,floor);
    };
    out.a.assign(F,0); out.u.assign(F,0); out.v.assign(F,0);
    for(int f=s;f<F-1;++f) {

        double beta=mandatory[f]?tau:q;
        if(f==s && !mandatory[f]) beta=q+first_leg(s)-first_leg(s+1)+gamma;

        if(f==s && !car.moving && !car.arriving && car.doors_open && car.dir==direction)
            beta=tau;
        const double denominator=F-1-f;
        out.u[f]=1.0/denominator;
        out.v[f]=q*f-overhead*(f?prefix[f-1]:0);
        out.a[f]=beta-(q*(f+1)-overhead*prefix[f])/denominator;
    }
    return out;
}
}
