#pragma once
#include "forward_pair_features.hpp"

namespace egcs2017::research {

struct ProfilePairFeatures {
    ForwardPairFeatures geometry;
    int profile_count=0;
    std::vector<double> a;
    std::vector<std::vector<double>> u,v;
    std::vector<bool> supported;
    bool contains(HallKey k) const {
        return geometry.contains(k) && supported[geometry.orient(k.floor)];
    }
    double pair(HallKey x,HallKey y) const {
        if(!contains(x)||!contains(y)||x==y) throw std::invalid_argument("outside profile pair domain");
        int i=geometry.orient(x.floor),j=geometry.orient(y.floor);if(i>j)std::swap(i,j);
        double out=a[i];for(int r=0;r<profile_count;++r)out+=u[i][r]*v[j][r];return out;
    }
    std::vector<double> correction(const std::vector<HallKey>& free,
                                   const std::vector<HallKey>& locked) const {
        const int F=geometry.floors;
        std::vector<bool> is_locked(F,false),is_free(F,false);
        std::vector<double> lo_u(profile_count,0),hi_v(profile_count,0),rows(F,0),out;
        double lo_a=0;int hi_n=0;
        for(auto k:locked) {
            if(!contains(k)||is_locked[geometry.orient(k.floor)]) throw std::invalid_argument("invalid profile lock");
            const int f=geometry.orient(k.floor);is_locked[f]=true;++hi_n;
            for(int r=0;r<profile_count;++r)hi_v[r]+=v[f][r];
        }
        for(auto k:free) {
            if(!contains(k)||is_free[geometry.orient(k.floor)]||is_locked[geometry.orient(k.floor)])
                throw std::invalid_argument("invalid profile free call");
            is_free[geometry.orient(k.floor)]=true;
        }
        for(int f=geometry.start;f<F-1;++f) {
            if(is_locked[f]) {--hi_n;for(int r=0;r<profile_count;++r)hi_v[r]-=v[f][r];}
            rows[f]=lo_a+hi_n*a[f];
            for(int r=0;r<profile_count;++r)rows[f]+=lo_u[r]*v[f][r]+u[f][r]*hi_v[r];
            if(is_locked[f]) {lo_a+=a[f];for(int r=0;r<profile_count;++r)lo_u[r]+=u[f][r];}
        }
        for(auto k:free)out.push_back(rows[geometry.orient(k.floor)]);
        return out;
    }
};

inline std::optional<ProfilePairFeatures> profile_features(
        const Building& b,const CarView& car,int direction,
        const std::vector<std::vector<double>>& atom,
        const std::vector<std::vector<double>>& mix) {
    auto base=forward_features(b,car,direction);if(!base)return std::nullopt;
    const int F=b.floors,R=static_cast<int>(atom.size());
    if(R==0 || static_cast<int>(mix.size())!=F) throw std::invalid_argument("profile dimensions");
    auto valid=[](const auto& row,int n) {
        if(static_cast<int>(row.size())!=n)throw std::invalid_argument("profile row dimensions");
        for(double x:row)if(!std::isfinite(x)||x<0)throw std::invalid_argument("negative/nonfinite profile");
    };
    for(const auto& row:atom)valid(row,F);
    for(const auto& row:mix)valid(row,R);
    ProfilePairFeatures out;out.geometry=*base;out.profile_count=R;
    out.a.assign(F,0);out.u.assign(F,std::vector<double>(R));out.v=out.u;out.supported.assign(F,false);
    std::vector<int> mandatory(F,0),prefix(F,0);
    for(const auto& [f,n]:car.car_calls) { (void)n;mandatory[base->orient(f)]=1; }
    for(int f=0;f<F;++f)prefix[f]=mandatory[f]+(f?prefix[f-1]:0);
    const double h=b.kin.v_max/b.kin.a_max+b.kin.settle+b.doors.open+b.doors.close;
    const double Q=h+b.doors.transfer;
    std::vector<std::vector<double>> cost(F,std::vector<double>(R)),tail=cost;
    for(int r=0;r<R;++r) {
        double sum=0;
        for(int f=0;f<F;++f) {sum+=atom[r][f]*(Q-h*mandatory[f]);cost[f][r]=sum;}
        sum=0;
        for(int f=F-1;f>=0;--f) {tail[f][r]=sum;sum+=atom[r][f];}
    }
    for(int f=base->start;f<F-1;++f) {
        double z=0;for(int r=0;r<R;++r)z+=mix[f][r]*tail[f][r];
        if(!std::isfinite(z))throw std::invalid_argument("profile normalizer overflow");
        if(z==0)continue;
        out.supported[f]=true;
        out.a[f]=base->a[f]+(Q*(f+1)-h*prefix[f])/(F-1-f);
        for(int r=0;r<R;++r) {
            out.u[f][r]=mix[f][r]/z;
            out.a[f]-=out.u[f][r]*cost[f][r];
        }
    }

    for(int f=0;f<F;++f)for(int r=0;r<R;++r)out.v[f][r]=f?cost[f-1][r]:0;
    return out;
}
}
