#pragma once
#include "forward_pair_features.hpp"

namespace egcs2017::research {

struct CrossPairFeatures {
    int floors=0,direction=0,start=0,highest=0;
    double gamma=0,kappa=0,door=0,tau=0,h0=0,q=0,release=0;
    bool open_at_start=false;
    std::vector<int> known,above_count,above_load;
    std::vector<double> below_delay,beta;
    int orient(int f) const{return direction==UP?f:floors-1-f;}
    bool first(HallKey k) const {
        int i=orient(k.floor);return k.dir==direction && i>=start && i<floors-1;
    }
    bool second(HallKey k) const {
        int j=orient(k.floor);return k.dir==-direction && j>0 && j<floors;
    }
    bool contains(HallKey a,HallKey b) const {
        return (first(a)&&second(b))||(first(b)&&second(a));
    }
    double departure(int i,double raw) const {
        if(open_at_start && i==start)
            return release+tau+(known[i]>0?door+tau*known[i]:0.0);
        return raw+door+tau*(1+known[i]);
    }
    struct Row {bool low;double u,r,high_r,end;};
    struct Column {double low_v,high_v,end;};
    Row row(HallKey key,double raw) const {
        if(!first(key))throw std::invalid_argument("not a first forward call");
        int i=orient(key.floor);double u=1.0/(floors-1-i),b=departure(i,raw);

        return {i<highest,u,beta[i]+q-h0*above_count[i]*u,
            beta[i]+q,b+q+kappa};
    }
    Column column(HallKey key,double raw) const {
        if(!second(key))throw std::invalid_argument("not an opposite second call");
        int j=orient(key.floor),a=std::max(highest,j);
        double edge=h0+tau*known[highest];
        return {gamma*(floors-1-a)*(floors-a)
                +(j==highest?edge*(floors-1-highest):0.0)
                -(j>=highest?tau:0.0)-(j>highest?h0:0.0),
            gamma*(floors-1-j)*(floors-j)-q,
            gamma*(floors-j)+below_delay[j]-raw};
    }
    double pair(HallKey x,HallKey y,double ux,double uy) const {
        if(!contains(x,y))throw std::invalid_argument("outside cross-pair domain");
        if(!first(x)){std::swap(x,y);std::swap(ux,uy);}
        const int i=orient(x.floor),j=orient(y.floor);
        auto r=row(x,ux);auto c=column(y,uy);
        if(r.low)return r.r+r.u*c.low_v;
        return j<=i ? r.end+c.end : r.high_r+r.u*c.high_v;
    }

    double pair_by_turnaround(HallKey x,HallKey y,double ux,double uy) const {
        if(!contains(x,y))throw std::invalid_argument("outside cross-pair domain");
        if(!first(x)){std::swap(x,y);std::swap(ux,uy);}
        int i=orient(x.floor),j=orient(y.floor),H=std::max(i,highest),a=std::max(H,j);
        const double u=1.0/(floors-1-i);
        const double qj=j<H?0.0:(j==H?(H-i)*u:u);
        const double pj=j>=H && j>i ? u:0.0;
        const double excess=std::max(H-j,0)+0.5*(floors-1-a)*(floors-a)*u;
        return departure(i,ux)+gamma*(j-i)+h0*(above_count[i]+1-above_count[i]*u)
            +tau*(above_load[i]+1)+kappa+below_delay[j]+2*gamma*excess
            -(h0+tau*known[j])*qj-tau*pj-uy;
    }

    std::vector<double> correction(const std::vector<HallKey>& free,
            const std::vector<HallKey>& locked,const std::vector<double>& free_raw,
            const std::vector<double>& locked_raw) const {
        if(free.size()!=free_raw.size() || locked.size()!=locked_raw.size())
            throw std::invalid_argument("cross correction unary dimensions");
        std::vector<bool> keys(2*floors,false);
        auto check=[&](HallKey k) {
            if(k.floor<0 || k.floor>=floors || (k.dir!=UP && k.dir!=DOWN))
                throw std::invalid_argument("invalid hall key");
            int index=2*k.floor+(k.dir==UP);
            if(keys[index])throw std::invalid_argument("duplicate or overlapping hall call");
            keys[index]=true;
        };
        for(auto k:free)check(k);
        for(auto k:locked)check(k);

        std::vector<double> cn(floors+1),ce(floors+1),cv(floors+1);
        std::vector<double> rn(floors+1),rr(floors+1),ru(floors+1),re(floors+1);
        double low_r=0,low_u=0,all_low_v=0;
        for(size_t k=0;k<locked.size();++k) {
            auto key=locked[k];int z=orient(key.floor);
            if(first(key)) {
                auto r=row(key,locked_raw[k]);
                if(r.low){low_r+=r.r;low_u+=r.u;}
                else {rn[z+1]+=1;rr[z+1]+=r.high_r;ru[z+1]+=r.u;re[z+1]+=r.end;}
            } else if(second(key)) {
                auto c=column(key,locked_raw[k]);
                cn[z+1]+=1;ce[z+1]+=c.end;cv[z+1]+=c.high_v;all_low_v+=c.low_v;
            }
        }
        for(int z=1;z<=floors;++z) {
            for(auto* p:{&cn,&ce,&cv,&rn,&rr,&ru,&re})(*p)[z]+=(*p)[z-1];
        }
        std::vector<double> out(free.size(),0);
        for(size_t k=0;k<free.size();++k) {
            auto key=free[k];int z=orient(key.floor);
            if(first(key)) {
                auto r=row(key,free_raw[k]);
                if(r.low)out[k]=cn.back()*r.r+r.u*all_low_v;
                else out[k]=cn[z+1]*r.end+ce[z+1]+(cn.back()-cn[z+1])*r.high_r
                    +r.u*(cv.back()-cv[z+1]);
            } else if(second(key)) {
                auto c=column(key,free_raw[k]);
                out[k]=low_r+c.low_v*low_u+rr[z]+c.high_v*ru[z]
                    +re.back()-re[z]+(rn.back()-rn[z])*c.end;
            }
        }
        return out;
    }
};

inline std::optional<CrossPairFeatures> cross_features(const Building& b,const CarView& car) {

    if(car.dir==0)return std::nullopt;
    auto forward=forward_features(b,car,car.dir);
    if(!forward)return std::nullopt;
    CrossPairFeatures f;
    f.floors=b.floors;f.direction=car.dir;f.start=f.orient(car.committed);f.highest=f.start;
    f.gamma=b.kin.floor_height/b.kin.v_max;f.kappa=b.kin.v_max/b.kin.a_max+b.kin.settle;
    f.door=b.doors.open+b.doors.close;f.tau=b.doors.transfer;f.h0=f.kappa+f.door;f.q=f.h0+f.tau;
    f.release=car.release;f.open_at_start=!car.moving&&!car.arriving&&car.doors_open;
    f.known.assign(b.floors,0);f.above_count=f.known;f.above_load=f.known;
    f.below_delay.assign(b.floors,0);
    f.beta.assign(b.floors,0);
    for(auto [floor,count]:car.car_calls) {
        const int z=f.orient(floor);f.known[z]=count;
        if(z>=f.start)f.highest=std::max(f.highest,z);
    }
    int count=0,load=0;
    for(int i=b.floors-1;i>=0;--i) {
        f.above_count[i]=count;f.above_load[i]=load;
        if(f.known[i]){++count;load+=f.known[i];}
    }
    int prefix=0;
    for(int i=0;i<b.floors-1;++i) {
        prefix+=f.known[i]>0;
        if(i>=f.start)f.beta[i]=forward->a[i]+(f.q*(i+1)-f.h0*prefix)/(b.floors-1-i);
    }
    double cost=0;
    for(int j=f.start-1;j>=0;--j) {
        f.below_delay[j]=cost;
        if(f.known[j])cost+=f.h0+f.tau*f.known[j];
    }
    return f;
}
}
