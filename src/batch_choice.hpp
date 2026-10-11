#pragma once
#include <array>
#include <deque>
#include <cmath>
#include "dispatchers.hpp"
#include "batch_partition.hpp"

namespace egcs2017 {
struct BatchGeometry {
    int C=13, f=6, h=4, p=6;
    double tA=0,tA0=0,tBf=0,tBh=0,f0=0,h0=0,fh=0,d0=5,tau=1.2;
    double dwell(int n) const { return n ? d0+tau*n : 0; }
    double leg(int floor) const { return floor==f ? f0:h0; }
    double arrival(int floor) const { return floor==f ? tBf:tBh; }
};

inline double single_batch_sum(const BatchGeometry& g,int floor,double t,int n,bool wait) {
    int a=n/g.C,r=n%g.C;
    double d=g.dwell(g.C), cycle=2*d+2*g.leg(floor);
    double result=g.C*(a*(t+(wait?0:d+g.leg(floor)))+cycle*a*(a-1)/2);
    if(r) result+=r*(t+a*cycle+(wait?0:g.dwell(r)+g.leg(floor)));
    return result;
}

inline double batch_difference(const BatchGeometry& g,int m,int n,bool wait) {

    int first_n=std::min(m,g.C-g.p);
    double first_alight=g.tA+g.dwell(first_n)+g.f0;
    double keep=wait?first_n*g.tA:(g.p+first_n)*first_alight;
    keep+=single_batch_sum(g,g.f,first_alight+g.dwell(g.p+first_n)+g.f0,m-first_n,wait);
    keep+=single_batch_sum(g,g.h,g.tBh,n,wait);
    int high=std::max(g.f,g.h),low=std::min(g.f,g.h);
    int hi=g.f>g.h?m:n,lo=g.f>g.h?n:m;
    int a=hi/g.C,r=hi%g.C;
    double d=g.dwell(g.C),cycle=2*d+2*g.leg(high);
    double first=g.arrival(high)+d+g.leg(high);
    double take=(wait?0:g.p*g.tA0)+g.C*(a*(wait?g.arrival(high):first)+cycle*a*(a-1)/2);
    double release=first+(a-1)*cycle+d;
    if(r) {
        double arr=a?release+g.leg(high):g.arrival(high);
        int boarded=std::min(lo,g.C-r);
        double arrlow=arr+g.dwell(r)+g.fh;
        double alight=arrlow+g.dwell(boarded)+g.leg(low);
        take+=wait?r*arr+boarded*arrlow:(r+boarded)*alight;
        lo-=boarded;
        release=alight+g.dwell(r+boarded);
    }
    take+=single_batch_sum(g,low,release+g.leg(low),lo,wait);
    return take-keep;
}

struct ResidueMoment { int base; double mass,first,second; };
inline std::vector<ResidueMoment> geometric_moments(int C,double mean) {
    if(!std::isfinite(mean)||mean<1) throw std::invalid_argument("queue mean must be finite and >= 1");
    double r=1-1/mean,t=std::pow(r,C),den=1-t;
    std::vector<ResidueMoment> v;
    for(int m=1;m<C;++m) v.push_back({m,(1-r)*std::pow(r,m-1),0,0});
    for(int m=C;m<2*C;++m) {
        double pref=(1-r)*std::pow(r,m-1);
        v.push_back({m,pref/den,pref*t/(den*den),pref*t*(1+t)/(den*den*den)});
    }
    return v;
}

inline double expected_batch_difference(const BatchGeometry& g,double mean1,double mean2,bool wait) {
    auto v1=geometric_moments(g.C,mean1),v2=geometric_moments(g.C,mean2);
    double total=0;
    for(auto x:v1) for(auto y:v2) {
        if(x.mass==0||y.mass==0) continue;
        auto val=[&](int i,int j){return batch_difference(g,x.base+i*g.C,y.base+j*g.C,wait);};
        double d=val(0,0),a=0,b=0,aa=0,bb=0,ab=0;
        if(x.base>=g.C) {double y1=val(1,0);aa=(val(2,0)-2*y1+d)/2;a=y1-d-aa;}
        if(y.base>=g.C) {double y1=val(0,1);bb=(val(0,2)-2*y1+d)/2;b=y1-d-bb;}
        if(x.base>=g.C&&y.base>=g.C) ab=val(1,1)-val(1,0)-val(0,1)+d;
        total+=d*x.mass*y.mass+a*x.first*y.mass+b*x.mass*y.first
            +aa*x.second*y.mass+bb*x.mass*y.second+ab*x.first*y.first;
    }
    return total;
}

struct BatchTransition {int load; double t,j,w;};
inline BatchTransition batch_floor_transition(const Building& b,int f,int next,int p,int m,double t) {
    int C=b.capacity,space=C-p,out;
    auto d=[&](int n){return b.doors.dwell(n,0);};
    double j=0,w,end;
    if(m<space) {w=m*t;out=p+m;end=t+d(m);}
    else {
        w=space*t;double drop=t+d(space)+b.kin.travel(f,0);j=C*drop;
        double release=drop+d(C);int a=(m-space)/C,r=(m-space)%C;
        if(a) {
            double first=release+b.kin.travel(0,f),cycle=2*d(C)+2*b.kin.travel(f,0);
            double prog=cycle*a*(a-1)/2;
            w+=C*(a*first+prog);j+=C*(a*(first+d(C)+b.kin.travel(f,0))+prog);
            release=first+(a-1)*cycle+2*d(C)+b.kin.travel(f,0);
        }
        out=r;
        if(r) {double last=release+b.kin.travel(0,f);w+=r*last;end=last+d(r);}
        else end=release;
    }
    double arrival=end+b.kin.travel(out?f:0,next);
    if(out&&next==0) {j+=out*arrival;arrival+=d(out);out=0;}
    return {out,arrival,j,w};
}

inline std::pair<double,double> batch_route_value(const Building& b,int p,double first,
                                                 const std::vector<std::pair<int,double>>& queues,bool mean_only) {

    int C=b.capacity;std::vector<double> prob(C),tm(C);prob[p]=1;tm[p]=first;
    double journey=0,waiting=0;
    for(size_t ix=0;ix<queues.size();++ix) {
        int f=queues[ix].first,next=ix+1<queues.size()?queues[ix+1].first:0;
        auto law=geometric_moments(C,queues[ix].second);
        if(mean_only) {
            int n=std::max(1,int(std::lround(queues[ix].second))),base=n<C?n:C+n%C,k=(n-base)/C;
            law={{base,1,double(k),double(k)*k}};
        }
        std::vector<double> pp(C),tt(C);
        for(int load=0;load<C;++load)if(prob[load]>0)for(auto r:law)if(r.mass>0) {
            std::array<BatchTransition,3> v;
            for(int k=0;k<3;++k)v[k]=batch_floor_transition(b,f,next,load,r.base+k*C,0);
            auto s0=batch_floor_transition(b,f,next,load,r.base,1);
            auto s1=batch_floor_transition(b,f,next,load,r.base+C,1);
            auto avg=[&](double BatchTransition::*member) {
                double y0=v[0].*member,y1=v[1].*member,y2=v[2].*member;
                double a2=r.base>=C?(y2-2*y1+y0)/2:0,a1=r.base>=C?y1-y0-a2:0;
                double slope0=s0.*member-y0,slope1=r.base>=C?(s1.*member-y1)-slope0:0;
                return prob[load]*(y0*r.mass+a1*r.first+a2*r.second)+tm[load]*(slope0*r.mass+slope1*r.first);
            };
            pp[v[0].load]+=prob[load]*r.mass;tt[v[0].load]+=avg(&BatchTransition::t);
            journey+=avg(&BatchTransition::j);waiting+=avg(&BatchTransition::w);
        }
        prob=std::move(pp);tm=std::move(tt);
    }
    return {journey,waiting};
}

class BatchChoiceDispatcher : public Dispatcher {
    std::unique_ptr<Dispatcher> base_=make_dispatcher("paper2017_ls_lk");
    std::map<HallKey,std::deque<std::pair<double,int>>> boardings_;
    bool mean_only_;
    bool route_mode_;
    bool reference_counts_;
    bool global_mode_, diagnostic_;
    double mean(const CallView& call,double now) {
        if(reference_counts_)return std::max(1,call.waiting);
        auto& q=boardings_[call.key];
        while(!q.empty()&&q.front().first<now-300) q.pop_front();
        int count=0;for(auto item:q)count+=item.second;
        return 1+count/std::min(300.0,std::max(1.0,now))*call.waited;
    }
public:
    explicit BatchChoiceDispatcher(bool mean_only=false,bool route_mode=false,bool reference_counts=false,bool global_mode=false,bool diagnostic=false):mean_only_(mean_only),route_mode_(route_mode),reference_counts_(reference_counts),global_mode_(global_mode),diagnostic_(diagnostic){}
    std::string name() const override {if(global_mode_)return diagnostic_?"batch_global_diag":(mean_only_?"batch_global_mean":"batch_global_expected");return reference_counts_?"batch_route_reference_counts":(route_mode_?(mean_only_?"batch_route_mean":"batch_route_expected"):(mean_only_?"batch_mean":"batch_expected"));}
    void observe_boarding(double t,int f,int d,int n) override {boardings_[{f,d}].push_back({t,n});}
    std::map<HallKey,int> decide(const Building& b,const std::vector<CarView>& cars,
                               const std::vector<CallView>& calls,const HallKey& nc,DecisionTrace* trace) override {
        auto result=base_->decide(b,cars,calls,nc,trace);
        std::map<int,std::vector<const CallView*>> assigned;
        double now=0;
        for(auto& call:calls) {
            now=std::max(now,call.registered+call.waited);
            auto it=result.find(call.key);
            if(it!=result.end()&&it->second>=0) assigned[it->second].push_back(&call);
        }
        if(route_mode_) {

            auto value=[&](const CarView& car,const std::vector<const CallView*>& work,
                           std::pair<double,double>& answer) {
                if(car.arriving||car.doors_open||car.load>=b.capacity) return false;
                if(car.load>0&&(car.car_calls.size()!=1||car.car_calls.begin()->first!=0||car.dir!=DOWN))return false;
                if(car.load==0&&!car.car_calls.empty())return false;
                if(work.empty()) {
                    if(!car.load){answer={0,0};return true;}
                    answer={car.load*(car.release+b.kin.time_from_state(car.x,car.moving?car.v:0)),0};return true;
                }
                std::vector<std::pair<int,double>> queues;
                for(auto c:work) {
                    if(c->key.dir!=DOWN||c->key.floor<=0)return false;
                    queues.emplace_back(c->key.floor,mean(*c,now));
                }
                std::sort(queues.begin(),queues.end(),[](auto x,auto y){return x.first>y.first;});
                int f=queues[0].first;double target=b.pos(f),dist=std::abs(target-car.x),v=car.moving?car.v:0;
                if(car.load&&target>=car.x)return false;
                if(car.moving&&((target-car.x)*car.dir<0||(f-car.committed)*car.dir<0))return false;
                if(dist+1e-8<b.kin.stop_distance(v))return false;
                answer=batch_route_value(b,car.load,car.release+b.kin.time_from_state(dist,v),queues,mean_only_);
                return true;
            };
            std::map<int,std::pair<double,double>> basecost;
            for(auto& car:cars){std::pair<double,double> x;if(value(car,assigned[car.id],x))basecost[car.id]=x;}
            if(global_mode_) {
                std::vector<const CarView*> eligible_cars;
                std::vector<const CallView*> movable;
                for(const auto& car:cars)if(basecost.count(car.id))eligible_cars.push_back(&car);
                for(auto car:eligible_cars)for(auto call:assigned[car->id])if(!call->close_to_service)movable.push_back(call);
                int n=movable.size();
                if(trace){trace->method=name();trace->after=result;}
                if(n<1||n>8||eligible_cars.size()<2)return result;
                int size=1<<n;
                std::vector<std::vector<PartitionCost>> table(eligible_cars.size(),std::vector<PartitionCost>(size));
                std::vector<int> base_masks(eligible_cars.size());double basej=0,basew=0;
                for(size_t c=0;c<eligible_cars.size();++c) {
                    const auto& car=*eligible_cars[c];
                    std::vector<const CallView*> fixed;
                    for(auto call:assigned[car.id])if(call->close_to_service)fixed.push_back(call);
                    for(int i=0;i<n;++i)if(result[movable[i]->key]==car.id)base_masks[c]|=1<<i;
                    for(int mask=0;mask<size;++mask) {
                        auto work=fixed;bool allowed=true;
                        for(int i=0;i<n;++i)if(mask&(1<<i)) {
                            if(movable[i]->forbidden_car==car.id){allowed=false;break;}
                            work.push_back(movable[i]);
                        }
                        std::pair<double,double> v;
                        if(allowed&&value(car,work,v))table[c][mask]={v.first,v.second,true};
                    }
                    auto v=table[c][base_masks[c]];
                    if(!v.valid)throw std::logic_error("base subset invalid");
                    basej+=v.journey;basew+=v.waiting;
                }

                double singlej=basej;
                for(int i=0;i<n;++i)for(size_t a=0;a<base_masks.size();++a)if(base_masks[a]&(1<<i))
                    for(size_t bidx=0;bidx<base_masks.size();++bidx)if(a!=bidx) {
                        auto va=table[a][base_masks[a]^(1<<i)],vb=table[bidx][base_masks[bidx]|(1<<i)];
                        if(!va.valid||!vb.valid)continue;
                        auto oa=table[a][base_masks[a]],ob=table[bidx][base_masks[bidx]];
                        double w=basew+va.waiting+vb.waiting-oa.waiting-ob.waiting;
                        double j=basej+va.journey+vb.journey-oa.journey-ob.journey;
                        if(w<=basew+1e-8)singlej=std::min(singlej,j);
                    }
                auto answer=partition_pareto(table,n,basew+1e-8);
                if(trace){trace->batch_global_complete=answer.complete;trace->batch_global_states=answer.transitions;trace->batch_single_journey=singlej-basej;}
                if(answer.complete&&answer.feasible&&answer.journey<basej-1e-7&&answer.waiting<=basew+1e-8) {
                    if(!diagnostic_)for(size_t c=0;c<eligible_cars.size();++c)for(int i=0;i<n;++i)
                        if(answer.masks[c]&(1<<i))result[movable[i]->key]=eligible_cars[c]->id;
                    if(trace){trace->batch_global_proposed=true;trace->batch_changed=!diagnostic_;trace->batch_wait=answer.waiting-basew;trace->batch_journey=answer.journey-basej;trace->after=result;}
                }
                return result;
            }
            double best=0,bw=0;int target=-1,eligible=0;HallKey key{-1,0};
            for(auto& A:cars)if(basecost.count(A.id))for(auto c:assigned[A.id])if(!c->close_to_service) {
                auto left=assigned[A.id];left.erase(std::find(left.begin(),left.end(),c));std::pair<double,double> va;
                if(!value(A,left,va))continue;
                for(auto& B:cars)if(B.id!=A.id&&basecost.count(B.id)&&c->forbidden_car!=B.id) {
                    auto right=assigned[B.id];right.push_back(c);std::pair<double,double> vb;
                    if(!value(B,right,vb))continue;
                    ++eligible;
                    double j=va.first+vb.first-basecost[A.id].first-basecost[B.id].first;
                    double w=va.second+vb.second-basecost[A.id].second-basecost[B.id].second;
                    if(w<=-1e-7&&j<best-1e-7){best=j;bw=w;target=B.id;key=c->key;}
                }
            }
            if(target>=0)result[key]=target;
            if(trace){trace->method=name();trace->after=result;trace->batch_eligible=eligible;trace->batch_changed=target>=0;trace->batch_wait=bw;trace->batch_journey=best;}
            return result;
        }
        double best=0,best_wait=0; HallKey chosen{-1,0};int car_id=-1,eligible=0;
        for(auto& A:cars) {
            if(A.load<1||A.load>=b.capacity||A.dir!=DOWN||A.arriving||A.doors_open
                ||A.car_calls.size()!=1||A.car_calls.begin()->first!=0||assigned[A.id].size()!=1) continue;
            const auto& call=*assigned[A.id][0]; int f=call.key.floor;
            if(call.close_to_service||call.key.dir!=DOWN||f<=0||b.pos(f)>=A.x||f>A.committed)continue;
            double va=A.moving?A.v:0;
            if(A.x-b.pos(f)+1e-8<b.kin.stop_distance(va))continue;
            for(auto& B:cars) {
                if(B.id==A.id||B.load||!B.car_calls.empty()||B.arriving||B.doors_open
                    ||assigned[B.id].size()!=1||call.forbidden_car==B.id)continue;
                const auto& other=*assigned[B.id][0];int h=other.key.floor;

                if(h<=0||h==f||other.key.dir!=DOWN||other.close_to_service
                    ||B.x>=b.pos(std::min(f,h))||(B.moving&&B.dir!=UP))continue;
                double vb=B.moving?B.v:0;
                if(b.pos(std::min(f,h))-B.x+1e-8<b.kin.stop_distance(vb))continue;
                BatchGeometry g;g.C=b.capacity;g.f=f;g.h=h;g.p=A.load;
                g.d0=b.doors.open+b.doors.close;g.tau=b.doors.transfer;
                g.f0=b.kin.travel(f,0);g.h0=b.kin.travel(h,0);g.fh=b.kin.travel(f,h);
                g.tA=A.release+b.kin.time_from_state(A.x-b.pos(f),va);
                g.tA0=A.release+b.kin.time_from_state(A.x,va);
                g.tBf=B.release+b.kin.time_from_state(b.pos(f)-B.x,vb);
                g.tBh=B.release+b.kin.time_from_state(b.pos(h)-B.x,vb);
                double mu=mean(call,now),nu=mean(other,now),w,j;
                ++eligible;
                if(mean_only_) {

                    int m=std::max(1,int(std::lround(mu))),n=std::max(1,int(std::lround(nu)));
                    w=batch_difference(g,m,n,true);j=batch_difference(g,m,n,false);
                } else {
                    w=expected_batch_difference(g,mu,nu,true);j=expected_batch_difference(g,mu,nu,false);
                }

                if(w<=-1e-7&&j<best-1e-7) {best=j;best_wait=w;chosen=call.key;car_id=B.id;}
            }
        }
        if(car_id>=0)result[chosen]=car_id;
        if(trace) {
            trace->method=name();trace->after=result;
            trace->batch_eligible=eligible;trace->batch_changed=car_id>=0;
            trace->batch_wait=best_wait;trace->batch_journey=best;
        }
        return result;
    }
};
}
