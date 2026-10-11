#include <bit>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <sstream>
#include "../research/structured_policy.hpp"
#include "traffic.hpp"

using namespace egcs2017;
using namespace egcs2017::research;

namespace {
struct Audit {
    std::vector<std::string> inputs;
    std::vector<std::map<HallKey,int>> outputs;
    int moving=0,arriving=0,open=0,locked=0,removed=0;
    double first_time=-1;
};
std::string input_record(double t,const std::vector<CarView>& cars,
        const std::vector<CallView>& calls,HallKey nc) {
    std::ostringstream s;s<<std::hexfloat<<t<<' '<<nc.floor<<' '<<nc.dir<<' '<<cars.size();
    for(const auto& c:cars) {
        s<<" car "<<c.id<<' '<<c.x<<' '<<c.v<<' '<<c.dir<<' '<<c.moving<<' '
         <<c.arriving<<' '<<c.committed<<' '<<c.release<<' '<<c.doors_open<<' '<<c.load;
        s<<" buttons "<<c.car_calls.size();
        for(auto [f,n]:c.car_calls)s<<' '<<f<<' '<<n;
        s<<" assigned "<<c.assigned.size();
        for(auto k:c.assigned)s<<' '<<k.floor<<' '<<k.dir;
    }
    s<<" calls "<<calls.size();
    for(const auto& c:calls)s<<' '<<c.key.floor<<' '<<c.key.dir<<' '<<c.registered<<' '
        <<c.assigned<<' '<<c.close_to_service<<' '<<c.waiting<<' '<<c.waited<<' '<<c.forbidden_car;
    return s.str();
}
class RecordingDispatcher final:public Dispatcher {
    std::unique_ptr<Dispatcher> inner;Audit& audit;
public:
    RecordingDispatcher(bool structured,Audit& a):
        inner(structured?std::unique_ptr<Dispatcher>(new StructuredDispatcher):
                         make_dispatcher("paper2017_ls_lk")),audit(a) {}
    std::string name() const override{return inner->name();}
    std::map<HallKey,int> decide(const Building& b,const std::vector<CarView>& cars,
            const std::vector<CallView>& calls,const HallKey& nc,DecisionTrace* trace) override {
        if(!trace)throw std::logic_error("engine omitted trace timestamp");
        if(audit.inputs.empty())audit.first_time=trace->t;
        audit.inputs.push_back(input_record(trace->t,cars,calls,nc));
        bool mv=false,ar=false,op=false,lk=false;
        for(const auto& c:cars){mv|=c.moving;ar|=c.arriving;op|=c.doors_open;}
        for(const auto& c:calls)lk|=c.close_to_service&&!(c.key==nc);
        audit.moving+=mv;audit.arriving+=ar;audit.open+=op;audit.locked+=lk;
        auto out=inner->decide(b,cars,calls,nc,trace);
        for(const auto& c:calls)if((!c.close_to_service||c.key==nc)&&out.at(c.key)<0)
            ++audit.removed;
        audit.outputs.push_back(out);return out;
    }
};
struct Run {
    Audit audit;std::vector<Passenger> people;
    std::vector<std::string> stops;Metrics metrics;
};
Run run(const Building& b,int C,double rate,double theta,double period,double duration,bool structured) {
    Run r;SimOptions opt;opt.theta=theta;opt.period=period;
    opt.check_invariants=true;opt.stop_log=true;opt.button_info=true;
    opt.win0=0;opt.win1=duration;

    const unsigned long traffic_seed=b.floors*101UL+C*7UL+static_cast<unsigned long>(rate*10);
    auto people=interfloor_traffic(b.floors,rate,30,duration,traffic_seed);
    Simulation sim(b,C,std::make_unique<RecordingDispatcher>(structured,r.audit),people,
                   nullptr,14400,opt);
    sim.run();r.people=sim.passengers();r.metrics=sim.metrics();
    if(!sim.calls().empty()||!sim.waiting().empty()||r.metrics.unserved||r.metrics.unfinished)
        throw std::runtime_error("run did not drain");
    for(const auto& c:sim.cars())if(c.load())throw std::runtime_error("onboard passenger after drain");
    for(const auto& x:sim.stop_log()) {
        std::ostringstream s;s<<std::hexfloat<<x.t0<<' '<<x.duration<<' '<<x.flight_t0<<' '
            <<x.car<<' '<<x.floor<<' '<<x.dir<<' '<<x.continuing<<' '<<x.boarded<<' '
            <<x.alight<<' '<<x.pickup_only<<' '<<x.in_window;
        r.stops.push_back(s.str());
    }
    return r;
}
bool same(double a,double b){return std::bit_cast<std::uint64_t>(a)==std::bit_cast<std::uint64_t>(b);}
template<class T> std::pair<int,int> differences(const std::vector<T>& a,const std::vector<T>& b) {
    int count=static_cast<int>(a.size()>b.size()?a.size()-b.size():b.size()-a.size()),first=-1;
    for(size_t i=0;i<std::min(a.size(),b.size());++i)if(a[i]!=b[i]){++count;if(first<0)first=static_cast<int>(i);}
    if(first<0&&count)first=static_cast<int>(std::min(a.size(),b.size()));
    return {count,first};
}
bool check(const Building& b,int C,double rate,double theta,double period,double duration) {
    const auto old=run(b,C,rate,theta,period,duration,false);
    const auto fresh=run(b,C,rate,theta,period,duration,true);
    auto [inputs,first_input]=differences(old.audit.inputs,fresh.audit.inputs);
    auto [outputs,first_output]=differences(old.audit.outputs,fresh.audit.outputs);
    auto [stops,first_stop]=differences(old.stops,fresh.stops);
    if(old.people.size()!=fresh.people.size())throw std::logic_error("arrival stream changed");
    int passengers=0;
    for(size_t i=0;i<old.people.size();++i) {
        const auto& a=old.people[i];const auto& z=fresh.people[i];
        if(a.id!=z.id||a.origin!=z.origin||a.dest!=z.dest||!same(a.arrival,z.arrival))
            throw std::logic_error("arrival stream mismatch");
        passengers+=a.car!=z.car||!same(a.boarded,z.boarded)||!same(a.alighted,z.alighted);
    }
    std::cout<<std::setprecision(17)<<"{\"floors\":"<<b.floors<<",\"cars\":"<<C
        <<",\"rate\":"<<rate<<",\"theta\":"<<theta<<",\"period\":"<<period
        <<",\"floor_height\":"<<b.kin.floor_height<<",\"duration\":"<<duration
        <<",\"traffic_seed\":"<<(b.floors*101UL+C*7UL+static_cast<unsigned long>(rate*10))
        <<",\"first_decision_time\":"<<old.audit.first_time
        <<",\"seed\":0,\"decisions\":"<<old.audit.inputs.size()
        <<",\"structured_decisions\":"<<fresh.audit.inputs.size()<<",\"passengers\":"<<old.people.size()
        <<",\"stops\":"<<old.stops.size()<<",\"input_changes\":"<<inputs
        <<",\"output_changes\":"<<outputs<<",\"passenger_changes\":"<<passengers
        <<",\"stop_changes\":"<<stops<<",\"first_input_change\":"<<first_input
        <<",\"first_output_change\":"<<first_output<<",\"first_stop_change\":"<<first_stop
        <<",\"moving_decisions\":"<<old.audit.moving<<",\"arriving_decisions\":"<<old.audit.arriving
        <<",\"open_decisions\":"<<old.audit.open<<",\"locked_decisions\":"<<old.audit.locked
        <<",\"removed_labels\":"<<old.audit.removed
        <<",\"awt\": ["<<old.metrics.awt<<','<<fresh.metrics.awt<<"]"
        <<",\"att\": ["<<old.metrics.att<<','<<fresh.metrics.att<<"]"
        <<",\"ajt\": ["<<old.metrics.ajt<<','<<fresh.metrics.ajt<<"]}"<<std::endl;
    return !inputs&&!outputs&&!passengers&&!stops;
}
void boundary() {
    Building b;b.floors=8;std::vector<CarView> cars(2);
    cars[0].id=0;cars[1].id=1;cars[0].dir=cars[1].dir=UP;
    cars[1].release=12.179999999998994;
    std::vector<CallView> calls{{{2,UP},0,0,true},{{5,UP},1,-1,false}};
    StructuredDispatcher candidate;auto old=make_dispatcher("paper2017_ls_lk");
    DecisionTrace trace;trace.t=123;
    const auto a=old->decide(b,cars,calls,{5,UP},nullptr);
    const auto z=candidate.decide(b,cars,calls,{5,UP},&trace);
    const auto adapter=structured_decide(b,cars,calls,{5,UP},PairKernel::Combined);
    if(a.at({5,UP})!=0||z.at({5,UP})!=1||z.at({2,UP})!=0||z!=adapter.after||trace.t!=123)
        throw std::logic_error("numeric contract witness lost");
    if(trace.unary!=adapter.matrices.u||trace.pair!=adapter.matrices.w||
       trace.unary_lk!=adapter.matrices.u_lk||trace.g_final!=adapter.final_cost||
       !trace.p.empty()||!trace.order.empty())throw std::logic_error("trace not actual policy values");
    if(std::fesetround(FE_UPWARD))throw std::runtime_error("cannot test rounding mode");
    bool rejected=false;
    try{candidate.decide(b,cars,calls,{5,UP},nullptr);}catch(const std::invalid_argument&){rejected=true;}
    std::fesetround(FE_TONEAREST);
    if(!rejected)throw std::logic_error("unsupported rounding mode accepted");
    std::cout<<"{\"legacy_car\":0,\"structured_car\":1,\"locked_car\":0,\"rounding_rejected\":true}\n";
}
}
int main(int argc,char** argv) {
    try {
        if(argc==2&&std::string(argv[1])=="--boundary"){boundary();return 0;}
        if(argc!=1)throw std::invalid_argument("stdin: floors cars rate theta period floor_height duration");
        int F,C;double rate,theta,period,height,duration;bool ok=true;int count=0;
        while(std::cin>>F>>C>>rate>>theta>>period>>height>>duration) {
            if(F<3||C<1||rate<=0||theta<0||period<0||height<=0||duration<=0)
                throw std::invalid_argument("invalid replay setting");
            Building b;b.floors=F;b.kin.floor_height=height;
            ok=check(b,C,rate,theta,period,duration)&&ok;++count;
        }
        if(!std::cin.eof()||!count)throw std::invalid_argument("empty/malformed replay settings");
        return ok?0:3;
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}
}
