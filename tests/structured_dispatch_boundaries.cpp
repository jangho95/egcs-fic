#include "../research/dispatch_audit_io.hpp"
#include <iomanip>
#include <limits>

using namespace egcs2017;
using namespace egcs2017::research;
using namespace egcs2017::research::audit;

int main() {
    try {
        const auto opt=locked_ls_options();SubmodularDispatcher original(opt,"paper2017_ls_lk");
        int contracts=0;double max_error=0;
        auto equal=[&](const Building& b,const std::vector<CarView>& cars,
                const std::vector<CallView>& calls,HallKey nc) {
            const auto M=build_matrices(b,cars,calls,nc,opt);
            const auto ref=original.decide(b,cars,calls,nc,nullptr);
            auto out=structured_decide(b,cars,calls,nc,PairKernel::Combined);
            max_error=std::max(max_error,matrix_error(M,out.matrices));
            if(ref!=out.after)throw std::runtime_error("constructed contract changed");
            ++contracts;return out;
        };
        Building b;b.floors=8;
        std::vector<CarView> cars(2);cars[0].id=2;cars[1].id=9;
        for(auto& c:cars){c.dir=UP;c.load=11;}
        CallView lock{{1,UP},0,2,true},free{{3,DOWN},1,-1,false},fresh{{5,UP},2,-1,true};
        const auto full=equal(b,cars,{lock,free,fresh},fresh.key);
        if(full.removed!=2)throw std::runtime_error("postprocessing not exercised");
        equal(b,cars,{lock},no_new_call());
        for(auto& c:cars)c.load=0;
        fresh.close_to_service=false;fresh.forbidden_car=2;
        equal(b,cars,{lock,fresh},fresh.key);
        fresh.forbidden_car=-1;
        for(int mode=0;mode<4;++mode) {
            cars[0].committed=1;cars[0].x=b.pos(1);cars[0].v=0;
            cars[0].arriving=mode==1;cars[0].doors_open=mode==2;cars[0].moving=mode==3;
            if(cars[0].moving){cars[0].x=b.pos(1)-1;cars[0].v=1;}
            cars[0].car_calls={{4,3}};
            equal(b,cars,{lock,free,fresh},fresh.key);
        }
        b.kin.floor_height=1;cars[0].moving=false;
        equal(b,cars,{lock,free,fresh},fresh.key);

        std::cout<<std::setprecision(17)<<"{\"contract_cases\":"<<contracts
            <<",\"maximum_contract_matrix_error\":"<<max_error<<",\"boundary_groups\":[";
        for(int group=0;group<6;++group) {
            b=Building{};b.floors=8;cars=std::vector<CarView>(2);
            cars[0].id=0;cars[1].id=1;cars[0].dir=cars[1].dir=UP;
            HallKey lk=group==0?HallKey{2,UP}:group%2?HallKey{2,UP}:HallKey{5,DOWN};
            HallKey fk=group==0?HallKey{5,UP}:group%2?HallKey{5,DOWN}:HallKey{2,UP};
            if(group>=3)cars[0].car_calls={{4,3}};
            if(group==5){cars[0].moving=true;cars[0].committed=1;cars[0].x=1;cars[0].v=1;}
            lock=CallView{lk,0,0,true};fresh=CallView{fk,1,-1,false};
            const auto M=build_matrices(b,cars,{lock,fresh},fk,opt);
            const double center=M.u[0][0]-M.u[0][1]-1e-12;
            if(center<0)throw std::runtime_error("negative release near tie");
            int changed[3]={};double loss[3]={};int cases=0;bool first=true;
            if(group)std::cout<<',';
            std::cout<<"{\"group\":"<<group<<",\"center\":"<<center<<",\"witnesses\":[";
            for(int offset=-64;offset<=64;++offset) {
                double t=center;
                for(int k=0;k<std::abs(offset);++k)t=std::nextafter(t,offset<0?
                    -std::numeric_limits<double>::infinity():std::numeric_limits<double>::infinity());
                cars[1].release=t;
                const auto ref=original.decide(b,cars,{lock,fresh},fk,nullptr);
                const auto R=build_matrices(b,cars,{lock,fresh},fk,opt);++cases;
                for(int k=0;k<3;++k) {
                    auto out=structured_decide(b,cars,{lock,fresh},fk,static_cast<PairKernel>(k));
                    if(out.after!=ref) {
                        ++changed[k];loss[k]=std::max(loss[k],std::abs(R.u[0][out.final[0]]-R.u[0][ref.at(fk)]));
                        if(!first)std::cout<<',';first=false;
                        std::cout<<"{\"kernel\":"<<k<<",\"release\":"<<t<<",\"reference_car\":"<<ref.at(fk)
                            <<",\"car\":"<<out.after.at(fk)<<",\"reference_u\":["<<R.u[0][0]<<','<<R.u[0][1]
                            <<"],\"kernel_u\":["<<out.matrices.u[0][0]<<','<<out.matrices.u[0][1]<<"]}";
                    }
                }
            }
            std::cout<<"],\"cases\":"<<cases<<",\"changed\":["<<changed[0]<<','<<changed[1]<<','<<changed[2]
                <<"],\"maximum_original_cost_difference\":["<<loss[0]<<','<<loss[1]<<','<<loss[2]<<"]}";
        }
        std::cout<<"]}\n";
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}
}
