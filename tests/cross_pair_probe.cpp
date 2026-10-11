#include "../research/cross_pair_features.hpp"
#include <iomanip>
#include <iostream>
#include <random>
using namespace egcs2017;
using namespace egcs2017::research;
int main(int argc,char** argv) {
 try {
    if(argc==2 && std::string(argv[1])=="--table") {
        Building b;b.floors=8;CarView car;car.dir=UP;car.committed=0;car.x=0;CostOptions raw;
        auto feature=cross_features(b,car);
        std::cout<<std::setprecision(17)<<"{\"empty_matrix\":[";
        for(int i=1;i<=6;++i) {
            if(i>1)std::cout<<',';std::cout<<'[';
            for(int j=1;j<=6;++j) {
                if(j>1)std::cout<<',';HallKey a{i,UP},z{j,DOWN};
                std::cout<<feature->pair(a,z,unary(b,car,a,raw),unary(b,car,z,raw));
            }
            std::cout<<']';
        }
        car.car_calls[4]=3;feature=cross_features(b,car);
        std::cout<<"],\"merged_triplet\":[";
        for(int j=3;j<=5;++j) {
            if(j>3)std::cout<<',';HallKey a{2,UP},z{j,DOWN};
            std::cout<<feature->pair(a,z,unary(b,car,a,raw),unary(b,car,z,raw));
        }
        car.dir=0;if(cross_features(b,car))throw std::runtime_error("idle direction accepted");
        car.dir=UP;b.kin.floor_height=1;
        if(cross_features(b,car))throw std::runtime_error("triangular rest travel accepted");
        std::cout<<"],\"rejected_domains\":2}\n";return 0;
    }
    std::mt19937 rng(3501);long pairs=0,states=0,moving=0,arriving=0,open=0,rows=0;
    double error=0,generator_error=0,row_error=0;CostOptions raw;
    for(int rep=0;rep<1200;++rep) {
        Building b;b.floors=4+rng()%15;
        b.kin.v_max=1.0+0.2*(rep%5);b.kin.a_max=0.6+0.1*(rep%4);
        b.kin.floor_height=b.kin.v_max*b.kin.v_max/b.kin.a_max+0.1*(rep%3);
        b.kin.settle=0.1+0.1*(rep%6);
        b.doors.open=1.0+0.25*(rep%7);b.doors.close=1.5+0.3*(rep%9);
        b.doors.transfer=0.4+0.2*(rep%8);
        CarView car;car.id=0;car.dir=rep%2?UP:DOWN;

        const int s=rep%4==0 ? 1+rng()%(b.floors-2) : rng()%(b.floors-1);
        auto orig=[&](int i){return car.dir==UP?i:b.floors-1-i;};
        car.committed=orig(s);car.x=b.pos(car.committed);car.release=(rng()%40)*0.125;
        if(rep%4==0) {
            car.moving=true;car.v=b.kin.v_max*(rng()%16+1)/16.0;
            double distance=b.kin.stop_distance(car.v)+0.05;
            car.x-=car.dir*distance;car.release=0;++moving;
        } else if(rep%4==1){car.arriving=true;++arriving;}
        else if(rep%4==2){car.doors_open=true;++open;}
        for(int z=0;z<b.floors;++z)if(rng()%4==0)car.car_calls[orig(z)]=1+rng()%3;
        auto feature=cross_features(b,car);
        if(!feature)throw std::runtime_error("valid synthetic car rejected");
        ++states;
        for(int i=s;i<b.floors-1;++i)for(int j=1;j<b.floors;++j) {
            HallKey a{orig(i),car.dir},z{orig(j),-car.dir};
            const double ua=unary(b,car,a,raw),uz=unary(b,car,z,raw);
            const double ref=pairwise(b,car,a,z,ua,uz);
            const double value=feature->pair(a,z,ua,uz);
            const double byturn=feature->pair_by_turnaround(a,z,ua,uz);
            const double e=std::abs(value-ref);
            if(e>1e-8) {
                std::cerr<<"rep="<<rep<<" F="<<b.floors<<" s="<<s<<" i="<<i<<" j="<<j
                  <<" moving="<<car.moving<<" arriving="<<car.arriving<<" open="<<car.doors_open
                  <<" ref="<<ref<<" formula="<<value<<" direct="<<byturn<<'\n';
                throw std::runtime_error("cross-pair mismatch");
            }
            error=std::max(error,e);generator_error=std::max(generator_error,std::abs(value-byturn));++pairs;
        }
        std::vector<HallKey> free,locked;std::vector<double> uf,ul;
        for(int z=0;z<b.floors;++z)for(int direction:{UP,DOWN}) {
            if((z==0&&direction==DOWN)||(z==b.floors-1&&direction==UP))continue;
            HallKey key{z,direction};int which=rng()%3;
            if(which==0){free.push_back(key);uf.push_back(unary(b,car,key,raw));}
            if(which==1){locked.push_back(key);ul.push_back(unary(b,car,key,raw));}
        }
        auto aggregate=feature->correction(free,locked,uf,ul);
        for(size_t i=0;i<free.size();++i) {
            double ref=0;
            for(size_t j=0;j<locked.size();++j)if(feature->contains(free[i],locked[j]))
                ref+=pairwise(b,car,free[i],locked[j],uf[i],ul[j]);
            row_error=std::max(row_error,std::abs(aggregate[i]-ref));++rows;
        }
    }
    if(row_error>1e-8)throw std::runtime_error("cross correction mismatch");
    std::cout<<std::setprecision(17)<<"{\"states\":"<<states<<",\"pairs\":"<<pairs
        <<",\"moving_states\":"<<moving<<",\"arriving_states\":"<<arriving<<",\"open_states\":"<<open
        <<",\"maximum_pair_error\":"<<error<<",\"maximum_generator_error\":"<<generator_error
        <<",\"rows\":"<<rows<<",\"maximum_row_error\":"<<row_error<<"}\n";
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}
}
