#include "../research/profile_pair_features.hpp"
#include <iomanip>
#include <iostream>
#include <random>

using namespace egcs2017;
using namespace egcs2017::research;

int main() {
    try {
        std::mt19937 rng(330921);CostOptions raw;
        int states=0,pairs=0,rows=0;double error=0,row_error=0,uniform_error=0;
        for(int sample=0;sample<300;++sample) {
            Building b;b.floors=5+rng()%10;
            const int F=b.floors,R=1+sample%3,dir=sample%2?UP:DOWN,s=1+rng()%(F-3);
            auto floor=[&](int f){return dir==UP?f:F-1-f;};
            CarView c;c.dir=dir;c.committed=floor(s);c.x=b.pos(c.committed);c.release=.75;
            c.moving=sample%4==0;c.arriving=sample%4==1;c.doors_open=sample%4==2;
            if(c.moving) {c.v=.3+.01*(rng()%100);c.x-=dir*(b.kin.stop_distance(c.v)+.2);c.release=0;}
            for(int f=0;f<F;++f)if(rng()%4==0)c.car_calls[floor(f)]=1+rng()%3;
            std::vector<std::vector<double>> atom(R,std::vector<double>(F)),mix(F,std::vector<double>(R));
            for(auto& row:atom)for(auto& a:row)a=1+rng()%11;
            for(auto& row:mix) {for(auto& a:row)a=rng()%5;row[0]+=1;}
            auto f=profile_features(b,c,dir,atom,mix);
            if(!f)throw std::runtime_error("profile geometry unexpectedly rejected");
            std::vector<HallKey> ns,ls;
            for(int i=s;i<F-1;++i)(i%3?ns:ls).push_back({floor(i),dir});
            for(int i=s;i<F-1;++i)for(int j=i+1;j<F-1;++j) {
                HallKey ki{floor(i),dir},kj{floor(j),dir};
                double total=0,z=0;
                for(int d=i+1;d<F;++d) {
                    double mass=0;for(int r=0;r<R;++r)mass+=mix[i][r]*atom[r][d];
                    auto path=run_path(b,c,{{ki,floor(d),1,0},{kj,-1,1,1}},2);
                    total+=mass*(path.pickup[0]+path.pickup[1]);z+=mass;
                }
                const double ref=total/z-unary(b,c,ki,raw)-unary(b,c,kj,raw);
                error=std::max(error,std::abs(ref-f->pair(ki,kj)));++pairs;
            }
            auto sums=f->correction(ns,ls);
            for(size_t i=0;i<ns.size();++i) {
                double ref=0;for(auto l:ls)ref+=f->pair(ns[i],l);
                row_error=std::max(row_error,std::abs(ref-sums[i]));++rows;
            }
            std::vector<std::vector<double>> one(1,std::vector<double>(F,1)),weight(F,std::vector<double>(1,1));
            auto unif=profile_features(b,c,dir,one,weight);auto base=forward_features(b,c,dir);
            for(int i=s;i<F-1;++i)for(int j=i+1;j<F-1;++j)
                uniform_error=std::max(uniform_error,std::abs(unif->pair({floor(i),dir},{floor(j),dir})-base->pair({floor(i),dir},{floor(j),dir})));
            ++states;
        }
        Building b;CarView c;c.dir=UP;
        std::vector<std::vector<double>> zero(1,std::vector<double>(b.floors,0)),mix(b.floors,std::vector<double>(1,1));
        auto unsupported=profile_features(b,c,UP,zero,mix);
        if(unsupported->contains({1,UP}))throw std::runtime_error("zero-mass origin accepted");
        bool rejected=false;zero[0][0]=-1;
        try {profile_features(b,c,UP,zero,mix);}catch(const std::invalid_argument&) {rejected=true;}
        if(!rejected)throw std::runtime_error("negative profile accepted");
        if(error>1e-9 || row_error>1e-9 || uniform_error>1e-9)throw std::runtime_error("profile comparison failed");
        std::cout<<std::setprecision(17)<<"{\"states\":"<<states<<",\"pairs\":"<<pairs<<",\"lock_rows\":"<<rows
          <<",\"maximum_weighted_path_error\":"<<error<<",\"maximum_row_error\":"<<row_error
          <<",\"maximum_uniform_reduction_error\":"<<uniform_error<<"}\n";
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}
}
