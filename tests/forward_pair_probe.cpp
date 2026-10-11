#include "../research/forward_pair_features.hpp"
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
using namespace egcs2017;
using namespace egcs2017::research;

int main() {
    try {
        std::mt19937 rng(32109);
        int states=0,pairs=0,rows=0,moving=0,arriving=0,open=0;
        double error=0, sum_error=0;
        CostOptions raw;
        for(int sample=0;sample<1000;++sample) {
            Building b; b.floors=3+int(rng()%16);
            if(sample%3==1) { b.kin.v_max=.8; b.kin.a_max=.6; b.kin.settle=.2; }
            if(sample%7==0) { b.doors.open=1.3; b.doors.close=2.7; b.doors.transfer=.6; }
            const int direction=sample%2?UP:DOWN, s=int(rng()%(b.floors-2));
            auto floor=[&](int z){return direction==UP?z:b.floors-1-z;};
            CarView c; c.id=0; c.dir=direction; c.committed=floor(s); c.x=b.pos(c.committed);
            c.release=.2+double(rng()%300)/100;
            c.moving=(s>0 && sample%4==0);
            if(c.moving) {
                c.v=b.kin.v_max*double(1+rng()%100)/100;
                c.x-=direction*(b.kin.stop_distance(c.v)+.3); c.release=0;
            } else {
                c.arriving=sample%4==1; c.doors_open=!c.arriving && sample%4==2;
            }
            for(int z=0;z<b.floors;++z) if(rng()%4==0) c.car_calls[floor(z)]=1+rng()%3;

            if(sample%11==0 && !c.moving) {
                c.dir=0; c.doors_open=false; c.arriving=false;
                for(auto it=c.car_calls.begin();it!=c.car_calls.end();) {
                    if((it->first-c.committed)*direction<0) it=c.car_calls.erase(it); else ++it;
                }
            }
            auto f=forward_features(b,c,direction);
            if(!f) throw std::runtime_error("expected forward domain");
            std::vector<HallKey> calls,locked,free;
            for(int z=s;z<b.floors-1;++z) {
                HallKey k{floor(z),direction}; calls.push_back(k);
                (z%3==0?locked:free).push_back(k);
            }
            auto correction=f->correction(free,locked);
            std::vector<std::vector<double>> fast(calls.size(),std::vector<double>(calls.size()));
            for(size_t i=0;i<calls.size();++i)
                for(size_t j=i+1;j<calls.size();++j) fast[i][j]=f->pair(calls[i],calls[j]);
            std::vector<double> u;
            for(auto k:calls) u.push_back(unary(b,c,k,raw));
            std::vector<std::vector<double>> ref(calls.size(),std::vector<double>(calls.size()));
            for(size_t i=0;i<calls.size();++i) {
                for(size_t j=i+1;j<calls.size();++j) {
                    ref[i][j]=ref[j][i]=pairwise(b,c,calls[i],calls[j],u[i],u[j]);
                    const double diff=std::abs(fast[i][j]-ref[i][j]);
                    error=std::max(error,diff); ++pairs;
                    if(diff>1e-8) {
                        std::ostringstream why;
                        why<<std::setprecision(17)<<"mismatch sample="<<sample<<" F="<<b.floors
                           <<" s="<<s<<" dir="<<c.dir<<" moving="<<c.moving<<" arriving="<<c.arriving
                           <<" open="<<c.doors_open<<" i="<<i+s<<" j="<<j+s
                           <<" expected="<<ref[i][j]<<" fast="<<fast[i][j]<<" mandatory:";
                        for(auto [z,n]:c.car_calls) why<<' '<<f->orient(z)<<':'<<n;
                        throw std::runtime_error(why.str());
                    }
                }
            }
            for(size_t i=0;i<free.size();++i) {
                double sum=0;
                for(auto k:locked) sum+=ref[f->orient(free[i].floor)-s][f->orient(k.floor)-s];
                sum_error=std::max(sum_error,std::abs(sum-correction[i])); ++rows;
            }
            ++states; moving+=c.moving; arriving+=c.arriving; open+=c.doors_open;
        }
        if(sum_error>1e-7) throw std::runtime_error("row-sum mismatch");
        Building short_moves; short_moves.kin.floor_height=1;
        CarView empty;
        if(forward_features(short_moves,empty,UP)) throw std::runtime_error("triangular regime accepted");
        empty.dir=DOWN;
        if(forward_features(Building{},empty,UP)) throw std::runtime_error("wrong direction accepted");
        empty.dir=0; empty.committed=2; empty.car_calls[0]=1;
        if(forward_features(Building{},empty,UP)) throw std::runtime_error("idle opposite target accepted");
        empty=CarView{};
        auto valid=forward_features(Building{},empty,UP);
        for(int bad=0;bad<3;++bad) {
            bool rejected=false;
            try {
                if(bad==0) valid->pair({1,UP},{1,UP});
                if(bad==1) valid->correction({{1,UP}},{{1,UP}});
                if(bad==2) valid->correction({{1,DOWN}},{{2,UP}});
            } catch(const std::invalid_argument&) {rejected=true;}
            if(!rejected) throw std::runtime_error("invalid pair/row domain accepted");
        }
        std::cout<<std::setprecision(17)<<"{\"synthetic_states\":"<<states<<",\"pairs\":"<<pairs
                 <<",\"lock_rows\":"<<rows<<",\"moving\":"<<moving<<",\"arriving\":"<<arriving
                 <<",\"open\":"<<open<<",\"maximum_pair_error\":"<<error
                 <<",\"maximum_lock_sum_error\":"<<sum_error<<"}\n";
    } catch(const std::exception& e) { std::cerr<<e.what()<<'\n';return 2; }
}
