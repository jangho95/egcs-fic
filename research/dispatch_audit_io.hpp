#pragma once
#include "structured_dispatch.hpp"
#include <iostream>
namespace egcs2017::research::audit {
struct Input {
    Building b;
    std::vector<CarView> cars;
    std::vector<CallView> calls;
    HallKey nc;
    std::vector<std::vector<double>> u,r;
    std::vector<std::vector<std::vector<double>>> w;
};

bool read(Input& s) {
    int C,n;
    if(!(std::cin>>s.b.floors>>C>>n)) return false;
    s.cars.resize(C);
    for(auto& c:s.cars) {
        int count;
        std::cin>>c.id>>c.x>>c.v>>c.dir>>c.moving>>c.arriving>>c.committed
                >>c.release>>c.doors_open>>c.load>>count;
        for(int j=0;j<count;++j) {int f,q;std::cin>>f>>q;c.car_calls[f]=q;}
    }
    s.calls.resize(n);
    for(auto& c:s.calls) std::cin>>c.key.floor>>c.key.dir;
    for(int i=0;i<C;++i) {
        int count;std::cin>>count;
        for(int j=0;j<count;++j) {
            CallView c;std::cin>>c.key.floor>>c.key.dir;
            c.close_to_service=true;c.assigned=s.cars[i].id;c.registered=-1;
            s.calls.push_back(c);
        }
    }
    s.u.assign(n,std::vector<double>(C));s.r=s.u;
    for(auto& row:s.u) for(auto& x:row) std::cin>>x;
    for(auto& row:s.r) for(auto& x:row) std::cin>>x;
    s.w.assign(n,std::vector<std::vector<double>>(n,std::vector<double>(C)));
    for(auto& row:s.w) for(auto& col:row) for(auto& x:col) std::cin>>x;
    std::cin>>s.nc.floor>>s.nc.dir;
    for(int i=0;i<n;++i) std::cin>>s.calls[i].registered>>s.calls[i].assigned>>s.calls[i].waiting;
    if(!std::cin) throw std::runtime_error("truncated saved dispatch input");
    return true;
}

double matrix_error(const Matrices& x,const Matrices& y) {
    double e=0;
    if(x.N.size()!=y.N.size()) throw std::runtime_error("call count changed");
    auto check=[&](double a,double b) {
        if(!std::isfinite(a)||!std::isfinite(b)) throw std::runtime_error("nonfinite cost");
        e=std::max(e,std::abs(a-b));
    };
    for(size_t i=0;i<x.N.size();++i) {
        if(!(x.N[i].key==y.N[i].key)) throw std::runtime_error("call order changed");
        for(size_t c=0;c<x.u[i].size();++c) {
            check(x.u[i][c],y.u[i][c]);check(x.u_raw[i][c],y.u_raw[i][c]);
            check(x.u_lk[i][c],y.u_lk[i][c]);check(x.adj[i][c],y.adj[i][c]);
            for(size_t j=0;j<x.N.size();++j) check(x.w[i][j][c],y.w[i][j][c]);
        }
    }
    return e;
}

}
