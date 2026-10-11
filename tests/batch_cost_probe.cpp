#include <iostream>
#include <iomanip>
#include "batch_choice.hpp"
using namespace egcs2017;
int main(){ BatchGeometry g;double mu,nu;int m,n;
while(std::cin>>g.C>>g.p>>g.f>>g.h>>g.tA>>g.tA0>>g.tBf>>g.tBh>>g.f0>>g.h0>>g.fh>>g.d0>>g.tau>>m>>n>>mu>>nu)
std::cout<<std::setprecision(17)<<batch_difference(g,m,n,false)<<' '<<batch_difference(g,m,n,true)<<' '<<expected_batch_difference(g,mu,nu,false)<<' '<<expected_batch_difference(g,mu,nu,true)<<'\n';}
