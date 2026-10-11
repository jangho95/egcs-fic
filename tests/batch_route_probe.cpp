#include <iostream>
#include <iomanip>
#include "batch_choice.hpp"
using namespace egcs2017;
int main(){Building b;int p,mode,n;double t;while(std::cin>>b.capacity>>p>>t>>mode>>n){std::vector<std::pair<int,double>> qs;for(int i=0;i<n;++i){int f;double mean;std::cin>>f>>mean;qs.emplace_back(f,mean);}auto [j,w]=batch_route_value(b,p,t,qs,mode);std::cout<<std::setprecision(17)<<j<<' '<<w<<'\n';}}
