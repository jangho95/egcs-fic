#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>
namespace egcs2017 {
struct PartitionCost { double journey=0, waiting=0; bool valid=false; };
struct PartitionResult {
    bool complete=true, feasible=false;
    long transitions=0;
    double journey=0, waiting=0;
    std::vector<int> masks;
};
inline PartitionResult partition_pareto(const std::vector<std::vector<PartitionCost>>& cost,
                                        int n,double budget,long limit=200000) {
    struct Label {double w,j;std::vector<int> masks;};
    PartitionResult answer;
    int size=1<<n,all=size-1;
    std::vector<std::vector<Label>> states(size);states[0].push_back({0,0,{}});
    for(const auto& car:cost) {
        if(int(car.size())!=size)throw std::invalid_argument("subset table size mismatch");
        std::vector<std::vector<Label>> next(size);
        for(int used=0;used<size;++used)if(!states[used].empty()) {
            int remaining=all^used;
            for(int sub=remaining;;sub=(sub-1)&remaining) {
                const auto& v=car[sub];
                if(v.valid)for(const auto& old:states[used]) {
                    if(++answer.transitions>limit) {answer.complete=false;return answer;}
                    double w=old.w+v.waiting,j=old.j+v.journey;
                    if(w<=budget) {auto masks=old.masks;masks.push_back(sub);next[used|sub].push_back({w,j,std::move(masks)});}
                }
                if(sub==0)break;
            }
        }
        for(auto& labels:next) {
            std::sort(labels.begin(),labels.end(),[](const Label& a,const Label& b){
                if(a.w!=b.w)return a.w<b.w;if(a.j!=b.j)return a.j<b.j;return a.masks<b.masks;
            });
            double best=std::numeric_limits<double>::infinity();size_t keep=0;
            for(size_t i=0;i<labels.size();++i)if(labels[i].j<best){best=labels[i].j;if(keep!=i)labels[keep]=std::move(labels[i]);++keep;}
            labels.resize(keep);
        }
        states=std::move(next);
    }
    for(const auto& v:states[all])if(!answer.feasible||v.j<answer.journey) {
        answer.feasible=true;answer.journey=v.j;answer.waiting=v.w;answer.masks=v.masks;
    }
    return answer;
}
}
