#pragma once
#include <cfenv>
#include <chrono>
#include <limits>
#include "structured_dispatch.hpp"

#ifdef __FAST_MATH__
#error "structured_lk_v1 requires compilation without fast-math"
#endif

namespace egcs2017::research {
class StructuredDispatcher final : public Dispatcher {
public:
    StructuredDispatcher() {
        static_assert(std::numeric_limits<double>::is_iec559 &&
                      std::numeric_limits<double>::digits == 53);
        if(std::fegetround()!=FE_TONEAREST)
            throw std::invalid_argument("structured_lk_v1 requires round-to-nearest");
    }
    std::string name() const override {return "structured_lk_v1";}
    std::map<HallKey,int> decide(const Building& b,const std::vector<CarView>& cars,
            const std::vector<CallView>& calls,const HallKey& nc,DecisionTrace* trace) override {
        if(std::fegetround()!=FE_TONEAREST)
            throw std::invalid_argument("rounding mode changed during structured dispatch");
        const auto start=std::chrono::steady_clock::now();
        const auto result=structured_decide(b,cars,calls,nc,PairKernel::Combined);
        if(trace) {

            const double time=trace->t;*trace=DecisionTrace{};trace->t=time;
            trace->method=name();trace->new_call=nc;
            const auto& M=result.matrices;
            for(const auto& car:cars)trace->cars.push_back(car.id);
            for(const auto& call:M.N) {
                trace->calls.push_back(call.key);trace->before[call.key]=call.assigned;
                trace->after[call.key]=result.after.at(call.key);
            }
            trace->unary=M.u;trace->unary_lk=M.u_lk;trace->pair=M.w;
            for(const auto& locked:M.locked)trace->n_locked+=static_cast<int>(locked.size());
            trace->g_greedy=result.greedy_cost;trace->g_final=result.final_cost;
            trace->search_mode=M.N.empty()?0:1;trace->search_iterations=result.iterations;
            trace->search_cap_hit=result.iterations>=locked_ls_options().ls_max_iter;
            for(size_t i=0;i<result.final.size();++i)
                trace->search_changed+=result.final[i]!=result.greedy[i];

            trace->micros=std::chrono::duration<double,std::micro>(
                std::chrono::steady_clock::now()-start).count();
        }
        return result.after;
    }
};
}
