#include "cost.hpp"

#include <algorithm>

namespace egcs2017 {

std::vector<int> destinations(const Building& b, const HallKey& k) {
    std::vector<int> out;
    if (k.dir == UP)
        for (int f = k.floor + 1; f < b.floors; ++f) out.push_back(f);
    else
        for (int f = k.floor - 1; f >= 0; --f) out.push_back(f);
    return out;
}

double unary(const Building& b, const CarView& car, const HallKey& i, const CostOptions& opt) {
    PathResult r = run_path(b, car, {VCall{i, -1, 1, 0}}, 1);
    double w = r.pickup[0];
    if (w < 0) w = 1e9;
    if (opt.bonus && car.car_calls.count(i.floor)) w -= std::min(0.20 * w, 10.0);
    if (opt.capacity_penalty && car.load >= std::max(1, b.capacity - opt.capacity_margin)) w += opt.capacity_cost;
    return w;
}

double pairwise(const Building& b, const CarView& car, const HallKey& i, const HallKey& j, double w_i, double w_j) {

    PathResult first = run_path(b, car, {VCall{i, -1, 1, 0}, VCall{j, -1, 1, 1}}, 1);
    int first_id = first.order.empty() ? 0 : first.order.front();
    const HallKey& fk = first_id == 0 ? i : j;
    std::vector<int> dests = destinations(b, fk);
    if (dests.empty()) return 0.0;
    double total = 0.0;
    for (int f : dests) {
        std::vector<VCall> calls{VCall{i, -1, 1, 0}, VCall{j, -1, 1, 1}};
        calls[first_id].dest = f;
        PathResult r = run_path(b, car, calls, 2);
        double a = r.pickup[0] >= 0 ? r.pickup[0] : 1e9;
        double c = r.pickup[1] >= 0 ? r.pickup[1] : 1e9;
        total += a + c;
    }
    return total / static_cast<double>(dests.size()) - w_i - w_j;
}

}
