#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include "cost.hpp"
using namespace egcs2017;

int main() {
    try {
        Building b;
        b.floors = 3;
        CostOptions raw;
        HallKey free{1, UP}, locked{0, UP};
        double max_error = 0;
        int paths = 0;
        std::cout << std::setprecision(17) << "{\"records\":[";
        bool first = true;
        for (double eps : {1.0, .1, .001, .000001}) {
            CarView a, other;
            a.id = 0; a.x = b.pos(0); a.committed = 0; a.dir = DOWN; a.release = 2;
            other.id = 1; other.x = b.pos(2); other.committed = 2;
            other.dir = UP; other.release = 2 + eps;

            if (a.release > b.doors.close || other.release > b.doors.close)
                throw std::runtime_error("invalid door-closing state");
            double travel = b.kin.travel(0, 1), dwell = b.doors.dwell(1, 0);
            double ua = unary(b, a, free, raw), ub = unary(b, other, free, raw);
            double ul = unary(b, a, locked, raw);
            double r = pairwise(b, a, free, locked, ua, ul);
            double loss = ua + r - ub;
            for (auto [got, expected] : {
                std::pair{ua, 2 + travel}, {ub, 2 + eps + travel},
                {ul, 2.0}, {r, dwell}, {loss, dwell - eps}})
                max_error = std::max(max_error, std::abs(got - expected));
            if (!(ul <= 5 && ua < ub && ua + r > ub))
                throw std::runtime_error("lock or strict choice failed");

            for (int d : destinations(b, locked)) {
                auto keep = run_path(b, a, {VCall{locked,d,1,0}, VCall{free,2,1,1}}, 2);
                auto a_only = run_path(b, a, {VCall{locked,d,1,0}}, 1);
                auto b_only = run_path(b, other, {VCall{free,2,1,0}}, 1);
                double path_loss = keep.pickup[0] + keep.pickup[1]
                                   - a_only.pickup[0] - b_only.pickup[0];
                max_error = std::max(max_error, std::abs(path_loss - loss));
                ++paths;
            }
            if (!first) std::cout << ',';
            first = false;
            std::cout << "{\"epsilon\":" << eps << ",\"u_A\":" << ua
                      << ",\"u_B\":" << ub << ",\"lock_eta\":" << ul
                      << ",\"omega\":" << r << ",\"loss\":" << loss
                      << ",\"loss_over_omega\":" << loss/r << '}';
        }
        if (max_error > 1e-10) throw std::runtime_error("identity mismatch");
        std::cout << "],\"paths\":" << paths << ",\"maximum_error\":" << max_error << "}\n";
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 2;
    }
}
