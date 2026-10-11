#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include "sim.hpp"

using namespace egcs2017;

int main() {
    try {
        std::string line;
        int index = 0;
        while (std::getline(std::cin, line)) {
            if (line.empty()) continue;
            std::istringstream input(line);
            int f, C, variant, p, m;
            if (!(input >> f >> C >> variant >> p >> m) || f < 1 || C < 1 || p < 0 || p >= C || m < 1)
                throw std::runtime_error("invalid probe header");
            Building b;
            b.floors = f+2; b.capacity = C;
            if (variant == 1) {
                b.kin.v_max = 1.1; b.kin.a_max = 0.8; b.kin.floor_height = 3.6; b.kin.settle = 0.8;
                b.doors.open = 1.3; b.doors.close = 2.7; b.doors.transfer = 0.7;
            } else if (variant == 2) {
                b.kin.v_max = 4.0; b.kin.a_max = 0.5; b.kin.floor_height = 2.8; b.kin.settle = 0.2;
                b.doors.open = 0.7; b.doors.close = 1.1; b.doors.transfer = 0.3;
            } else if (variant != 0) throw std::runtime_error("unknown kinematics variant");
            std::vector<int> dest(p+m);
            for (int& d : dest)
                if (!(input >> d) || d < 0 || d >= f) throw std::runtime_error("invalid destination");
            std::string surplus;
            if (input >> surplus) throw std::runtime_error("extra input");
            const int top = f+1;

            const double branch = p ? b.kin.travel(0, top)+b.doors.dwell(p, 0)+1.0 : 0.0;
            std::vector<Passenger> people;
            for (int i=0; i<p+m; ++i) {
                Passenger x;
                x.origin = i<p ? top : f; x.dest = dest[i];
                x.arrival = i<p ? 0.0 : branch;
                people.push_back(x);
            }
            SimOptions options;
            options.theta = 0;
            options.fixed_assignment = true;
            options.assigned_only_boarding = true;
            options.full_load_bypass = true;
            options.check_invariants = true;
            options.forced_assignment[{top, DOWN}] = 0;
            options.forced_assignment[{f, DOWN}] = 0;
            Simulation s(b, 1, make_dispatcher("paper2017_ls_lk"), people, nullptr, branch, options);
            s.run();
            if (s.cars()[0].load() != p || s.waiting().at({f, DOWN}).size() != static_cast<size_t>(m))
                throw std::runtime_error("branch cohort mismatch");
            CarView v = s.view(s.cars()[0], branch);
            if (!v.moving || std::abs(b.pos(f)-v.x)+1e-9 < v.v*v.v/(2*b.kin.a_max))
                throw std::runtime_error("infeasible first pickup geometry");
            double arrival = b.kin.time_from_state(std::abs(b.pos(f)-v.x), v.v);
            double direct_arrival=0, direct_journey=0;
            if (p) {
                int highest = -1;
                for (int i=0; i<p; ++i) highest = std::max(highest, s.passengers()[i].dest);
                direct_arrival = b.kin.time_from_state(std::abs(b.pos(highest)-v.x), v.v);

                Simulation direct(s, make_dispatcher("paper2017_ls_lk"));
                direct.set_max_time(1e7);
                SimOptions::Intervention iv;
                iv.mode = SimOptions::Intervention::LATER;
                iv.call = {f, DOWN}; iv.car = 0;
                direct.apply_intervention(iv);
                direct.run();
                for (int i=0; i<p; ++i) {
                    if (direct.passengers()[i].alighted < 0) throw std::runtime_error("direct cohort unserved");
                    direct_journey += direct.passengers()[i].alighted-branch;
                }
            }
            s.set_max_time(1e7);
            s.run();
            double W=0, J=0;
            std::cout << std::setprecision(17) << "{\"index\":" << index++
                      << ",\"arrival\":" << arrival << ",\"door\":" << b.doors.open+b.doors.close
                      << ",\"direct_arrival\":" << direct_arrival << ",\"direct_journey\":" << direct_journey
                      << ",\"transfer\":" << b.doors.transfer << ",\"travel\":[";
            for (int i=0; i<=f; ++i) {
                if (i) std::cout << ',';
                std::cout << '[';
                for (int j=0; j<=f; ++j) { if (j) std::cout << ','; std::cout << b.kin.travel(i,j); }
                std::cout << ']';
            }
            std::cout << "],\"old_destinations\":[";
            for (int i=0; i<p; ++i) { if (i) std::cout << ','; std::cout << s.passengers()[i].dest; }
            std::cout << "],\"queue_destinations\":[";
            for (int i=p; i<p+m; ++i) { if (i>p) std::cout << ','; std::cout << s.passengers()[i].dest; }
            std::cout << "],\"passengers\":[";
            for (int i=0; i<p+m; ++i) {
                const auto& x = s.passengers()[i];
                if (x.boarded < 0 || x.alighted < 0 || x.car != 0)
                    throw std::runtime_error("unserved or wrong car");
                if (i>=p) W += x.boarded-branch;
                J += x.alighted-branch;
                if (i) std::cout << ',';
                std::cout << '[' << x.boarded-branch << ',' << x.alighted-branch << ']';
            }
            if (!s.waiting().empty() || !s.calls().empty()) throw std::runtime_error("nonempty drain");
            std::cout << "],\"waiting\":" << W << ",\"journey\":" << J << "}\n";
        }
    } catch (const std::exception& e) {
        std::cerr << "mixed drain probe failed: " << e.what() << '\n';
        return 2;
    }
}
