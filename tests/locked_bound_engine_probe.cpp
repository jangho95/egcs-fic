#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include "sim.hpp"
using namespace egcs2017;

namespace {
struct Result { double wait = 0, error = 0, free_pickup = 0; int free_car = -1; };
Result run(const std::string& policy, double delta, int destination,
           double theta = 5, double residual_A = 2) {
    Building b; b.floors = 3;
    const double travel = b.kin.travel(0, 2), dwell = b.doors.dwell(1, 0);
    const double drop_A = 2 * travel + dwell, branch = drop_A + (dwell - residual_A);
    const double locked_arrival = branch - .5;
    std::vector<Passenger> people;
    auto add = [&](double t, int from, int to) {
        Passenger p; p.arrival = t; p.origin = from; p.dest = to; people.push_back(p);
    };
    add(0, 2, 0);
    add(travel + delta, 0, 2);
    add(locked_arrival, 0, destination);
    add(branch, 1, 2);
    Result result;
    int captured = 0;
    SimOptions options; options.theta = theta; options.check_invariants = true;
    options.on_decision = [&](double t, int, const Building& building,
                             const std::vector<CarView>& cars,
                             const std::vector<CallView>& calls, const HallKey& nc) {
        if (!(nc == HallKey{1, UP})) return;
        ++captured;
        auto close = [&](double x, double expected) {
            result.error = std::max(result.error, std::abs(x - expected));
        };
        close(t, branch);
        if (cars.size() != 2 || calls.size() != 2) throw std::runtime_error("wrong state size");
        for (int c = 0; c < 2; ++c) {
            const auto& car = cars[c];
            if (car.moving || car.arriving || car.doors_open || car.load || !car.car_calls.empty())
                throw std::runtime_error("not an empty closing-door car");
            close(car.x, building.pos(c ? 2 : 0));
            close(car.release, residual_A + (c ? delta : 0));
            if (car.dir != (c ? UP : DOWN)) throw std::runtime_error("wrong service direction");
        }
        int locked = 0, free = 0;
        for (const auto& call : calls) {
            if (call.key == HallKey{0, UP}) {
                if (!call.close_to_service || call.assigned != 0)
                    throw std::runtime_error("call not naturally theta-locked to A");
                ++locked;
            } else if (call.key == nc && !call.close_to_service && call.assigned == -1) ++free;
        }
        if (locked != 1 || free != 1) throw std::runtime_error("wrong lock partition");
        CostOptions raw;
        const double ua = unary(building, cars[0], nc, raw);
        const double ub = unary(building, cars[1], nc, raw);
        const double ul = unary(building, cars[0], HallKey{0, UP}, raw);
        const double cross = pairwise(building, cars[0], nc, HallKey{0, UP}, ua, ul);
        close(ua, residual_A + building.kin.travel(0, 1));
        close(ub, residual_A + delta + building.kin.travel(2, 1));
        close(ul, residual_A); close(cross, dwell);
    };
    Simulation simulation(b, 2, make_dispatcher(policy), people, nullptr, 1000, options);
    simulation.run();
    if (captured != 1) throw std::runtime_error("comparison state not captured exactly once");
    int served = 0;
    for (const auto& p : simulation.passengers()) {
        if (p.boarded < 0 || p.alighted < 0) throw std::runtime_error("unfinished passenger");
        if (p.arrival >= locked_arrival - 1e-9) {
            result.wait += p.boarded - branch;
            ++served;
        }
        if (p.origin == 1) {
            result.free_car = p.car; result.free_pickup = p.boarded - branch;
        }
    }
    if (served != 2 || !simulation.calls().empty() || !simulation.waiting().empty())
        throw std::runtime_error("system not drained");
    return result;
}
}

int main(int argc, char** argv) {
    try {
        if (argc > 2 || (argc == 2 && std::string(argv[1]) != "--theta-sweep"))
            throw std::invalid_argument("usage: locked_bound_engine_probe [--theta-sweep]");
        const bool sweep = argc == 2;
        const std::vector<std::pair<double, double>> settings = sweep
            ? std::vector<std::pair<double, double>>{{5,1},{2,1},{.1,.05},{.001,.0005},{.000001,.0000005},{0,0}}
            : std::vector<std::pair<double, double>>{{5,2}};
        const std::vector<double> deltas = sweep ? std::vector<double>{.1,.000001}
                                                : std::vector<double>{1,.1,.001,.000001};
        std::cout << std::setprecision(17) << "{\"records\":[";
        double max_error = 0; int runs = 0; bool first = true;
        for (auto [theta, residual_A] : settings) {
          for (double delta : deltas) {
            for (int destination : {1, 2}) {
                const auto omitted = run("paper2017_ls", delta, destination, theta, residual_A);
                const auto corrected = run("paper2017_ls_lk", delta, destination, theta, residual_A);
                runs += 2;
                if (omitted.free_car != 0 || corrected.free_car != 1)
                    throw std::runtime_error("strict assignment not realized");
                const double loss = omitted.wait - corrected.wait;
                max_error = std::max({max_error, omitted.error, corrected.error,
                                      std::abs(loss - (6.2 - delta))});
                if (!first) std::cout << ',';
                first = false;
                std::cout << "{\"theta\":" << theta << ",\"residual_A\":" << residual_A
                          << ",\"delta\":" << delta << ",\"locked_destination\":" << destination
                          << ",\"omitted_wait\":" << omitted.wait << ",\"corrected_wait\":" << corrected.wait
                          << ",\"loss\":" << loss << ",\"omega\":6.2,\"ratio\":" << loss/6.2
                          << ",\"omitted_car\":" << omitted.free_car << ",\"corrected_car\":" << corrected.free_car << '}';
            }
          }
        }
        if (max_error > 1e-9) throw std::runtime_error("engine/analytic mismatch");
        std::cout << "],\"runs\":" << runs << ",\"maximum_error\":" << max_error << "}\n";
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 2; }
}
