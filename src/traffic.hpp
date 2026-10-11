#pragma once
#include <algorithm>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include "sim.hpp"

namespace egcs2017 {

inline std::vector<Passenger> interfloor_traffic(int floors, double rate_pct, double population_per_floor,
                                                 double duration, unsigned long seed) {
    std::mt19937_64 rng(seed);
    double population = population_per_floor * (floors - 1);
    double lambda = rate_pct / 100.0 * population / 300.0;
    std::exponential_distribution<double> gap(lambda);
    std::uniform_int_distribution<int> floor(1, floors - 1);
    std::vector<Passenger> out;
    double t = gap(rng);
    while (t < duration) {
        Passenger p;
        p.arrival = t;
        p.origin = floor(rng);
        do { p.dest = floor(rng); } while (p.dest == p.origin);
        out.push_back(p);
        t += gap(rng);
    }
    return out;
}

struct TrafficPhase {
    double t0 = 0.0, t1 = 3600.0;
    double rate_pct = 20.0;
    double p_in = 0.0, p_out = 0.0, p_inter = 1.0;
};

struct TrafficSpec {
    int floors = 10;
    double population_per_floor = 30.0;
    std::vector<TrafficPhase> phases;
    double population() const { return population_per_floor * (floors - 1); }
    double rate_at(double t) const {
        for (auto& ph : phases) if (t >= ph.t0 && t < ph.t1) return ph.rate_pct / 100.0 * population() / 300.0;
        return 0.0;
    }
    const TrafficPhase* phase_at(double t) const {
        for (auto& ph : phases) if (t >= ph.t0 && t < ph.t1) return &ph;
        return nullptr;
    }
    double end() const { double e = 0.0; for (auto& ph : phases) e = std::max(e, ph.t1); return e; }

    static TrafficSpec named(const std::string& name, int floors, double rate_pct, double population_per_floor, double duration) {
        TrafficSpec s; s.floors = floors; s.population_per_floor = population_per_floor;
        if (name == "interfloor") s.phases.push_back(TrafficPhase{0.0, duration, rate_pct, 0.0, 0.0, 1.0});
        else if (name == "up") s.phases.push_back(TrafficPhase{0.0, duration, rate_pct, 0.85, 0.10, 0.05});
        else if (name == "downt") {
            s.phases.push_back(TrafficPhase{0.0, 900.0, 5.0, 0.0, 0.0, 1.0});
            s.phases.push_back(TrafficPhase{900.0, 2700.0, rate_pct, 0.10, 0.85, 0.05});
        } else throw std::invalid_argument("unknown traffic profile " + name);
        return s;
    }
};

inline Passenger draw_passenger(std::mt19937_64& rng, const TrafficPhase& ph, int floors, double t) {
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    std::uniform_int_distribution<int> upper(1, floors - 1);
    Passenger p; p.arrival = t;
    double u = u01(rng);
    if (u < ph.p_in) { p.origin = 0; p.dest = upper(rng); }
    else if (u < ph.p_in + ph.p_out) { p.origin = upper(rng); p.dest = 0; }
    else { p.origin = upper(rng); do { p.dest = upper(rng); } while (p.dest == p.origin); }
    return p;
}

inline std::vector<Passenger> profile_arrivals(const TrafficSpec& spec, std::mt19937_64& rng, double t0, double t1) {
    std::vector<Passenger> out;
    for (auto& ph : spec.phases) {
        double a = std::max(t0, ph.t0), b = std::min(t1, ph.t1);
        if (b <= a) continue;
        double lambda = ph.rate_pct / 100.0 * spec.population() / 300.0;
        if (lambda <= 0) continue;
        std::exponential_distribution<double> gap(lambda);
        double t = a + gap(rng);
        while (t < b) { out.push_back(draw_passenger(rng, ph, spec.floors, t)); t += gap(rng); }
    }
    std::sort(out.begin(), out.end(), [](const Passenger& x, const Passenger& y) { return x.arrival < y.arrival; });
    return out;
}

inline std::vector<Passenger> profile_traffic(const TrafficSpec& spec, unsigned long seed) {
    std::mt19937_64 rng(seed);
    return profile_arrivals(spec, rng, 0.0, spec.end());
}

}
