#pragma once
#include <deque>
#include <map>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "dispatchers.hpp"

namespace egcs2017 {

struct GaOptions {
    int pop = 10, gens = 10;
    double pc = 0.8, pm = 0.1;
    bool seeding = true;
    bool last_best = true;
    double pen_threshold = 60.0;
    double pen_weight = 1.0;
    bool estimate = false;
    double origin_window = 50.0;
    unsigned long seed = 1;

    enum Route { FIG4, COLLECTIVE } route = FIG4;

    enum Ewt { ACCUMULATE, LITERAL } ewt = ACCUMULATE;

};

struct GaCall {
    HallKey key;
    int estimation = 1;
    double waited = 0.0;
};

struct GaStop {
    int floor = 0;
    int board = 0, alight = 0;
    double t_arrive = 0.0, t_complete = 0.0;
};

struct GaRouteResult {
    std::vector<GaStop> stops;
    std::vector<double> first_pickup;
    std::vector<double> contribution;
    double ewt = 0.0;
    double end_time = 0.0;
    int unloading_events = 0;
};

GaRouteResult ga_route(const Building& b, const CarView& car, const std::vector<GaCall>& calls, const GaOptions& opt);

class Ga2022Dispatcher : public Dispatcher {
public:
    Ga2022Dispatcher(GaOptions o, std::string label) : opt_(o), label_(std::move(label)) {}
    std::string name() const override { return label_; }
    std::map<HallKey, int> decide(const Building&, const std::vector<CarView>&, const std::vector<CallView>&,
                                  const HallKey&, DecisionTrace*) override;
    void observe_boarding(double t, int floor, int dir, int n) override;
    const GaOptions& options() const { return opt_; }

    static double fitness(const Building& b, const std::vector<CarView>& cars, const std::vector<CallView>& calls,
                          const std::vector<int>& genes, const std::vector<int>& estimation, const GaOptions& opt);

    static double lift_cost(const Building& b, const CarView& car, const std::vector<GaCall>& calls, const GaOptions& opt);

    int estimation_for(const HallKey& k, double now) const;

private:
    GaOptions opt_;
    std::string label_;

    std::map<std::pair<HallKey, double>, int> last_best_;
    long decisions_ = 0;
    struct Boarding { double t; int floor, dir, n; };
    std::deque<Boarding> boardings_;
};

}
