#pragma once
#include "path.hpp"

namespace egcs2017 {

struct CostOptions {
    double onboard_stop_weight = 0.0;
    bool pairwise = true;
    bool bonus = false;
    bool capacity_penalty = false;

    int capacity_margin = 2;
    double capacity_cost = 1000.0;
    int order_k = 0;
    double order_penalty = 0.0;

    enum Search { NONE, LS, OPT } search = NONE;
    int ls_max_iter = 200;
    int opt_max_calls = 7;
    long opt_max_states = 20000;

    bool locked_aware = false;
    int reeval_candidates = 0;

    int reeval_samples = 16;

    bool reeval_legacy_score = false;
};

double unary(const Building& b, const CarView& car, const HallKey& i, const CostOptions& opt);

double pairwise(const Building& b, const CarView& car, const HallKey& i, const HallKey& j, double w_i, double w_j);

std::vector<int> destinations(const Building& b, const HallKey& k);

}
