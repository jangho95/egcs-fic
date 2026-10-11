#pragma once
#include <map>
#include <vector>

#include "model.hpp"

namespace egcs2017 {

struct VCall {
    HallKey key;
    int dest = -1;
    int passengers = 1;
    int id = 0;
};

struct Move {
    enum Kind { LEG, TAIL_OPP, TAIL_BEHIND, HALT } kind = HALT;
    int target = -1;
    int dir = 0;
};

Move next_move(int pos, int dir, const std::map<int, int>& car_calls, const std::vector<HallKey>& pending);

int initial_direction(int pos, const std::map<int, int>& car_calls, const std::vector<HallKey>& pending);

struct PathResult {
    std::vector<double> pickup;
    std::vector<int> order;
    double end_time = 0.0;
    std::map<int, double> alight;
};

PathResult run_path(const Building& b, const CarView& car, std::vector<VCall> calls, int stop_after = 0);

PathResult run_path_full(const Building& b, const CarView& car, std::vector<VCall> calls);

}
