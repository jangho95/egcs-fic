#pragma once
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "kinematics.hpp"

namespace egcs2017 {

constexpr int UP = 1, DOWN = -1;

struct HallKey {
    int floor;
    int dir;
    bool operator<(const HallKey& o) const { return floor != o.floor ? floor < o.floor : dir < o.dir; }
    bool operator==(const HallKey& o) const { return floor == o.floor && dir == o.dir; }
};

struct Doors {
    double open = 2.0;
    double close = 3.0;
    double transfer = 1.2;
    double dwell(int board, int alight) const {
        if (board + alight <= 0) return 0.0;
        return open + close + transfer * (board + alight);
    }
};

struct Building {
    int floors = 12;
    int capacity = 13;
    Kinematics kin;
    Doors doors;
    double pos(int floor) const { return floor * kin.floor_height; }
};

struct CarView {
    int id = 0;
    double x = 0.0;
    double v = 0.0;
    int dir = 0;
    bool moving = false;
    bool arriving = false;
    int committed = 0;
    double release = 0.0;
    bool doors_open = false;
    int load = 0;
    std::map<int, int> car_calls;
    std::vector<HallKey> assigned;
    int floor() const { return committed; }
};

struct CallView {
    HallKey key;
    double registered = 0.0;
    int assigned = -1;
    bool close_to_service = false;
    int waiting = 1;
    double waited = 0.0;
    int forbidden_car = -1;
};

}
