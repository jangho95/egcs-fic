#include "path.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace egcs2017 {

Move next_move(int pos, int dir, const std::map<int, int>& car_calls, const std::vector<HallKey>& pending) {
    if (car_calls.empty() && pending.empty()) return Move{Move::HALT, -1, 0};
    if (dir == 0) dir = initial_direction(pos, car_calls, pending);
    auto ahead = [&](int f) { return (f - pos) * dir; };
    int best = -1, best_d = std::numeric_limits<int>::max();
    for (auto& [f, n] : car_calls)
        if (ahead(f) >= 0 && ahead(f) < best_d) { best = f; best_d = ahead(f); }
    for (auto& k : pending)
        if (k.dir == dir && ahead(k.floor) >= 0 && ahead(k.floor) < best_d) { best = k.floor; best_d = ahead(k.floor); }
    if (best >= 0) return Move{Move::LEG, best, dir};
    int far = -1, far_d = -1;
    for (auto& k : pending)
        if (k.dir == -dir && ahead(k.floor) >= 0 && ahead(k.floor) > far_d) { far = k.floor; far_d = ahead(k.floor); }
    if (far >= 0) return Move{Move::TAIL_OPP, far, dir};
    return Move{Move::TAIL_BEHIND, -1, -dir};
}

int initial_direction(int pos, const std::map<int, int>& car_calls, const std::vector<HallKey>& pending) {
    int target = -1, best = std::numeric_limits<int>::max();
    auto consider = [&](int f) {
        int d = std::abs(f - pos);
        if (d < best || (d == best && f < target)) { best = d; target = f; }
    };
    for (auto& [f, n] : car_calls) consider(f);
    for (auto& k : pending) consider(k.floor);
    if (target < 0) return 0;
    if (target == pos) {
        int d = DOWN;
        for (auto& k : pending)
            if (k.floor == pos && k.dir == UP) d = UP;
        return d;
    }
    return target > pos ? UP : DOWN;
}

namespace {

struct PathState {
    explicit PathState(const Building& bb) : b(bb) {}
    const Building& b;
    double t = 0.0;
    int pos = 0;
    int dir = 0;
    std::map<int, int> car_calls;
    std::vector<VCall> pending;
    PathResult out;
    int picked = 0;

    std::vector<HallKey> keys() const {
        std::vector<HallKey> k;
        k.reserve(pending.size());
        for (auto& c : pending) k.push_back(c.key);
        return k;
    }
    bool anything_ahead(int d) const {
        for (auto& [f, n] : car_calls)
            if ((f - pos) * d > 0) return true;
        for (auto& c : pending)
            if ((c.key.floor - pos) * d > 0) return true;
        return false;
    }
    int board_call(int d, double arrival) {
        for (size_t i = 0; i < pending.size(); ++i) {
            if (pending[i].key.floor == pos && pending[i].key.dir == d) {
                VCall c = pending[i];
                pending.erase(pending.begin() + i);
                out.pickup[c.id] = arrival;
                out.order.push_back(c.id);
                ++picked;
                if (c.dest >= 0) car_calls[c.dest] += c.passengers;
                return c.passengers;
            }
        }
        return 0;
    }
    void stop() {
        double arrival = t;
        int alight = 0;
        if (auto it = car_calls.find(pos); it != car_calls.end()) { alight = it->second; car_calls.erase(it); out.alight[pos] = arrival; }
        int board = board_call(dir, arrival);
        if (!anything_ahead(dir)) {
            bool opp = false;
            for (auto& c : pending)
                if (c.key.floor == pos && c.key.dir == -dir) opp = true;
            if (opp) { dir = -dir; board += board_call(dir, arrival); }
        }
        t += b.doors.dwell(board, alight);
    }
};

}

static PathResult run_path_impl(const Building& b, const CarView& car, std::vector<VCall> calls, int stop_after, bool full);

PathResult run_path(const Building& b, const CarView& car, std::vector<VCall> calls, int stop_after) {
    return run_path_impl(b, car, std::move(calls), stop_after, false);
}

PathResult run_path_full(const Building& b, const CarView& car, std::vector<VCall> calls) {
    return run_path_impl(b, car, std::move(calls), 0, true);
}

static PathResult run_path_impl(const Building& b, const CarView& car, std::vector<VCall> calls, int stop_after, bool full) {
    int n = 0;
    for (auto& c : calls) n = std::max(n, c.id + 1);
    PathState s{b};
    s.out.pickup.assign(n, -1.0);
    s.car_calls = car.car_calls;
    s.pending = std::move(calls);
    s.dir = car.dir;
    s.pos = car.committed;
    int want = stop_after > 0 ? stop_after : static_cast<int>(s.pending.size());
    const auto& kin = b.kin;

    if (!car.moving) {

        s.t = car.release;
        if (car.arriving) {
            s.stop();
        } else if (car.doors_open && car.dir != 0) {
            for (size_t i = 0; i < s.pending.size(); ++i)
                if (s.pending[i].key.floor == s.pos && s.pending[i].key.dir == car.dir) {
                    VCall c = s.pending[i];
                    s.pending.erase(s.pending.begin() + i);
                    s.out.pickup[c.id] = 0.0;
                    s.out.order.push_back(c.id);
                    ++s.picked;
                    if (c.dest >= 0) s.car_calls[c.dest] += c.passengers;
                    s.t += b.doors.transfer * c.passengers;
                    break;
                }
        }
        if (s.dir == 0) s.dir = initial_direction(s.pos, s.car_calls, s.keys());
    } else {

        Move m = next_move(s.pos, s.dir, s.car_calls, s.keys());
        auto dist_to = [&](int f) { return std::abs(b.pos(f) - car.x); };
        if (m.kind == Move::LEG || m.kind == Move::TAIL_OPP) {
            s.t += kin.time_from_state(dist_to(m.target), car.v);
            s.pos = m.target;
            if (m.kind == Move::TAIL_OPP) s.dir = -s.dir;
            s.stop();
        } else {

            s.t += kin.time_from_state(dist_to(s.pos), car.v);
        }
    }

    int guard = 4 * b.floors + 4 * static_cast<int>(s.pending.size()) + 8;
    while (s.picked < want || (full && !s.car_calls.empty())) {
        if (--guard < 0) throw std::runtime_error("path did not terminate");
        Move m = next_move(s.pos, s.dir, s.car_calls, s.keys());
        if (m.kind == Move::HALT) break;
        if (m.kind == Move::TAIL_BEHIND) { s.dir = m.dir; continue; }
        s.t += kin.travel(s.pos, m.target);
        s.pos = m.target;
        if (m.kind == Move::TAIL_OPP) s.dir = -s.dir;
        s.stop();
    }
    s.out.end_time = s.t;
    return s.out;
}

}
