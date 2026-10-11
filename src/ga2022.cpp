#include "ga2022.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>

namespace egcs2017 {

namespace {

struct RouteState {
    const Building& b;
    const GaOptions& opt;
    double t = 0.0;
    int pos = 0, ncd = 0;
    int known = 0, unknown = 0;
    std::map<int, int> cc;
    std::vector<int> rem;
    std::vector<int> just_left;
    const std::vector<GaCall>& calls;
    GaRouteResult out;
    double acc = 0.0;

    RouteState(const Building& bb, const CarView& car, const std::vector<GaCall>& cs, const GaOptions& o) : b(bb), opt(o), calls(cs) {
        rem.resize(cs.size());
        for (size_t i = 0; i < cs.size(); ++i) rem[i] = std::max(1, cs[i].estimation);
        out.first_pickup.assign(cs.size(), -1.0);
        out.contribution.assign(cs.size(), 0.0);
        cc = car.car_calls;
        known = car.load;
        if (cc.empty()) { unknown = known; known = 0; }
    }
    int load() const { return known + unknown; }

    void advance(double dt) { t += dt; seg_since_last_ += dt; }
    bool pending() const { for (int r : rem) if (r > 0) return true; return false; }
    bool ahead_of(int f, int d) const { return (f - pos) * d > 0; }

    int choose_direction() const {
        if (!cc.empty()) {
            if (ncd != 0) return ncd;
            int best = -1, bd = std::numeric_limits<int>::max();
            for (auto& [f, n] : cc) if (std::abs(f - pos) < bd) { bd = std::abs(f - pos); best = f; }
            return best > pos ? UP : (best < pos ? DOWN : (ncd != 0 ? ncd : UP));
        }
        int best = -1, bd = std::numeric_limits<int>::max();
        for (size_t i = 0; i < calls.size(); ++i) {
            if (rem[i] <= 0) continue;
            int d = std::abs(calls[i].key.floor - pos);
            if (d < bd || (d == bd && calls[i].key.floor < best)) { bd = d; best = calls[i].key.floor; }
        }
        if (best < 0) return 0;
        if (best == pos) { for (size_t i = 0; i < calls.size(); ++i) if (rem[i] > 0 && calls[i].key.floor == pos) return calls[i].key.dir; }
        return best > pos ? UP : DOWN;
    }

    int next_stop(int d, bool allow_here) const {
        int best = -1, bd = std::numeric_limits<int>::max();
        auto consider = [&](int f) {
            int dist = (f - pos) * d;
            if (dist < 0 || (dist == 0 && !allow_here)) return;
            if (dist < bd) { bd = dist; best = f; }
        };
        for (auto& [f, n] : cc) consider(f);
        for (size_t i = 0; i < calls.size(); ++i) if (rem[i] > 0) consider(calls[i].key.floor);
        return best;
    }
    bool anything_ahead(int d) const {
        for (auto& [f, n] : cc) if (ahead_of(f, d)) return true;
        for (size_t i = 0; i < calls.size(); ++i) if (rem[i] > 0 && ahead_of(calls[i].key.floor, d)) return true;
        return false;
    }

    bool stop(bool first_leg_open_doors = false) {
        GaStop st; st.floor = pos; st.t_arrive = t;
        if (auto it = cc.find(pos); it != cc.end()) {
            int remaining = static_cast<int>(cc.size());
            st.alight = remaining > 0 ? (known + remaining - 1) / remaining : 0;
            st.alight = std::min(st.alight, known);
            known -= st.alight;
            cc.erase(it);
            if (cc.empty()) { st.alight += known; known = 0; }
        }

        std::vector<int> order;
        for (int pass = 0; pass < 2; ++pass)
            for (size_t i = 0; i < calls.size(); ++i)
                if (rem[i] > 0 && calls[i].key.floor == pos && ((pass == 0) == (calls[i].key.dir == ncd))) order.push_back(static_cast<int>(i));
        std::vector<std::pair<int, int>> boarded;
        for (int i : order) {
            int room = b.capacity - load();
            int n = std::min(rem[i], room);
            if (n <= 0) continue;
            rem[i] -= n; unknown += n; st.board += n;
            boarded.emplace_back(i, n);
        }
        if (st.board + st.alight == 0 && !first_leg_open_doors) return false;

        double seg = (first_leg_open_doors ? 0.0 : b.doors.open) + b.doors.transfer * (st.board + st.alight);
        advance(seg);
        st.t_complete = t;
        if (opt.ewt == GaOptions::LITERAL) {

            double est_here = 0; for (auto& [i, n] : boarded) est_here += n;
            if (est_here > 0) { acc = (acc + seg_since_last_) * est_here; seg_since_last_ = 0.0; }
            for (auto& [i, n] : boarded) { if (out.first_pickup[i] < 0) out.first_pickup[i] = t; out.contribution[i] += acc * (n / est_here); }
        } else {
            for (auto& [i, n] : boarded) { if (out.first_pickup[i] < 0) out.first_pickup[i] = t; out.contribution[i] += t * n; }
        }
        just_left.clear();
        for (auto& [i, n] : boarded) just_left.push_back(i);
        out.stops.push_back(st);
        doors_to_close_ = true;
        return true;
    }

    void run_to(int f) {
        if (doors_to_close_) { advance(b.doors.close); doors_to_close_ = false; }
        advance(b.kin.travel(pos, f));
        pos = f;
    }

    void forced_unloading() {
        if (!doors_to_close_) advance(b.doors.open);
        advance(b.doors.transfer * unknown);
        GaStop st; st.floor = pos; st.t_arrive = t; st.alight = unknown; st.t_complete = t;
        unknown = 0;
        out.stops.push_back(st);
        ++out.unloading_events;
        doors_to_close_ = true;
    }
    bool doors_to_close_ = false;
    double seg_since_last_ = 0.0;
};

}

GaRouteResult ga_route(const Building& b, const CarView& car, const std::vector<GaCall>& calls, const GaOptions& opt) {
    RouteState s(b, car, calls, opt);
    const auto& kin = b.kin;
    s.pos = car.committed;
    s.ncd = car.dir;
    if (!car.moving) {

        s.advance(car.release);
        if (car.arriving) {
            s.stop();
        } else if (car.doors_open && car.dir != 0) {
            bool here = false;
            for (size_t i = 0; i < calls.size(); ++i) if (calls[i].key.floor == s.pos && calls[i].key.dir == car.dir) here = true;
            if (here) {

                s.t = 0.0; s.seg_since_last_ = 0.0;
                s.stop(true);
                s.advance(car.release + (s.out.stops.empty() ? 0.0 : b.doors.transfer * s.out.stops.back().board) - s.t);
                s.doors_to_close_ = false;
            }
        }
    } else {

        int f = s.next_stop(s.ncd, true);
        auto dist_to = [&](int ff) { return std::abs(b.pos(ff) - car.x); };
        if (f >= 0) { s.advance(kin.time_from_state(dist_to(f), car.v)); s.pos = f; s.stop(); }
        else { s.advance(kin.time_from_state(dist_to(s.pos), car.v)); }
    }
    int guard = 8 * b.floors + 8 * static_cast<int>(calls.size()) + 16;
    bool need_dir = true;
    auto pending_here_with_room = [&]() {
        if (b.capacity - s.load() <= 0) return false;
        for (size_t i = 0; i < calls.size(); ++i) if (s.rem[i] > 0 && calls[i].key.floor == s.pos) return true;
        return false;
    };
    while (s.pending() || !s.cc.empty()) {
        if (--guard < 0) {
            std::string msg = "ga_route did not terminate: pos " + std::to_string(s.pos) + " ncd " + std::to_string(s.ncd) + " known " + std::to_string(s.known) +
                              " unknown " + std::to_string(s.unknown) + " cap " + std::to_string(b.capacity) + " cc:";
            for (auto& [f, n] : s.cc) msg += " " + std::to_string(f);
            msg += " calls:";
            for (size_t i = 0; i < calls.size(); ++i) msg += " (" + std::to_string(calls[i].key.floor) + "," + std::to_string(calls[i].key.dir) + ")rem" + std::to_string(s.rem[i]);
            throw std::runtime_error(msg);
        }

        if (s.load() >= b.capacity && s.cc.empty() && s.pending()) { s.forced_unloading(); need_dir = true; continue; }

        if (pending_here_with_room()) {
            if (s.doors_to_close_) { s.advance(b.doors.close); s.doors_to_close_ = false; }
            if (s.ncd == 0) {
                for (size_t i = 0; i < calls.size(); ++i) if (s.rem[i] > 0 && calls[i].key.floor == s.pos) { s.ncd = calls[i].key.dir; break; }
            }
            s.stop();
            need_dir = s.load() == 0;
            continue;
        }

        if (need_dir || s.ncd == 0 || !s.anything_ahead(s.ncd)) {
            int d = s.choose_direction();
            if (d == 0) break;
            s.ncd = d;
            need_dir = false;
        }
        int f = s.next_stop(s.ncd, false);
        if (f < 0) {
            if (s.anything_ahead(-s.ncd)) { s.ncd = -s.ncd; continue; }
            break;
        }
        s.run_to(f);
        s.stop();
        need_dir = s.load() == 0;
    }
    s.out.ewt = 0.0;
    for (double c : s.out.contribution) s.out.ewt += c;
    s.out.end_time = s.t;
    return s.out;
}

double Ga2022Dispatcher::lift_cost(const Building& b, const CarView& car, const std::vector<GaCall>& calls, const GaOptions& opt) {
    if (calls.empty()) return 0.0;
    double f = 0.0;
    if (opt.route == GaOptions::COLLECTIVE) {

        std::vector<VCall> vc;
        for (size_t i = 0; i < calls.size(); ++i) vc.push_back(VCall{calls[i].key, -1, 1, static_cast<int>(i)});
        PathResult r = run_path(b, car, vc, 0);
        for (size_t i = 0; i < calls.size(); ++i) {
            double T = r.pickup[i] >= 0 ? r.pickup[i] : 1e6;
            f += T * calls[i].estimation + opt.pen_weight * std::max(0.0, calls[i].waited + T - opt.pen_threshold);
        }
        return f;
    }
    GaRouteResult r = ga_route(b, car, calls, opt);
    for (size_t i = 0; i < calls.size(); ++i) {
        double T = r.first_pickup[i] >= 0 ? r.first_pickup[i] : 1e6;
        f += (r.first_pickup[i] >= 0 ? r.contribution[i] : 1e6 * calls[i].estimation);
        f += opt.pen_weight * std::max(0.0, calls[i].waited + T - opt.pen_threshold);
    }
    return f;
}

double Ga2022Dispatcher::fitness(const Building& b, const std::vector<CarView>& cars, const std::vector<CallView>& calls,
                                 const std::vector<int>& genes, const std::vector<int>& estimation, const GaOptions& opt) {
    const int n = static_cast<int>(calls.size()), C = static_cast<int>(cars.size());
    double f = 0.0;
    for (int c = 0; c < C; ++c) {
        std::vector<GaCall> gc;
        for (int i = 0; i < n; ++i) if (genes[i] == c) gc.push_back(GaCall{calls[i].key, estimation[i], calls[i].waited});
        if (!gc.empty()) f += lift_cost(b, cars[c], gc, opt);
    }
    return f;
}

void Ga2022Dispatcher::observe_boarding(double t, int floor, int dir, int n) {
    if (!opt_.estimate) return;
    boardings_.push_back(Boarding{t, floor, dir, n});
    double horizon = 3.0 * opt_.origin_window;
    while (!boardings_.empty() && boardings_.front().t < t - horizon - 1.0) boardings_.pop_front();
}

int Ga2022Dispatcher::estimation_for(const HallKey& k, double now) const {
    if (!opt_.estimate) return 1;
    int w[3] = {0, 0, 0};
    for (auto& bd : boardings_) {
        if (bd.floor != k.floor || bd.dir != k.dir) continue;
        double age = now - bd.t;
        if (age < 0) continue;
        int q = static_cast<int>(age / opt_.origin_window);
        if (q < 3) w[q] += bd.n;
    }
    int tendency = 0;
    if (w[0] > w[1] && w[1] > w[2]) tendency = 1;
    else if (w[0] < w[1] && w[1] < w[2]) tendency = -1;
    return std::max(1, w[0] + tendency);
}

std::map<HallKey, int> Ga2022Dispatcher::decide(const Building& b, const std::vector<CarView>& cars,
                                                const std::vector<CallView>& calls_in, const HallKey& nc,
                                                DecisionTrace* trace) {
    auto t0 = std::chrono::steady_clock::now();
    std::map<HallKey, int> a;
    for (auto& c : calls_in) a[c.key] = c.assigned;
    ++decisions_;
    const int C = static_cast<int>(cars.size());

    std::vector<CallView> calls = calls_in;
    std::sort(calls.begin(), calls.end(), [](const CallView& x, const CallView& y) { return x.key < y.key; });
    const int n = static_cast<int>(calls.size());
    if (n == 0) return a;
    std::map<int, int> col_of_id;
    for (int c = 0; c < C; ++c) col_of_id[cars[c].id] = c;

    std::vector<int> locked(n, 0), fixed(n, -1), est(n, 1);
    double now = calls.front().registered + calls.front().waited;
    for (int i = 0; i < n; ++i) {
        if (calls[i].close_to_service && !(calls[i].key == nc) && calls[i].assigned >= 0 && col_of_id.count(calls[i].assigned)) {
            locked[i] = 1; fixed[i] = col_of_id[calls[i].assigned];
        }
        est[i] = estimation_for(calls[i].key, now);
    }

    std::mt19937_64 rng((opt_.seed + 0x9E3779B97F4A7C15ULL * static_cast<unsigned long long>(decisions_)) ^
                        (static_cast<unsigned long long>(n) << 40) ^ static_cast<unsigned long long>(std::llround(now * 1000.0)));
    auto rand_car = [&]() { return static_cast<int>(rng() % C); };
    auto complete = [&](std::vector<int>& g) { for (int i = 0; i < n; ++i) if (locked[i]) g[i] = fixed[i]; };

    const int P = std::max(2, opt_.pop);
    std::vector<std::vector<int>> pop(P, std::vector<int>(n, 0));
    for (auto& g : pop) { for (int i = 0; i < n; ++i) g[i] = rand_car(); complete(g); }
    int seed_idx = -1, lastbest_idx = -1;
    if (opt_.seeding) {
        seed_idx = 0;
        for (int i = 0; i < n; ++i) {
            double target = b.pos(calls[i].key.floor), best = std::numeric_limits<double>::infinity();
            int bc = 0;
            for (int c = 0; c < C; ++c) { double d = std::abs(cars[c].x - target); if (d < best - 1e-9) { best = d; bc = c; } }
            pop[0][i] = bc;
        }
        complete(pop[0]);
    }
    if (opt_.last_best && !last_best_.empty()) {
        lastbest_idx = opt_.seeding ? 1 : 0;
        auto& g = pop[lastbest_idx];
        for (int i = 0; i < n; ++i) {

            auto it = last_best_.find({calls[i].key, calls[i].registered});
            g[i] = (it != last_best_.end() && col_of_id.count(it->second)) ? col_of_id[it->second] : rand_car();
        }
        complete(g);
    }

    int evaluations = 0;
    auto eval = [&](const std::vector<int>& g) { ++evaluations; return fitness(b, cars, calls, g, est, opt_); };
    std::vector<double> fit(P);
    for (int h = 0; h < P; ++h) fit[h] = eval(pop[h]);
    auto argmin = [](const std::vector<double>& v) { return static_cast<int>(std::min_element(v.begin(), v.end()) - v.begin()); };
    std::vector<double> best_by_gen;
    int bi = argmin(fit);
    best_by_gen.push_back(fit[bi]);
    double seed_fit = seed_idx >= 0 ? fit[seed_idx] : -1.0, lastbest_fit = lastbest_idx >= 0 ? fit[lastbest_idx] : -1.0;
    std::vector<int> elite = pop[bi];
    double elite_fit = fit[bi];
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    for (int gen = 0; gen < opt_.gens; ++gen) {

        std::vector<double> wsel(P);
        for (int h = 0; h < P; ++h) wsel[h] = 1.0 / (fit[h] + 1.0);
        std::discrete_distribution<int> roulette(wsel.begin(), wsel.end());
        std::vector<std::vector<int>> children;
        children.reserve(P);
        while (static_cast<int>(children.size()) < P) {
            const std::vector<int>& p1 = pop[roulette(rng)];
            const std::vector<int>& p2 = pop[roulette(rng)];
            std::vector<int> c1 = p1, c2 = p2;
            if (u01(rng) < opt_.pc) {

                for (int i = 0; i < n; ++i) {
                    if (locked[i] || p1[i] == p2[i]) continue;
                    bool from1 = rng() % 2 == 0;
                    c1[i] = from1 ? p1[i] : p2[i];
                    c2[i] = from1 ? p2[i] : p1[i];
                }
            }
            for (auto* ch : {&c1, &c2}) {

                if (u01(rng) < opt_.pm) {
                    std::vector<int> free;
                    for (int i = 0; i < n; ++i) if (!locked[i]) free.push_back(i);
                    if (!free.empty()) (*ch)[free[rng() % free.size()]] = rand_car();
                }
                if (static_cast<int>(children.size()) < P) children.push_back(*ch);
            }
        }
        std::vector<double> cfit(P);
        for (int h = 0; h < P; ++h) cfit[h] = eval(children[h]);

        int worst = static_cast<int>(std::max_element(cfit.begin(), cfit.end()) - cfit.begin());
        children[worst] = elite; cfit[worst] = elite_fit;
        pop.swap(children); fit.swap(cfit);
        bi = argmin(fit);
        elite = pop[bi]; elite_fit = fit[bi];
        best_by_gen.push_back(elite_fit);
    }

    const std::vector<int>& best = elite;
    last_best_.clear();
    for (int i = 0; i < n; ++i) { a[calls[i].key] = cars[best[i]].id; last_best_[{calls[i].key, calls[i].registered}] = cars[best[i]].id; }
    if (trace) {
        trace->method = name();
        trace->new_call = nc;
        for (auto& cv : calls) trace->calls.push_back(cv.key);
        for (auto& car : cars) trace->cars.push_back(car.id);
        for (auto& cv : calls) trace->before[cv.key] = cv.assigned;
        for (auto& cv : calls) trace->after[cv.key] = a[cv.key];
        trace->ga_best_by_gen = best_by_gen;
        trace->ga_seed_fit = seed_fit; trace->ga_lastbest_fit = lastbest_fit; trace->ga_final_fit = elite_fit;
        trace->ga_evaluations = evaluations;
        trace->ga_locked = locked;
        trace->ga_estimation = est;
        trace->micros = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
    }
    return a;
}

}
