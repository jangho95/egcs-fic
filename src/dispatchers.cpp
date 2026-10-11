#include "dispatchers.hpp"
#include "batch_choice.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <random>
#include <set>
#include <stdexcept>

namespace egcs2017 {

namespace {

std::map<HallKey, int> current_assignment(const std::vector<CallView>& calls) {
    std::map<HallKey, int> a;
    for (auto& c : calls) a[c.key] = c.assigned;
    return a;
}

double choose(int n, int k) {
    if (k < 0 || k > n) return 0.0;
    double r = 1.0;
    for (int i = 1; i <= k; ++i) r = r * (n - k + i) / i;
    return r;
}

double g_of(const std::vector<int>& car_of, const std::vector<std::vector<double>>& u,
            const std::vector<std::vector<std::vector<double>>>& w, int C, const CostOptions& opt) {
    int n = static_cast<int>(car_of.size());
    double g = 0.0;
    std::vector<std::vector<int>> S(C);
    for (int i = 0; i < n; ++i) { g += u[i][car_of[i]]; S[car_of[i]].push_back(i); }
    for (int c = 0; c < C; ++c) {
        for (size_t a = 0; a < S[c].size(); ++a)
            for (size_t b = a + 1; b < S[c].size(); ++b) g += w[S[c][a]][S[c][b]][c];
        if (opt.order_k >= 2 && opt.order_penalty > 0.0) g += opt.order_penalty * choose(static_cast<int>(S[c].size()), opt.order_k);
    }
    return g;
}

std::vector<int> greedy(const std::vector<std::vector<double>>& u, const std::vector<std::vector<std::vector<double>>>& w, int C,
                        const CostOptions& opt, std::vector<double>& p, std::vector<std::tuple<int, int, double>>& order) {
    const int n = static_cast<int>(u.size());

    p.assign(n, 0.0);
    for (int i = 0; i < n; ++i) {
        double best = -std::numeric_limits<double>::infinity();
        for (int c = 0; c < C; ++c) {
            double s = u[i][c];
            for (int j = 0; j < n; ++j) if (j != i) s += w[i][j][c];
            best = std::max(best, s);
        }
        p[i] = best;
    }
    std::vector<int> assigned_car(n, -1);
    std::vector<std::vector<int>> S(C);
    order.clear();
    for (int step = 0; step < n; ++step) {
        double best_gain = -std::numeric_limits<double>::infinity();
        int bi = -1, bc = -1;
        for (int i = 0; i < n; ++i) {
            if (assigned_car[i] >= 0) continue;
            for (int c = 0; c < C; ++c) {
                double gain = p[i] - u[i][c];
                for (int j : S[c]) gain -= w[i][j][c];
                if (opt.order_k >= 2 && opt.order_penalty > 0.0)
                    gain -= opt.order_penalty * choose(static_cast<int>(S[c].size()), opt.order_k - 1);
                if (gain > best_gain + 1e-12) { best_gain = gain; bi = i; bc = c; }
            }
        }
        assigned_car[bi] = bc;
        S[bc].push_back(bi);
        order.emplace_back(bi, bc, best_gain);
    }
    return assigned_car;
}

int local_search(std::vector<int>& car_of, const std::vector<std::vector<double>>& u,
                 const std::vector<std::vector<std::vector<double>>>& w, int C, const CostOptions& opt) {
    int n = static_cast<int>(car_of.size());
    double g = g_of(car_of, u, w, C, opt);
    int iters = 0;
    while (iters < opt.ls_max_iter) {
        double best = g;
        std::vector<int> best_x;
        for (int i = 0; i < n; ++i)
            for (int c = 0; c < C; ++c) {
                if (c == car_of[i]) continue;
                std::vector<int> x = car_of; x[i] = c;
                double v = g_of(x, u, w, C, opt);
                if (v < best - 1e-9) { best = v; best_x = x; }
            }
        for (int i = 0; i < n; ++i)
            for (int j = i + 1; j < n; ++j) {
                if (car_of[i] == car_of[j]) continue;
                std::vector<int> x = car_of; std::swap(x[i], x[j]);
                double v = g_of(x, u, w, C, opt);
                if (v < best - 1e-9) { best = v; best_x = x; }
            }
        if (best_x.empty()) break;
        car_of = best_x; g = best; ++iters;
    }
    return iters;
}

long exhaustive(std::vector<int>& car_of, const std::vector<std::vector<double>>& u,
                const std::vector<std::vector<std::vector<double>>>& w, int C, const CostOptions& opt) {
    int n = static_cast<int>(car_of.size());
    double states = std::pow(static_cast<double>(C), n);
    if (n > opt.opt_max_calls || states > static_cast<double>(opt.opt_max_states)) return 0;
    double best = g_of(car_of, u, w, C, opt);
    std::vector<int> x(n, 0), best_x = car_of;
    long count = 0;
    while (true) {
        ++count;
        double v = g_of(x, u, w, C, opt);
        if (v < best - 1e-9) { best = v; best_x = x; }
        int k = 0;
        while (k < n && ++x[k] == C) { x[k] = 0; ++k; }
        if (k == n) break;
    }
    car_of = best_x;
    return count;
}

}

std::map<HallKey, int> CollectiveDispatcher::decide(const Building& b, const std::vector<CarView>& cars,
                                                    const std::vector<CallView>& calls, const HallKey& nc,
                                                    DecisionTrace* trace) {
    auto a = current_assignment(calls);
    double best = std::numeric_limits<double>::infinity();
    int best_car = cars.front().id;
    double target = b.pos(nc.floor);
    bool all_full = std::all_of(cars.begin(), cars.end(), [&](const CarView& c) { return c.load >= b.capacity; });
    for (auto& car : cars) {
        if (!all_full && car.load >= b.capacity) continue;
        double dist;
        if (car.dir == 0 && !car.moving) {
            dist = std::abs(target - car.x);
        } else {
            int d = car.dir;
            bool ahead = (nc.floor - car.committed) * d >= 0;
            if (ahead && (nc.dir == d || true)) {
                dist = std::abs(target - car.x);
                if (nc.dir != d) {

                    int far = car.committed;
                    for (auto& [f, n] : car.car_calls) if ((f - far) * d > 0) far = f;
                    for (auto& k : car.assigned) if ((k.floor - far) * d > 0) far = k.floor;
                    if ((far - nc.floor) * d > 0) dist = std::abs(b.pos(far) - car.x) + std::abs(b.pos(far) - target);
                }
            } else {
                int far = car.committed;
                for (auto& [f, n] : car.car_calls) if ((f - far) * d > 0) far = f;
                for (auto& k : car.assigned) if ((k.floor - far) * d > 0) far = k.floor;
                dist = std::abs(b.pos(far) - car.x) + std::abs(b.pos(far) - target);
            }
        }
        if (dist < best - 1e-9) { best = dist; best_car = car.id; }
    }
    a[nc] = best_car;
    if (trace) { trace->method = name(); trace->new_call = nc; trace->after = a; }
    return a;
}

std::map<HallKey, int> EtaDispatcher::decide(const Building& b, const std::vector<CarView>& cars,
                                             const std::vector<CallView>& calls, const HallKey& nc,
                                             DecisionTrace* trace) {
    auto a = current_assignment(calls);
    double best = std::numeric_limits<double>::infinity();
    int best_car = cars.front().id;
    bool all_full = std::all_of(cars.begin(), cars.end(), [&](const CarView& c) { return c.load >= b.capacity; });
    for (auto& car : cars) {
        if (!all_full && car.load >= b.capacity) continue;
        std::vector<VCall> vc;
        int id = 0;
        for (auto& k : car.assigned) {
            if (k == nc) continue;
            int top = k.dir == UP ? b.floors - 1 : 0;
            int mid = static_cast<int>(std::lround((k.floor + top) / 2.0));
            if (mid == k.floor) mid = k.floor + k.dir;
            vc.push_back(VCall{k, mid, 1, id++});
        }
        int new_id = id;
        vc.push_back(VCall{nc, -1, 1, new_id});
        PathResult r = run_path(b, car, vc, 0);
        double eta = r.pickup[new_id] >= 0 ? r.pickup[new_id] : 1e9;
        if (eta < best - 1e-9) { best = eta; best_car = car.id; }
    }
    a[nc] = best_car;
    if (trace) { trace->method = name(); trace->new_call = nc; trace->after = a; }
    return a;
}

double onboard_stop_externality(const Building& b, const CarView& car, const HallKey& call) {

    if (car.load <= 0 || car.load >= b.capacity || car.car_calls.empty() || car.dir != call.dir || car.arriving || car.doors_open)
        return 0.0;
    double ahead = (b.pos(call.floor) - car.x) * car.dir;
    if (ahead <= 1e-9 || (call.floor - car.committed) * car.dir < 0) return 0.0;
    int first = car.car_calls.begin()->first;
    for (auto& [f, count] : car.car_calls) {
        (void)count;
        if ((f - call.floor) * car.dir <= 0) return 0.0;
        if ((f - first) * car.dir < 0) first = f;
    }
    double v = car.moving ? car.v : 0.0;
    double stop = b.kin.time_from_state(ahead, v) + b.kin.travel(call.floor, first);
    double direct = b.kin.time_from_state((b.pos(first) - car.x) * car.dir, v);
    return car.load * (std::max(0.0, stop - direct) + b.doors.dwell(1, 0));
}

Matrices build_matrices(const Building& b, const std::vector<CarView>& cars, const std::vector<CallView>& calls,
                        const HallKey& nc, const CostOptions& opt) {
    Matrices M;

    for (auto& c : calls)
        if (!c.close_to_service || c.key == nc) M.N.push_back(c);
    std::stable_sort(M.N.begin(), M.N.end(), [](const CallView& x, const CallView& y) { return x.registered < y.registered; });
    const int n = static_cast<int>(M.N.size()), C = static_cast<int>(cars.size());
    bool all_full = std::all_of(cars.begin(), cars.end(), [&](const CarView& c) { return c.load >= b.capacity; });

    M.locked.assign(C, {});
    for (auto& cv : calls)
        if (cv.close_to_service && !(cv.key == nc) && cv.assigned >= 0)
            for (int c = 0; c < C; ++c) if (cars[c].id == cv.assigned) M.locked[c].push_back(cv.key);
    M.u.assign(n, std::vector<double>(C, 0.0)); M.u_raw = M.u; M.adj = M.u; M.u_lk = M.u;
    M.w.assign(n, std::vector<std::vector<double>>(n, std::vector<double>(C, 0.0)));
    CostOptions raw; raw.bonus = false; raw.capacity_penalty = false;
    for (int c = 0; c < C; ++c) {
        const CarView& car = cars[c];
        std::vector<double> wi_raw(n);
        for (int i = 0; i < n; ++i) {
            wi_raw[i] = unary(b, car, M.N[i].key, raw);
            double adjusted = unary(b, car, M.N[i].key, opt);
            if (opt.onboard_stop_weight > 0.0)
                adjusted += opt.onboard_stop_weight * onboard_stop_externality(b, car, M.N[i].key);

            if (!opt.capacity_penalty && !all_full && car.load >= b.capacity) adjusted += 1e6;
            if (M.N[i].forbidden_car == car.id) adjusted += 1e6;
            M.u_raw[i][c] = wi_raw[i];
            M.adj[i][c] = adjusted - wi_raw[i];

            M.u[i][c] = adjusted;
            if (opt.locked_aware) {

                for (auto& l : M.locked[c]) {
                    double wl = unary(b, car, l, raw);
                    double v = pairwise(b, car, M.N[i].key, l, wi_raw[i], wl);
                    if (v < -1e-6) throw std::logic_error("negative locked pairwise term");
                    M.u[i][c] += std::max(0.0, v);
                    M.u_lk[i][c] += std::max(0.0, v);
                }
            }
        }
        if (opt.pairwise)
            for (int i = 0; i < n; ++i)
                for (int j = i + 1; j < n; ++j) {

                    double v = pairwise(b, car, M.N[i].key, M.N[j].key, wi_raw[i], wi_raw[j]);
                    if (v < -1e-6) throw std::logic_error("negative pairwise term: model/engine inconsistency");
                    if (v < 0.0) v = 0.0;
                    M.w[i][j][c] = M.w[j][i][c] = v;
                }
    }
    return M;
}

std::vector<double> path_cost_samples(const Building& b, const std::vector<CarView>& cars, const Matrices& M,
                                      const std::vector<int>& car_of, const std::vector<std::vector<int>>& dest_free,
                                      const std::vector<std::vector<std::vector<int>>>& dest_locked, bool with_locked) {
    const int n = static_cast<int>(M.N.size()), C = static_cast<int>(cars.size());
    const int K = n > 0 ? static_cast<int>(dest_free[0].size()) : 0;
    std::vector<double> totals(K, 0.0);
    for (int s = 0; s < K; ++s) {
        for (int c = 0; c < C; ++c) {
            std::vector<VCall> vc;
            int id = 0;
            for (int i = 0; i < n; ++i) if (car_of[i] == c) vc.push_back(VCall{M.N[i].key, dest_free[i][s], 1, id++});
            if (with_locked)
                for (size_t l = 0; l < M.locked[c].size(); ++l) vc.push_back(VCall{M.locked[c][l], dest_locked[c][l][s], 1, id++});
            if (vc.empty()) continue;
            PathResult r = run_path(b, cars[c], vc, 0);
            for (auto& call : vc) totals[s] += r.pickup[call.id] >= 0 ? r.pickup[call.id] : 1e6;
        }
    }
    return totals;
}

double reeval_adjustment(const Matrices& M, const std::vector<int>& car_of, int C, const CostOptions& opt) {
    const int n = static_cast<int>(M.N.size());
    double a = 0.0;
    std::vector<int> count(C, 0);
    for (int i = 0; i < n; ++i) { a += M.adj[i][car_of[i]]; ++count[car_of[i]]; }
    if (opt.order_k >= 2 && opt.order_penalty > 0.0)
        for (int c = 0; c < C; ++c) a += opt.order_penalty * choose(count[c], opt.order_k);
    return a;
}

std::vector<std::vector<int>> reeval_candidates(const std::vector<int>& search_result, const std::vector<int>& greedy_result,
                                                const std::vector<std::vector<double>>& u,
                                                const std::vector<std::vector<std::vector<double>>>& w, int C, const CostOptions& opt) {
    const int n = static_cast<int>(search_result.size());
    std::vector<std::vector<int>> cands;
    auto add = [&](const std::vector<int>& x) {
        for (auto& y : cands) if (y == x) return;
        cands.push_back(x);
    };
    add(search_result);
    add(greedy_result);
    std::vector<std::pair<double, std::vector<int>>> neigh;
    for (int i = 0; i < n; ++i)
        for (int c = 0; c < C; ++c) if (c != search_result[i]) { auto x = search_result; x[i] = c; neigh.emplace_back(g_of(x, u, w, C, opt), x); }
    for (int i = 0; i < n; ++i)
        for (int j = i + 1; j < n; ++j) if (search_result[i] != search_result[j]) { auto x = search_result; std::swap(x[i], x[j]); neigh.emplace_back(g_of(x, u, w, C, opt), x); }
    std::sort(neigh.begin(), neigh.end(), [](auto& p1, auto& p2) { return p1.first < p2.first; });
    for (auto& [gv, x] : neigh) { if (static_cast<int>(cands.size()) >= opt.reeval_candidates) break; add(x); }
    return cands;
}

void common_destination_samples(const Building& b, const Matrices& M, double first_registered, const HallKey& nc, int K,
                                std::vector<std::vector<int>>& dest_free, std::vector<std::vector<std::vector<int>>>& dest_locked) {
    const int n = static_cast<int>(M.N.size()), C = static_cast<int>(M.locked.size());
    std::mt19937_64 rng(static_cast<unsigned long long>(std::llround(first_registered * 1000.0)) * 1315423911ULL ^
                        (static_cast<unsigned long long>(nc.floor) << 32) ^ static_cast<unsigned long long>(nc.dir + 7) ^ static_cast<unsigned long long>(n));
    auto sample_dest = [&](const HallKey& k) {
        auto d = destinations(b, k);
        return d[rng() % d.size()];
    };
    dest_free.assign(n, std::vector<int>(K));
    for (int i = 0; i < n; ++i) for (int s2 = 0; s2 < K; ++s2) dest_free[i][s2] = sample_dest(M.N[i].key);
    dest_locked.assign(C, {});
    for (int c = 0; c < C; ++c) {
        dest_locked[c].resize(M.locked[c].size(), std::vector<int>(K));
        for (size_t l = 0; l < M.locked[c].size(); ++l) for (int s2 = 0; s2 < K; ++s2) dest_locked[c][l][s2] = sample_dest(M.locked[c][l]);
    }
}

std::map<HallKey, int> SubmodularDispatcher::decide(const Building& b, const std::vector<CarView>& cars,
                                                    const std::vector<CallView>& calls, const HallKey& nc,
                                                    DecisionTrace* trace) {
    auto t0 = std::chrono::steady_clock::now();
    auto us_since = [](std::chrono::steady_clock::time_point a0) {
        return std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - a0).count();
    };
    auto a = current_assignment(calls);
    Matrices M = build_matrices(b, cars, calls, nc, opt_);
    const int n = static_cast<int>(M.N.size()), C = static_cast<int>(cars.size());
    if (n == 0) return a;
    const auto& u = M.u;
    const auto& w = M.w;
    double us_matrix = us_since(t0);
    auto t_g = std::chrono::steady_clock::now();
    std::vector<double> p;
    std::vector<std::tuple<int, int, double>> order;
    std::vector<int> assigned_car = greedy(u, w, C, opt_, p, order);
    double us_greedy = us_since(t_g);

    auto t_s = std::chrono::steady_clock::now();
    double g_greedy = g_of(assigned_car, u, w, C, opt_), g_final = g_greedy;
    int search_iters = 0, search_mode = 0;
    bool cap_hit = false;
    std::vector<int> greedy_result = assigned_car;
    if (opt_.search != CostOptions::NONE) {
        long states = opt_.search == CostOptions::OPT ? exhaustive(assigned_car, u, w, C, opt_) : 0;
        if (states > 0) { search_mode = 2; search_iters = static_cast<int>(std::min<long>(states, 1 << 30)); }
        else { search_mode = 1; search_iters = local_search(assigned_car, u, w, C, opt_); cap_hit = search_iters >= opt_.ls_max_iter; }
        g_final = g_of(assigned_car, u, w, C, opt_);
    }
    double us_search = us_since(t_s);

    auto t_r = std::chrono::steady_clock::now();
    int reeval_n = 0, reeval_pick = -1;
    std::vector<double> reeval_scores, reeval_se, reeval_se_diff, reeval_g;
    if (opt_.reeval_candidates > 0 && n > 0) {
        std::vector<std::vector<int>> cands = reeval_candidates(assigned_car, greedy_result, u, w, C, opt_);

        std::vector<std::vector<int>> dest_free;
        std::vector<std::vector<std::vector<int>>> dest_locked;
        int K = opt_.reeval_samples;
        common_destination_samples(b, M, calls.empty() ? 0.0 : calls.front().registered, nc, K, dest_free, dest_locked);

        bool with_locked = opt_.reeval_legacy_score ? true : opt_.locked_aware;
        double best = std::numeric_limits<double>::infinity();
        std::vector<double> tot0;
        for (size_t ci = 0; ci < cands.size(); ++ci) {
            std::vector<double> tot = path_cost_samples(b, cars, M, cands[ci], dest_free, dest_locked, with_locked);
            if (ci == 0) tot0 = tot;
            double mean = 0.0; for (double v : tot) mean += v; mean /= K;
            double var = 0.0; for (double v : tot) var += (v - mean) * (v - mean); var = K > 1 ? var / (K - 1) : 0.0;

            double dmean = 0.0; for (int s2 = 0; s2 < K; ++s2) dmean += tot[s2] - tot0[s2]; dmean /= K;
            double dvar = 0.0; for (int s2 = 0; s2 < K; ++s2) { double e = tot[s2] - tot0[s2] - dmean; dvar += e * e; } dvar = K > 1 ? dvar / (K - 1) : 0.0;
            double score = mean + (opt_.reeval_legacy_score ? 0.0 : reeval_adjustment(M, cands[ci], C, opt_));
            reeval_scores.push_back(score); reeval_se.push_back(std::sqrt(var / K)); reeval_se_diff.push_back(std::sqrt(dvar / K));
            reeval_g.push_back(g_of(cands[ci], u, w, C, opt_));
            if (score < best - 1e-9) { best = score; reeval_pick = static_cast<int>(ci); }
        }
        reeval_n = static_cast<int>(cands.size());
        if (reeval_pick >= 0) assigned_car = cands[reeval_pick];
        g_final = g_of(assigned_car, u, w, C, opt_);
    }
    double us_reeval = us_since(t_r);
    int changed = 0;
    for (int i = 0; i < n; ++i) changed += assigned_car[i] != greedy_result[i];
    int removed = 0;
    for (int i = 0; i < n; ++i) {
        const CarView& car = cars[assigned_car[i]];

        bool near_full = opt_.capacity_penalty && car.load >= std::max(1, b.capacity - opt_.capacity_margin);
        a[M.N[i].key] = near_full ? -1 : car.id;
        removed += near_full;
    }
    if (trace) {
        trace->g_greedy = g_greedy; trace->g_final = g_final;
        trace->search_iterations = search_iters; trace->search_changed = changed; trace->search_mode = search_mode;
        trace->search_cap_hit = cap_hit;
        trace->us_matrix = us_matrix; trace->us_greedy = us_greedy; trace->us_search = us_search;
        trace->reeval_candidates = reeval_n; trace->reeval_pick = reeval_pick; trace->us_reeval = us_reeval;
        trace->reeval_scores = reeval_scores; trace->reeval_se = reeval_se; trace->reeval_se_diff = reeval_se_diff; trace->reeval_g = reeval_g;
        trace->reeval_removed = reeval_n > 0 ? removed : 0;
        trace->method = name();
        trace->new_call = nc;
        for (auto& cv : M.N) trace->calls.push_back(cv.key);
        for (auto& car : cars) trace->cars.push_back(car.id);
        trace->unary = u;
        trace->unary_lk = M.u_lk;
        trace->n_locked = 0;
        for (auto& l : M.locked) trace->n_locked += static_cast<int>(l.size());
        trace->pair = w;
        trace->p = p;
        trace->order = order;
        for (auto& cv : M.N) trace->before[cv.key] = cv.assigned;
        for (auto& cv : M.N) trace->after[cv.key] = a[cv.key];
    }
    auto t1 = std::chrono::steady_clock::now();
    if (trace) trace->micros = std::chrono::duration<double, std::micro>(t1 - t0).count();
    return a;
}

std::unique_ptr<Dispatcher> make_dispatcher(const std::string& method) {
    if (method == "batch_global_expected" || method == "batch_global_mean" || method == "batch_global_diag")
        return std::make_unique<BatchChoiceDispatcher>(method == "batch_global_mean",true,false,true,method == "batch_global_diag");
    if (method == "batch_expected" || method == "batch_mean")
        return std::make_unique<BatchChoiceDispatcher>(method == "batch_mean");
    if (method == "batch_route_expected" || method == "batch_route_mean")
        return std::make_unique<BatchChoiceDispatcher>(method == "batch_route_mean", true);
    if (method == "batch_route_reference_counts")
        return std::make_unique<BatchChoiceDispatcher>(true, true, true);
    if (method == "collective") return std::make_unique<CollectiveDispatcher>();
    if (method == "eta") return std::make_unique<EtaDispatcher>();
    CostOptions o;
    if (method == "unary") { o.pairwise = false; return std::make_unique<SubmodularDispatcher>(o, method); }
    if (method == "pairwise") return std::make_unique<SubmodularDispatcher>(o, method);
    if (method == "pairwise_bonus") { o.bonus = true; return std::make_unique<SubmodularDispatcher>(o, method); }
    if (method == "paper2017") { o.bonus = true; o.capacity_penalty = true; return std::make_unique<SubmodularDispatcher>(o, method); }
    if (method == "paper2017_order4") {
        o.bonus = true; o.capacity_penalty = true; o.order_k = 4; o.order_penalty = 5.0;
        return std::make_unique<SubmodularDispatcher>(o, method);
    }

    if (method == "paper2017_ls") { o.bonus = true; o.capacity_penalty = true; o.search = CostOptions::LS; return std::make_unique<SubmodularDispatcher>(o, method); }
    if (method == "paper2017_opt") { o.bonus = true; o.capacity_penalty = true; o.search = CostOptions::OPT; return std::make_unique<SubmodularDispatcher>(o, method); }

    o.bonus = true; o.capacity_penalty = true;
    if (method == "observed_stop_005" || method == "observed_stop_015" || method == "observed_stop_050") {
        o.search = CostOptions::LS; o.locked_aware = true;
        o.onboard_stop_weight = method == "observed_stop_005" ? 0.05 : (method == "observed_stop_015" ? 0.15 : 0.5);
        return std::make_unique<SubmodularDispatcher>(o, method);
    }
    if (method == "paper2017_lk") { o.locked_aware = true; return std::make_unique<SubmodularDispatcher>(o, method); }
    if (method == "paper2017_ls_lk") { o.search = CostOptions::LS; o.locked_aware = true; return std::make_unique<SubmodularDispatcher>(o, method); }
    if (method == "paper2017_ls_re2") { o.search = CostOptions::LS; o.reeval_candidates = 8; return std::make_unique<SubmodularDispatcher>(o, method); }
    if (method == "paper2017_ls_lk_re2") { o.search = CostOptions::LS; o.locked_aware = true; o.reeval_candidates = 8; return std::make_unique<SubmodularDispatcher>(o, method); }
    if (method == "paper2017_ls_re") { o.search = CostOptions::LS; o.reeval_candidates = 8; o.reeval_legacy_score = true; return std::make_unique<SubmodularDispatcher>(o, method); }
    if (method == "paper2017_ls_lk_re") { o.search = CostOptions::LS; o.locked_aware = true; o.reeval_candidates = 8; o.reeval_legacy_score = true; return std::make_unique<SubmodularDispatcher>(o, method); }
    throw std::invalid_argument("unknown method: " + method);
}

}

namespace egcs2017 {
double g_of_public(const std::vector<int>& car_of, const std::vector<std::vector<double>>& u,
                   const std::vector<std::vector<std::vector<double>>>& w, int C, const CostOptions& opt) { return g_of(car_of, u, w, C, opt); }
int local_search_public(std::vector<int>& car_of, const std::vector<std::vector<double>>& u,
                        const std::vector<std::vector<std::vector<double>>>& w, int C, const CostOptions& opt) { return local_search(car_of, u, w, C, opt); }
std::vector<int> greedy_public(const std::vector<std::vector<double>>& u, const std::vector<std::vector<std::vector<double>>>& w, int C,
                               const CostOptions& opt) { std::vector<double> p; std::vector<std::tuple<int, int, double>> order; return greedy(u, w, C, opt, p, order); }
long exhaustive_public(std::vector<int>& car_of, const std::vector<std::vector<double>>& u,
                       const std::vector<std::vector<std::vector<double>>>& w, int C, const CostOptions& opt) { return exhaustive(car_of, u, w, C, opt); }
}
