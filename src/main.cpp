#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "dispatchers.hpp"
#include "ga2022.hpp"
#include "sim.hpp"
#include "trace.hpp"
#include "traffic.hpp"
#include "observations.hpp"
#include <tuple>

using namespace egcs2017;

namespace {

struct Args {
    std::string cmd;
    std::map<std::string, std::string> kv;
    std::string get(const std::string& k, const std::string& d) const {
        auto it = kv.find(k);
        return it == kv.end() ? d : it->second;
    }
    double num(const std::string& k, double d) const { return std::stod(get(k, std::to_string(d))); }
    int integer(const std::string& k, int d) const { return std::stoi(get(k, std::to_string(d))); }
    bool has(const std::string& k) const { return kv.count(k) > 0; }
};

Args parse(int argc, char** argv) {
    Args a;
    if (argc < 2) throw std::invalid_argument("missing command");
    a.cmd = argv[1];
    for (int i = 2; i < argc; ++i) {
        std::string s = argv[i];
        if (s.rfind("--", 0) != 0) throw std::invalid_argument("bad argument " + s);
        std::string key = s.substr(2);
        std::string val = "1";
        if (i + 1 < argc && std::string(argv[i + 1]).rfind("--", 0) != 0) val = argv[++i];
        a.kv[key] = val;
    }
    return a;
}

std::vector<int> ints(const std::string& s) {
    std::vector<int> out;
    std::stringstream ss(s);
    std::string tok;
    while (std::getline(ss, tok, ',')) if (!tok.empty()) out.push_back(std::stoi(tok));
    return out;
}
std::vector<std::string> strs(const std::string& s) {
    std::vector<std::string> out;
    std::stringstream ss(s);
    std::string tok;
    while (std::getline(ss, tok, ',')) if (!tok.empty()) out.push_back(tok);
    return out;
}

Building building_from(const Args& a, int floors) {
    Building b;
    b.floors = floors;
    b.capacity = a.integer("capacity", 13);
    b.kin.v_max = a.num("speed", 1.6);
    b.kin.a_max = a.num("accel", 0.8);
    b.kin.settle = a.num("settle", 0.5);
    b.kin.floor_height = a.num("height", 3.5);
    b.doors.open = a.num("door-open", 2.0);
    b.doors.close = a.num("door-close", 3.0);
    b.doors.transfer = a.num("transfer", 1.2);
    return b;
}

GaOptions ga_options_from(const Args& a, unsigned long seed) {
    GaOptions g;
    g.pop = a.integer("ga-pop", 10); g.gens = a.integer("ga-gens", 10);
    g.pc = a.num("ga-pc", 0.8); g.pm = a.num("ga-pm", 0.1);
    g.seeding = a.integer("ga-seeding", 1) != 0; g.last_best = a.integer("ga-last-best", 1) != 0;
    g.pen_threshold = a.num("ga-pen-threshold", 60.0); g.pen_weight = a.num("ga-pen-weight", 1.0);
    g.seed = seed;
    std::string route = a.get("ga-route", "fig4");
    if (route == "fig4") g.route = GaOptions::FIG4;
    else if (route == "collective") g.route = GaOptions::COLLECTIVE;
    else throw std::invalid_argument("--ga-route must be fig4 or collective");
    std::string ewt = a.get("ga-ewt", "accumulate");
    if (ewt == "accumulate") g.ewt = GaOptions::ACCUMULATE;
    else if (ewt == "literal") g.ewt = GaOptions::LITERAL;
    else throw std::invalid_argument("--ga-ewt must be accumulate or literal");
    return g;
}

std::unique_ptr<Dispatcher> dispatcher_from(const Args& a, const std::string& method, unsigned long seed = 1) {
    if (method == "ga2022_all") return std::make_unique<Ga2022Dispatcher>(ga_options_from(a, seed), method);
    if (method == "ga2022_all_ewtp") { GaOptions g = ga_options_from(a, seed); g.estimate = true; return std::make_unique<Ga2022Dispatcher>(g, method); }
    return make_dispatcher(method);
}

SimOptions options_from(const Args& a) {
    SimOptions o;
    o.theta = a.num("theta", 5.0);
    o.button_info = a.integer("button-info", 1) != 0;
    o.retry_dt = a.num("retry", 1.0);
    o.check_invariants = a.has("check");
    o.check_predictions = a.has("check-pred");
    o.full_load_bypass = a.has("full-load-bypass");
    o.assist_age = a.num("assist-age", -1.0);
    o.idle_lobby = a.has("idle-lobby");
    if (o.full_load_bypass && o.check_predictions)
        throw std::invalid_argument("--check-pred has no capacity-aware bypass predictor; use --check and direct trajectory checks");
    std::string lock = a.get("lock", "theta");
    if (lock == "theta") o.lock_mode = SimOptions::THETA;
    else if (lock == "stability") o.lock_mode = SimOptions::STABILITY;
    else throw std::invalid_argument("--lock must be theta or stability");
    o.period = a.num("period", 0.0);
    return o;
}

bool rollout_method(const std::string& m, SimOptions& so) {
    if (m.rfind("ro_", 0) != 0) return false;
    std::string rest = m.substr(3);
    so.rollout.enabled = true;
    using F = SimOptions::Rollout::Future;
    if (rest.rfind("nofuture_", 0) == 0) { so.rollout.future = F::NONE; rest = rest.substr(9); }
    else if (rest.rfind("future_", 0) == 0) { so.rollout.future = F::TRUE_ARRIVALS; rest = rest.substr(7); }
    else if (rest.rfind("sampled_", 0) == 0) { so.rollout.future = F::SAMPLED_TRUE; rest = rest.substr(8); }
    else if (rest.rfind("est_", 0) == 0) { so.rollout.future = F::SAMPLED_EST; rest = rest.substr(4); }
    else throw std::invalid_argument("rollout method: " + m);
    if (rest.rfind("wait", 0) == 0) { so.rollout.system = false; rest = rest.substr(4); }
    else if (rest.rfind("sys", 0) == 0) { so.rollout.system = true; rest = rest.substr(3); }
    else throw std::invalid_argument("rollout method: " + m);

    while (!rest.empty()) {
        if (rest.size() < 2 || rest[0] != '_') throw std::invalid_argument("rollout method: " + m);
        size_t end = rest.find('_', 1);
        std::string tok = rest.substr(1, end == std::string::npos ? std::string::npos : end - 1);
        if (tok[0] == 'h') so.rollout.horizon = std::stod(tok.substr(1));
        else if (tok[0] == 'k') so.rollout.samples = std::stoi(tok.substr(1));
        else if (tok[0] == 'w') so.rollout.est_window = std::stod(tok.substr(1));
        else throw std::invalid_argument("rollout method: " + m);
        rest = end == std::string::npos ? "" : rest.substr(end);
    }
    return true;
}

std::string metrics_json(const Metrics& m) {
    std::ostringstream o;
    o << "{\"n\":" << m.n << ",\"unserved\":" << m.unserved << ",\"unfinished\":" << m.unfinished << ",\"awt\":" << m.awt
      << ",\"p95_wait\":" << m.p95_wait << ",\"max_wait\":" << m.max_wait << ",\"frac_over_60\":" << m.frac_over_60
      << ",\"att\":" << m.att << ",\"ajt\":" << m.ajt << ",\"p95_journey\":" << m.p95_journey << ",\"decisions\":" << m.decisions
      << ",\"reassignments\":" << m.reassignments << ",\"unassigned_retries\":" << m.unassigned_retries
      << ",\"decision_us_mean\":" << m.decision_us_mean << ",\"decision_us_p95\":" << m.decision_us_p95
      << ",\"decision_us_max\":" << m.decision_us_max << ",\"sim_end\":" << m.sim_end
      << ",\"search_decisions\":" << m.search_decisions << ",\"search_changed_decisions\":" << m.search_changed_decisions
      << ",\"search_exhaustive_decisions\":" << m.search_exhaustive_decisions << ",\"search_suboptimal_greedy\":" << m.search_suboptimal_greedy
      << ",\"search_iter_cap_hits\":" << m.search_iter_cap_hits << ",\"search_g_reduction_mean\":" << m.search_g_reduction_mean
      << ",\"us_matrix_mean\":" << m.us_matrix_mean << ",\"us_greedy_mean\":" << m.us_greedy_mean << ",\"us_search_mean\":" << m.us_search_mean
      << ",\"us_search_p95\":" << m.us_search_p95 << ",\"us_search_max\":" << m.us_search_max
      << ",\"reeval_decisions\":" << m.reeval_decisions << ",\"reeval_changed_decisions\":" << m.reeval_changed_decisions
      << ",\"reeval_removed_decisions\":" << m.reeval_removed_decisions << ",\"reeval_candidates_mean\":" << m.reeval_candidates_mean
      << ",\"us_reeval_mean\":" << m.us_reeval_mean << ",\"us_reeval_p95\":" << m.us_reeval_p95 << ",\"us_reeval_max\":" << m.us_reeval_max
      << ",\"rollout_decisions\":" << m.rollout_decisions << ",\"rollout_changed\":" << m.rollout_changed << ",\"us_rollout_mean\":" << m.us_rollout_mean
      << ",\"full_events\":" << m.full_events << ",\"full_left\":" << m.full_left << ",\"full_extra_wait_ps\":" << m.full_extra_wait_ps
      << ",\"passed_over_passengers\":" << m.passed_over_passengers << ",\"passed_over_max\":" << m.passed_over_max
      << ",\"episodes\":" << m.episodes << ",\"episodes_1\":" << m.episodes_1 << ",\"episodes_2\":" << m.episodes_2 << ",\"episodes_3p\":" << m.episodes_3p
      << ",\"stops\":" << m.stops << ",\"pickup_only_stops\":" << m.pickup_only_stops << ",\"dropoff_only_stops\":" << m.dropoff_only_stops << ",\"mixed_stops\":" << m.mixed_stops
      << ",\"onboard_dwell_ps\":" << m.onboard_dwell_ps << ",\"pickup_only_onboard_ps\":" << m.pickup_only_onboard_ps
      << ",\"stops_win\":" << m.stops_win << ",\"pickup_only_stops_win_down\":" << m.pickup_only_stops_win_down << ",\"pickup_only_stops_win_up\":" << m.pickup_only_stops_win_up
      << ",\"onboard_dwell_ps_win\":" << m.onboard_dwell_ps_win << ",\"pickup_only_onboard_ps_win_down\":" << m.pickup_only_onboard_ps_win_down << ",\"pickup_only_onboard_ps_win_up\":" << m.pickup_only_onboard_ps_win_up
      << ",\"calls_registered\":" << m.calls_registered
      << ",\"idle_assignments\":" << m.idle_assignments << ",\"empty_travel_ps\":" << m.empty_travel_ps
      << ",\"queue_max\":" << m.queue_max << ",\"queue_avg\":" << m.queue_avg << ",\"drain_time\":" << m.drain_time << ",\"assist_issued\":" << m.assist_issued << ",\"assist_us_total\":" << m.assist_us_total << "}";
    return o.str();
}

struct RunSpec {
    int floors, cars, seed;
    double rate;
    std::string method;
};

struct RunResult {
    Metrics metrics;
    PredictionStats predictions;
};

RunResult run_once(const Args& a, const RunSpec& r, Trace* trace) {
    Building b = building_from(a, r.floors);
    unsigned long traffic_seed = static_cast<unsigned long>(r.seed) * 1000003UL + r.floors * 101UL + r.cars * 7UL + static_cast<unsigned long>(r.rate * 10);
    std::string profile = a.get("profile", "interfloor");
    TrafficSpec spec = TrafficSpec::named(profile, r.floors, r.rate, a.num("population", 30.0), a.num("duration", 3600.0));

    auto passengers = profile == "interfloor"
                          ? interfloor_traffic(r.floors, r.rate, a.num("population", 30.0), a.num("duration", 3600.0), traffic_seed)
                          : profile_traffic(spec, traffic_seed);
    SimOptions so = options_from(a);
    if (r.method.rfind("batch_", 0) == 0 && !so.full_load_bypass)
        throw std::invalid_argument("batch policies require --full-load-bypass for their route model");
    if (profile != "interfloor") so.rollout.spec = &spec;

    if (!spec.phases.empty()) { so.win0 = spec.phases.back().t0; so.win1 = spec.phases.back().t1; }
    std::string method = r.method;
    if (rollout_method(r.method, so)) { method = so.rollout.base; so.rollout.seed = static_cast<unsigned long>(r.seed); }

    so.rollout.true_rate = r.rate / 100.0 * a.num("population", 30.0) * (r.floors - 1) / 300.0;
    if (a.has("rollout-diag")) {
        so.rollout.enabled = true; so.rollout.diag = true;
        so.rollout.diag_path = a.get("rollout-diag", "rollout_diag.jsonl");
        so.rollout.diag_samples = a.integer("rollout-diag-k", 16);
        so.rollout.horizon = a.num("rollout-diag-h", 30.0);
        so.rollout.seed = static_cast<unsigned long>(r.seed);
        for (int w : ints(a.get("rollout-diag-windows", "60,300,900"))) so.rollout.diag_windows.push_back(w);
    }

    std::unique_ptr<std::ofstream> dump;
    if (a.has("dump-states")) {
        dump = std::make_unique<std::ofstream>(a.get("dump-states", "states.jsonl"), std::ios::app);
        int min_calls = a.integer("dump-min-calls", 2);
        so.on_decision = [&, min_calls](double t, int index, const Building& bb, const std::vector<CarView>& cv, const std::vector<CallView>& calls, const HallKey& nc) {
            int n = 0;
            for (auto& k : calls) if (!k.close_to_service || k.key == nc) ++n;
            if (n < min_calls) return;
            CostOptions raw; raw.bonus = false; raw.capacity_penalty = false; raw.locked_aware = true;
            Matrices M = build_matrices(bb, cv, calls, nc, raw);
            std::ostringstream o;
            o << std::setprecision(17);
            o << "{\"floors\":" << r.floors << ",\"cars\":" << r.cars << ",\"rate\":" << r.rate << ",\"seed\":" << r.seed << ",\"t\":" << t << ",\"index\":" << index
              << ",\"new_call\":[" << nc.floor << "," << nc.dir << "],\"car_views\":[";
            for (size_t c = 0; c < cv.size(); ++c) {
                const CarView& v = cv[c];
                o << (c ? "," : "") << "{\"id\":" << v.id << ",\"x\":" << v.x << ",\"v\":" << v.v << ",\"dir\":" << v.dir << ",\"moving\":" << (v.moving ? "true" : "false")
                  << ",\"arriving\":" << (v.arriving ? "true" : "false") << ",\"committed\":" << v.committed << ",\"release\":" << v.release
                  << ",\"doors_open\":" << (v.doors_open ? "true" : "false") << ",\"load\":" << v.load << ",\"car_calls\":[";
                bool first = true;
                for (auto& [f, n2] : v.car_calls) { o << (first ? "" : ",") << "[" << f << "," << n2 << "]"; first = false; }
                o << "]}";
            }
            o << "],\"calls\":[";
            for (size_t i = 0; i < M.N.size(); ++i) o << (i ? "," : "") << "[" << M.N[i].key.floor << "," << M.N[i].key.dir << "," << M.N[i].registered << "," << M.N[i].assigned << "," << M.N[i].waiting << "]";
            o << "],\"locked\":[";
            for (size_t c = 0; c < M.locked.size(); ++c) {
                o << (c ? "," : "") << "[";
                for (size_t l = 0; l < M.locked[c].size(); ++l) o << (l ? "," : "") << "[" << M.locked[c][l].floor << "," << M.locked[c][l].dir << "]";
                o << "]";
            }
            auto mat = [&](const std::vector<std::vector<double>>& m) {
                o << "[";
                for (size_t i = 0; i < m.size(); ++i) { o << (i ? "," : "") << "["; for (size_t c = 0; c < m[i].size(); ++c) o << (c ? "," : "") << m[i][c]; o << "]"; }
                o << "]";
            };
            o << "],\"u_raw\":"; mat(M.u_raw);
            o << ",\"u_lk\":"; mat(M.u_lk);
            o << ",\"w\":[";
            for (size_t i = 0; i < M.w.size(); ++i) {
                o << (i ? "," : "") << "[";
                for (size_t j = 0; j < M.w[i].size(); ++j) { o << (j ? "," : "") << "["; for (size_t c = 0; c < M.w[i][j].size(); ++c) o << (c ? "," : "") << M.w[i][j][c]; o << "]"; }
                o << "]";
            }
            o << "]}\n";
            *dump << o.str();
        };
    }
    Simulation sim(b, r.cars, dispatcher_from(a, method, static_cast<unsigned long>(r.seed) * 7919UL + 17UL), passengers, trace, a.num("max-time", 4 * 3600.0), so);
    if (trace) {
        std::ostringstream meta;
        meta << "\"floors\":" << r.floors << ",\"cars\":" << r.cars << ",\"rate\":" << r.rate << ",\"seed\":" << r.seed
             << ",\"method\":\"" << r.method << "\",\"floor_height\":" << b.kin.floor_height << ",\"capacity\":" << b.capacity
             << ",\"passengers\":" << passengers.size() << ",\"duration\":" << a.num("duration", 3600.0)
             << ",\"theta\":" << so.theta << ",\"button_info\":" << (so.button_info ? "true" : "false")
             << ",\"lock\":\"" << (so.lock_mode == SimOptions::STABILITY ? "stability" : "theta") << "\",\"period\":" << so.period
             << ",\"trace_decisions_max\":" << a.integer("trace-decisions", 300);
        trace->meta(meta.str());
    }
    sim.run();
    RunResult out{sim.metrics(), sim.predictions()};
    if (trace) trace->finish(metrics_json(out.metrics));
    return out;
}

std::string predictions_text(const PredictionStats& ps) {
    std::ostringstream o;
    o << "prediction check: predicted " << ps.predicted << ", invalidated " << ps.invalidated << " (redecided " << ps.inv_redecided
      << ", joined " << ps.inv_joined << ", foreign boarding " << ps.inv_foreign_board << ", open-door boarding " << ps.inv_open_board
      << ", full release " << ps.inv_full_release << ", taken by another car " << ps.inv_taken << "), cars skipped (mixed destinations) "
      << ps.mixed_excluded << ", compared " << ps.compared << ", mismatches " << ps.mismatches;
    if (ps.mismatches) o << " (worst " << ps.worst << " s; first: " << ps.first_mismatch << ")";
    return o.str();
}

int cmd_run(const Args& a) {
    RunSpec r{a.integer("floors", 12), a.integer("cars", 3), a.integer("seed", 1), a.num("rate", 20.0), a.get("method", "pairwise")};
    std::unique_ptr<Trace> trace;
    if (a.has("trace")) trace = std::make_unique<Trace>(a.get("trace", "trace.json"), a.num("sample-dt", 0.5), a.integer("trace-decisions", 300));
    RunResult rr = run_once(a, r, trace.get());
    std::cout << metrics_json(rr.metrics) << "\n";
    if (a.has("check-pred")) {
        std::cerr << predictions_text(rr.predictions) << "\n";
        if (rr.predictions.mismatches) return 3;
    }
    return 0;
}

int cmd_grid(const Args& a) {
    auto floors = ints(a.get("floors", "8,10,12"));
    auto cars = ints(a.get("cars", "2,3,4,5,6"));
    auto rates = ints(a.get("rates", "10,15,20,25,30"));
    int seeds = a.integer("seeds", 10);
    int seed_from = a.integer("seed-from", 0);
    auto methods = strs(a.get("methods", "collective,eta,unary,pairwise,pairwise_bonus,paper2017,paper2017_order4"));
    int threads = a.integer("threads", static_cast<int>(std::thread::hardware_concurrency()));
    for (auto& [k, v] : a.kv) {
        static const std::set<std::string> known{"out", "floors", "cars", "rates", "seeds", "seed-from", "methods", "threads", "population", "duration",
                                                 "max-time", "capacity", "speed", "accel", "settle", "height", "door-open", "door-close", "transfer",
                                                 "theta", "button-info", "retry", "check", "check-pred", "lock", "period",
                                                 "ga-pop", "ga-gens", "ga-pc", "ga-pm", "ga-seeding", "ga-last-best", "ga-pen-threshold", "ga-pen-weight",
                                                 "ga-route", "ga-ewt", "profile", "full-load-bypass", "assist-age", "idle-lobby"};
        if (!known.count(k)) throw std::invalid_argument("unknown option --" + k);
    }
    std::vector<RunSpec> specs;
    for (int f : floors) for (int c : cars) for (int r : rates) for (int s = seed_from; s < seed_from + seeds; ++s) for (auto& m : methods)
        specs.push_back(RunSpec{f, c, s, static_cast<double>(r), m});
    std::vector<RunResult> results(specs.size());
    std::atomic<size_t> next{0};
    std::atomic<size_t> done{0};
    auto worker = [&]() {
        while (true) {
            size_t i = next.fetch_add(1);
            if (i >= specs.size()) return;
            results[i] = run_once(a, specs[i], nullptr);
            size_t d = ++done;
            if (d % 500 == 0 || d == specs.size()) std::cerr << d << "/" << specs.size() << " runs\n";
        }
    };
    std::vector<std::thread> pool;
    for (int t = 0; t < std::max(1, threads); ++t) pool.emplace_back(worker);
    for (auto& t : pool) t.join();

    std::ofstream out(a.get("out", "results.csv"));
    out << std::setprecision(10);
    out << "floors,cars,rate,seed,method,n,unserved,unfinished,awt,p95_wait,max_wait,frac_over_60,att,ajt,p95_journey,decisions,reassignments,unassigned_retries,decision_us_mean,decision_us_p95,sim_end,search_decisions,search_changed_decisions,search_exhaustive_decisions,search_suboptimal_greedy,search_iter_cap_hits,search_g_reduction_mean,us_matrix_mean,us_greedy_mean,us_search_mean,us_search_p95,us_search_max,reeval_decisions,reeval_changed_decisions,reeval_removed_decisions,reeval_candidates_mean,us_reeval_mean,us_reeval_p95,us_reeval_max,rollout_decisions,rollout_changed,us_rollout_mean,full_events,full_left,full_extra_wait_ps,passed_over_passengers,passed_over_max,episodes,episodes_1,episodes_2,episodes_3p,stops,pickup_only_stops,dropoff_only_stops,mixed_stops,onboard_dwell_ps,pickup_only_onboard_ps,stops_win,pickup_only_stops_win_down,pickup_only_stops_win_up,onboard_dwell_ps_win,pickup_only_onboard_ps_win_down,pickup_only_onboard_ps_win_up,calls_registered,idle_assignments,empty_travel_ps,queue_max,queue_avg,drain_time,assist_issued,assist_us_total\n";
    for (size_t i = 0; i < specs.size(); ++i) {
        const RunSpec& r = specs[i];
        const Metrics& m = results[i].metrics;
        out << r.floors << "," << r.cars << "," << r.rate << "," << r.seed << "," << r.method << "," << m.n << "," << m.unserved << ","
            << m.unfinished << "," << m.awt << "," << m.p95_wait << "," << m.max_wait << "," << m.frac_over_60 << "," << m.att << "," << m.ajt << ","
            << m.p95_journey << "," << m.decisions << "," << m.reassignments << "," << m.unassigned_retries << "," << m.decision_us_mean << ","
            << m.decision_us_p95 << "," << m.sim_end << "," << m.search_decisions << "," << m.search_changed_decisions << ","
            << m.search_exhaustive_decisions << "," << m.search_suboptimal_greedy << "," << m.search_iter_cap_hits << "," << m.search_g_reduction_mean << ","
            << m.us_matrix_mean << "," << m.us_greedy_mean << "," << m.us_search_mean << "," << m.us_search_p95 << "," << m.us_search_max << ","
            << m.reeval_decisions << "," << m.reeval_changed_decisions << "," << m.reeval_removed_decisions << "," << m.reeval_candidates_mean << ","
            << m.us_reeval_mean << "," << m.us_reeval_p95 << "," << m.us_reeval_max << "," << m.rollout_decisions << "," << m.rollout_changed << "," << m.us_rollout_mean << ","
            << m.full_events << "," << m.full_left << "," << m.full_extra_wait_ps << "," << m.passed_over_passengers << "," << m.passed_over_max << ","
            << m.episodes << "," << m.episodes_1 << "," << m.episodes_2 << "," << m.episodes_3p << "," << m.stops << "," << m.pickup_only_stops << ","
            << m.dropoff_only_stops << "," << m.mixed_stops << "," << m.onboard_dwell_ps << "," << m.pickup_only_onboard_ps << "," << m.stops_win << ","
            << m.pickup_only_stops_win_down << "," << m.pickup_only_stops_win_up << "," << m.onboard_dwell_ps_win << "," << m.pickup_only_onboard_ps_win_down << ","
            << m.pickup_only_onboard_ps_win_up << "," << m.calls_registered << "," << m.idle_assignments << "," << m.empty_travel_ps << ","
            << m.queue_max << "," << m.queue_avg << "," << m.drain_time << "," << m.assist_issued << "," << m.assist_us_total << "\n";
    }
    if (a.has("check-pred")) {
        PredictionStats all;
        for (auto& rr : results) all.merge(rr.predictions);
        std::cerr << predictions_text(all) << "\n";
        if (all.mismatches) return 3;
    }
    return 0;
}

int cmd_costs(const Args& a) {
    std::ifstream in(a.get("snapshot", "state.txt"));
    if (!in) throw std::runtime_error("cannot read snapshot");
    int floors = 12;
    CarView car;
    std::vector<HallKey> calls;
    std::string line;
    while (std::getline(in, line)) {
        std::istringstream ss(line);
        std::string k;
        ss >> k;
        if (k == "floors") ss >> floors;
        else if (k == "car") { int f, d; ss >> f >> d; car.committed = f; car.x = 0; car.dir = d; car.moving = false; car.release = 0.0; car.doors_open = false; }
        else if (k == "carcall") { int f, n; ss >> f >> n; car.car_calls[f] += n; }
        else if (k == "call") { int f, d; ss >> f >> d; calls.push_back(HallKey{f, d}); }
    }
    Building b = building_from(a, floors);
    car.x = b.pos(car.committed);
    CostOptions raw;
    std::vector<double> u;
    for (auto& c : calls) u.push_back(unary(b, car, c, raw));
    std::cout << "{\"unary\":[";
    for (size_t i = 0; i < u.size(); ++i) std::cout << (i ? "," : "") << std::setprecision(12) << u[i];
    std::cout << "],\"pair\":[";
    for (size_t i = 0; i < calls.size(); ++i) {
        std::cout << (i ? "," : "") << "[";
        for (size_t j = 0; j < calls.size(); ++j) {
            double v = i == j ? 0.0 : pairwise(b, car, calls[i], calls[j], u[i], u[j]);
            std::cout << (j ? "," : "") << std::setprecision(12) << v;
        }
        std::cout << "]";
    }
    std::cout << "]}\n";
    return 0;
}

double f_value(const std::vector<int>& car_of, const std::vector<std::vector<double>>& u,
               const std::vector<std::vector<std::vector<double>>>& w, const std::vector<double>& p, int C,
               int order_k, double order_penalty) {
    int n = static_cast<int>(u.size());
    double g = 0.0, unassigned = 0.0;
    std::vector<std::vector<int>> S(C);
    for (int i = 0; i < n; ++i) {
        if (car_of[i] < 0) { unassigned += p[i]; continue; }
        g += u[i][car_of[i]];
        S[car_of[i]].push_back(i);
    }
    for (int c = 0; c < C; ++c) {
        for (size_t a = 0; a < S[c].size(); ++a)
            for (size_t b = a + 1; b < S[c].size(); ++b) g += w[S[c][a]][S[c][b]][c];
        if (order_k >= 2 && order_penalty > 0.0) {

            int m = static_cast<int>(S[c].size());
            double comb = 1.0;
            for (int i = 1; i <= order_k; ++i) comb = comb * (m - order_k + i) / i;
            if (m >= order_k) g += order_penalty * comb;
        }
    }

    double h1 = 0.0, cst = 0.0;
    for (int i = 0; i < n; ++i) { h1 -= p[i] * (C - (car_of[i] >= 0 ? 1 : 0)); cst += C * p[i]; }
    return -g + h1 + cst;
}

CarView random_rest_car(std::mt19937_64& g, const Building& b, int id) {
    CarView car;
    car.id = id;
    car.committed = static_cast<int>(g() % b.floors);
    car.x = b.pos(car.committed);
    car.dir = (g() % 3 == 0) ? 0 : (g() % 2 ? UP : DOWN);
    car.load = static_cast<int>(g() % 4);
    if (car.dir != 0) {
        int k = static_cast<int>(g() % 3);
        for (int q = 0; q < k; ++q) {
            int f = static_cast<int>(g() % b.floors);
            if ((f - car.committed) * car.dir > 0) car.car_calls[f] = 1;
        }
        if (car.car_calls.empty() && (g() % 2)) car.dir = 0;
    }
    return car;
}
HallKey random_call(std::mt19937_64& g, const Building& b) {
    while (true) {
        HallKey k{static_cast<int>(g() % b.floors), g() % 2 ? UP : DOWN};
        if (k.dir == UP && k.floor == b.floors - 1) continue;
        if (k.dir == DOWN && k.floor == 0) continue;
        return k;
    }
}

double reference_path_cost(const Building& b, const std::vector<CarView>& cars, const Matrices& M, const std::vector<int>& car_of,
                           bool with_locked, long max_enum, int n_sample, std::mt19937_64& g, int* exact_cars) {
    const int n = static_cast<int>(M.N.size()), C = static_cast<int>(cars.size());
    double total = 0.0;
    if (exact_cars) *exact_cars = 0;
    for (int c = 0; c < C; ++c) {
        std::vector<HallKey> keys;
        for (int i = 0; i < n; ++i) if (car_of[i] == c) keys.push_back(M.N[i].key);
        if (with_locked) for (auto& l : M.locked[c]) keys.push_back(l);
        if (keys.empty()) continue;
        std::vector<std::vector<int>> dests;
        double combos = 1.0;
        for (auto& k : keys) { dests.push_back(destinations(b, k)); combos *= dests.back().size(); }
        auto cost_of = [&](const std::vector<int>& d) {
            std::vector<VCall> vc;
            for (size_t q = 0; q < keys.size(); ++q) vc.push_back(VCall{keys[q], d[q], 1, static_cast<int>(q)});
            PathResult r = run_path(b, cars[c], vc, 0);
            double t = 0.0;
            for (auto& call : vc) t += r.pickup[call.id] >= 0 ? r.pickup[call.id] : 1e6;
            return t;
        };
        std::vector<int> idx(keys.size(), 0), d(keys.size());
        if (combos <= static_cast<double>(max_enum)) {
            double sum = 0.0; long cnt = 0;
            while (true) {
                for (size_t q = 0; q < keys.size(); ++q) d[q] = dests[q][idx[q]];
                sum += cost_of(d); ++cnt;
                size_t q = 0;
                while (q < keys.size() && ++idx[q] == static_cast<int>(dests[q].size())) { idx[q] = 0; ++q; }
                if (q == keys.size()) break;
            }
            total += sum / cnt;
            if (exact_cars) ++*exact_cars;
        } else {
            double sum = 0.0;
            for (int s2 = 0; s2 < n_sample; ++s2) {
                for (size_t q = 0; q < keys.size(); ++q) d[q] = dests[q][g() % dests[q].size()];
                sum += cost_of(d);
            }
            total += sum / n_sample;
        }
    }
    return total;
}

void all_destination_combinations(const Building& b, const Matrices& M, std::vector<std::vector<int>>& dest_free) {
    const int n = static_cast<int>(M.N.size());
    std::vector<std::vector<int>> dests;
    long K = 1;
    for (int i = 0; i < n; ++i) { dests.push_back(destinations(b, M.N[i].key)); K *= dests.back().size(); }
    dest_free.assign(n, std::vector<int>(K));
    std::vector<int> idx(n, 0);
    for (long s2 = 0; s2 < K; ++s2) {
        for (int i = 0; i < n; ++i) dest_free[i][s2] = dests[i][idx[i]];
        int q = 0;
        while (q < n && ++idx[q] == static_cast<int>(dests[q].size())) { idx[q] = 0; ++q; }
    }
}

int cmd_reeval_check(const Args& a) {
    auto floors = ints(a.get("floors", "10"));
    auto cars_list = ints(a.get("cars", "3,4"));
    auto rates = ints(a.get("rates", "20,30"));
    int seeds = a.integer("seeds", 2), seed_from = a.integer("seed-from", 0);
    int max_states = a.integer("max-states", 400), min_calls = a.integer("min-calls", 2), min_total = a.integer("min-total", 3);
    long ref_enum = a.integer("ref-enum", 20000);
    int ref_samples = a.integer("ref-samples", 4000), K = a.integer("samples", 16);
    std::string method = a.get("method", "paper2017_ls_lk");
    struct State { Building b; std::vector<CarView> cars; std::vector<CallView> calls; HallKey nc; int floors, ncars; double rate; int seed; double t; int index; };
    std::vector<State> states;
    std::mt19937_64 pick(a.integer("pick-seed", 5));
    long seen = 0;
    for (int f : floors) for (int c : cars_list) for (int r : rates) for (int sd = seed_from; sd < seed_from + seeds; ++sd) {
        Building b = building_from(a, f);
        auto passengers = interfloor_traffic(f, r, a.num("population", 30.0), a.num("duration", 3600.0),
                                             static_cast<unsigned long>(sd) * 1000003UL + f * 101UL + c * 7UL + static_cast<unsigned long>(r * 10));
        SimOptions so = options_from(a);
        std::vector<State> local;
        so.on_decision = [&](double t, int index, const Building& bb, const std::vector<CarView>& cv, const std::vector<CallView>& calls, const HallKey& nc) {
            int n = 0, locked = 0;
            for (auto& k : calls) { if (!k.close_to_service || k.key == nc) ++n; else ++locked; }
            if (n < min_calls || n + locked < min_total) return;
            ++seen;
            local.push_back(State{bb, cv, calls, nc, f, c, static_cast<double>(r), sd, t, index});
        };
        Simulation sim(b, c, make_dispatcher(method), passengers, nullptr, a.num("max-time", 4 * 3600.0), so);
        sim.run();
        for (auto& st : local) states.push_back(st);
    }

    if (static_cast<int>(states.size()) > max_states) {
        std::shuffle(states.begin(), states.end(), pick);
        states.resize(max_states);
    }
    CostOptions o_lk; o_lk.bonus = true; o_lk.capacity_penalty = true; o_lk.locked_aware = true; o_lk.search = CostOptions::LS; o_lk.reeval_candidates = a.integer("candidates", 8); o_lk.reeval_samples = K;
    CostOptions o_plain = o_lk; o_plain.locked_aware = false;
    const char* RULES[4] = {"g_plain", "g_lk", "path16_plain", "path16_lk"};

    int n_states[2] = {0, 0}, ref_not_first[2] = {0, 0};
    double potential[2] = {0, 0};
    int agree[2][4] = {{0, 0, 0, 0}, {0, 0, 0, 0}};
    double regret[2][4] = {{0, 0, 0, 0}, {0, 0, 0, 0}}, abs_err[2][4] = {{0, 0, 0, 0}, {0, 0, 0, 0}}, sq_err[2][4] = {{0, 0, 0, 0}, {0, 0, 0, 0}};
    long n_cands[2] = {0, 0};
    std::mt19937_64 ref_rng(a.integer("ref-seed", 99));
    std::ostringstream js;
    js << "[";
    bool first_js = true;
    for (auto& st : states) {
        Matrices M = build_matrices(st.b, st.cars, st.calls, st.nc, o_lk);
        Matrices Mp = build_matrices(st.b, st.cars, st.calls, st.nc, o_plain);
        const int C = static_cast<int>(st.cars.size());
        std::vector<int> greedy_x = greedy_public(M.u, M.w, C, o_lk), ls_x = greedy_x;
        local_search_public(ls_x, M.u, M.w, C, o_lk);
        auto cands = reeval_candidates(ls_x, greedy_x, M.u, M.w, C, o_lk);
        if (cands.size() < 2) continue;
        std::vector<std::vector<int>> dest_free;
        std::vector<std::vector<std::vector<int>>> dest_locked;
        common_destination_samples(st.b, M, st.calls.empty() ? 0.0 : st.calls.front().registered, st.nc, K, dest_free, dest_locked);
        std::vector<double> ref(cands.size()), val[4], se_diff[2];
        for (auto& v : val) v.resize(cands.size());
        for (auto& v : se_diff) v.assign(cands.size(), 0.0);
        std::vector<double> tp0, tl0;
        bool all_exact = true;
        for (size_t ci = 0; ci < cands.size(); ++ci) {
            int exact_cars = 0;
            double rp = reference_path_cost(st.b, st.cars, M, cands[ci], true, ref_enum, ref_samples, ref_rng, &exact_cars);
            int used_cars = 0;
            for (int c = 0; c < C; ++c) { bool any = !M.locked[c].empty(); for (size_t i = 0; i < cands[ci].size(); ++i) any |= cands[ci][i] == c; used_cars += any; }
            all_exact &= exact_cars == used_cars;
            ref[ci] = rp + reeval_adjustment(M, cands[ci], C, o_lk);
            val[0][ci] = g_of_public(cands[ci], Mp.u, Mp.w, C, o_plain);
            val[1][ci] = g_of_public(cands[ci], M.u, M.w, C, o_lk);
            auto tp = path_cost_samples(st.b, st.cars, M, cands[ci], dest_free, dest_locked, false);
            auto tl = path_cost_samples(st.b, st.cars, M, cands[ci], dest_free, dest_locked, true);
            if (ci == 0) { tp0 = tp; tl0 = tl; }
            double mp = 0, ml = 0; for (double v : tp) mp += v; for (double v : tl) ml += v;
            val[2][ci] = mp / K + reeval_adjustment(M, cands[ci], C, o_plain);
            val[3][ci] = ml / K + reeval_adjustment(M, cands[ci], C, o_lk);

            auto sed = [&](const std::vector<double>& x, const std::vector<double>& x0) {
                double m = 0; for (int s2 = 0; s2 < K; ++s2) m += x[s2] - x0[s2]; m /= K;
                double v = 0; for (int s2 = 0; s2 < K; ++s2) { double e = x[s2] - x0[s2] - m; v += e * e; }
                return K > 1 ? std::sqrt(v / (K - 1) / K) : 0.0;
            };
            se_diff[0][ci] = sed(tp, tp0); se_diff[1][ci] = sed(tl, tl0);
        }
        auto argmin = [](const std::vector<double>& v) { size_t b = 0; for (size_t i = 1; i < v.size(); ++i) if (v[i] < v[b] - 1e-9) b = i; return b; };
        size_t rb = argmin(ref);
        const int grp = all_exact ? 0 : 1;
        ++n_states[grp];
        ref_not_first[grp] += rb != 0;
        potential[grp] += ref[0] - ref[rb];
        js << (first_js ? "" : ",") << "{\"floors\":" << st.floors << ",\"cars\":" << st.ncars << ",\"rate\":" << st.rate << ",\"seed\":" << st.seed
           << ",\"t\":" << fmt(st.t) << ",\"decision_index\":" << st.index << ",\"n\":" << M.N.size() << ",\"locked\":";
        int nl = 0; for (auto& l : M.locked) nl += static_cast<int>(l.size());
        js << nl << ",\"candidates\":" << cands.size() << ",\"exact\":" << (all_exact ? "true" : "false") << ",\"ref\":[";
        for (size_t ci = 0; ci < cands.size(); ++ci) js << (ci ? "," : "") << fmt(ref[ci]);
        js << "]";
        first_js = false;
        for (int r = 0; r < 4; ++r) {
            size_t pb = argmin(val[r]);
            agree[grp][r] += pb == rb;
            regret[grp][r] += ref[pb] - ref[rb];

            for (size_t ci = 1; ci < cands.size(); ++ci) {
                double e = (val[r][ci] - val[r][0]) - (ref[ci] - ref[0]);
                abs_err[grp][r] += std::abs(e); sq_err[grp][r] += e * e;
            }
            js << ",\"" << RULES[r] << "\":[";
            for (size_t ci = 0; ci < cands.size(); ++ci) js << (ci ? "," : "") << fmt(val[r][ci]);
            js << "]";
        }
        for (int q = 0; q < 2; ++q) {
            js << ",\"" << RULES[2 + q] << "_se_diff\":[";
            for (size_t ci = 0; ci < cands.size(); ++ci) js << (ci ? "," : "") << fmt(se_diff[q][ci]);
            js << "]";
        }
        n_cands[grp] += static_cast<long>(cands.size()) - 1;
        js << "}";
    }
    js << "]";
    if (a.has("out")) { std::ofstream f(a.get("out", "reeval_check.json")); f << js.str() << "\n"; }
    std::cerr << "reeval-check: states seen " << seen << ", analysed " << n_states[0] + n_states[1] << " (reference exact on every used car of every candidate: "
              << n_states[0] << "; partly sampled with " << ref_samples << " samples: " << n_states[1] << "), method " << method << ", K=" << K << ", candidates<=" << o_lk.reeval_candidates << "\n";
    for (int grp = 0; grp < 2; ++grp) {
        if (n_states[grp] == 0) continue;
        std::cerr << (grp == 0 ? "-- exact-reference states" : "-- partly sampled-reference states (reference error and min-selection bias included)") << ": "
                  << "reference argmin != search result in " << ref_not_first[grp] << "/" << n_states[grp]
                  << "; mean potential gain (ref[0]-ref_min) " << std::fixed << std::setprecision(3) << potential[grp] / n_states[grp] << " s per decision (summed pickup time of the calls)\n";
        std::cerr << std::left << std::setw(14) << "rule" << std::right << std::setw(12) << "agree" << std::setw(12) << "regret/s" << std::setw(14) << "MAE(diff)/s" << std::setw(14) << "RMSE(diff)/s\n";
        for (int r = 0; r < 4; ++r)
            std::cerr << std::left << std::setw(14) << RULES[r] << std::right << std::setw(12) << (std::to_string(agree[grp][r]) + "/" + std::to_string(n_states[grp]))
                      << std::setw(12) << std::fixed << std::setprecision(3) << regret[grp][r] / n_states[grp] << std::setw(14) << abs_err[grp][r] / std::max<long>(1, n_cands[grp])
                      << std::setw(14) << std::sqrt(sq_err[grp][r] / std::max<long>(1, n_cands[grp])) << "\n";
    }
    return 0;
}

int cmd_counterfactual(const Args& a) {
    RunSpec r{a.integer("floors", 10), a.integer("cars", 3), a.integer("seed", 0), a.num("rate", 30.0), "paper2017_ls_lk"};
    int max_states = a.integer("max-states", 3);
    std::string profile = a.get("profile", "downt");

    bool no_future = a.has("no-future");
    bool branch_fixed_exclusive = a.has("branch-fixed-exclusive");
    int branch_to_car = a.integer("branch-to-car", -1);
    if (branch_to_car < -1 || branch_to_car >= r.cars) throw std::runtime_error("branch-to-car outside car range");
    if (branch_to_car >= 0 && !no_future) throw std::runtime_error("branch-to-car requires no-future");
    if (branch_fixed_exclusive && !no_future) throw std::runtime_error("branch-fixed-exclusive requires no-future");
    if (a.has("branch-observations") && a.integer("button-info", 1) != 1) throw std::runtime_error("branch-observations requires button-info 1");
    bool fix_others = a.has("fix-others");
    Building b = building_from(a, r.floors);
    unsigned long traffic_seed = static_cast<unsigned long>(r.seed) * 1000003UL + r.floors * 101UL + r.cars * 7UL + static_cast<unsigned long>(r.rate * 10);
    TrafficSpec spec = TrafficSpec::named(profile, r.floors, r.rate, a.num("population", 30.0), a.num("duration", 3600.0));
    auto passengers = profile == "interfloor" ? interfloor_traffic(r.floors, r.rate, a.num("population", 30.0), a.num("duration", 3600.0), traffic_seed)
                                              : profile_traffic(spec, traffic_seed);
    SimOptions so = options_from(a);
    if (!spec.phases.empty()) { so.win0 = spec.phases.back().t0; so.win1 = spec.phases.back().t1; }
    so.stop_log = true;

    Simulation orig(b, r.cars, make_dispatcher("paper2017_ls_lk"), passengers, nullptr, 4 * 3600.0, so);
    orig.run();
    std::vector<Simulation::StopRecord> cands;
    for (auto& st : orig.stop_log()) if (st.pickup_only && st.dir == DOWN && st.in_window && st.continuing > 0) cands.push_back(st);
    std::sort(cands.begin(), cands.end(), [](const Simulation::StopRecord& x, const Simulation::StopRecord& y) {
        double lx = x.duration * x.continuing, ly = y.duration * y.continuing;
        return lx != ly ? lx > ly : x.t0 < y.t0;
    });
    if (!a.has("all-states") && static_cast<int>(cands.size()) > max_states) cands.resize(max_states);
    std::unique_ptr<std::ofstream> out;
    if (a.has("out")) out = std::make_unique<std::ofstream>(a.get("out", "counterfactual.jsonl"), std::ios::app);
    const auto& op = orig.passengers();
    std::unique_ptr<std::ofstream> observation_out;
    if (a.has("branch-observations")) observation_out = std::make_unique<std::ofstream>(a.get("branch-observations", "observations.jsonl"), std::ios::app);
    std::set<std::tuple<double,int,int>> observed_branches;
    for (auto& st : cands) {
        double t_b = st.flight_t0;
        HallKey call{st.floor, st.dir};
        std::ostringstream o;
        o << std::setprecision(10) << "{\"profile\":\"" << profile << "\",\"floors\":" << r.floors << ",\"cars\":" << r.cars << ",\"rate\":" << r.rate << ",\"seed\":" << r.seed
          << ",\"stop_t\":" << st.t0 << ",\"branch_t\":" << t_b << ",\"car\":" << st.car << ",\"floor\":" << st.floor << ",\"continuing\":" << st.continuing
          << ",\"stop_duration\":" << st.duration << ",\"stop_delay_ps\":" << st.duration * st.continuing;

        SimOptions sb = so; sb.stop_log = bool(observation_out);
        Simulation base(b, r.cars, make_dispatcher("paper2017_ls_lk"), passengers, nullptr, t_b, sb);
        base.run();
        double elapsed = 0, ac = 0;
        bool in_flight = base.car_in_flight_to(st.car, st.floor, elapsed, ac);
        auto cit = base.calls().find(call);
        bool assigned = cit != base.calls().end() && cit->second.assigned == st.car;
        bool before_decel = in_flight && elapsed < ac - 1e-9;
        bool locked = assigned && base.call_locked_by_theta(call);
        bool other_car = false;
        for (auto& c : base.cars()) if (c.id != st.car && c.load() < b.capacity) other_car = true;
        std::string reason;
        if (!in_flight) reason = "car not in flight to the floor at the branch";
        else if (!assigned) reason = "call not assigned to the car at the branch";
        else if (!before_decel) reason = "deceleration already started";
        else if (locked) reason = "call theta-locked at the branch";
        o << ",\"feasible\":" << (reason.empty() ? "true" : "false") << ",\"reason\":\"" << reason << "\",\"other_car\":" << (other_car ? "true" : "false")
          << ",\"elapsed\":" << elapsed << ",\"accel_cruise\":" << ac;
        if (!reason.empty()) { o << "}\n"; if (out) *out << o.str(); std::cerr << o.str(); continue; }
        if (observation_out && observed_branches.emplace(t_b, st.car, st.floor).second) {
            std::vector<CarView> observed_cars;
            for (const auto& car : base.cars()) observed_cars.push_back(base.view(car,t_b));
            std::vector<ObservedCall> observed_calls;
            for (const auto& [k,state] : base.calls()) {
                ObservedCall oc;
                oc.view.key=k; oc.view.registered=state.registered; oc.view.assigned=state.assigned;
                oc.view.close_to_service=base.call_locked_by_theta(k);
                for (const auto& stop : base.stop_log())
                    if (stop.floor==k.floor && stop.dir==k.dir && stop.t0>=state.registered-1e-9)
                        oc.prior_service_possible=true;
                for (const auto& car : base.cars())
                    if (car.stop_t0>=state.registered-1e-9 && car.stop_t0>=0 && car.floor==k.floor && car.stop_dir==k.dir)
                        oc.prior_service_possible=true;
                observed_calls.push_back(oc);
            }
            *observation_out << std::setprecision(10) << "{\"floors\":" << r.floors << ",\"cars\":" << r.cars
                << ",\"rate\":" << r.rate << ",\"seed\":" << r.seed << ",\"branch_t\":" << t_b
                << ",\"car\":" << st.car << ",\"floor\":" << st.floor << ",\"observed\":"
                << observation_json(b,t_b,observed_cars,observed_calls) << "}\n";
        }

        int p_on = static_cast<int>(base.cars()[st.car].onboard.size());
        int m_wait = 0; { auto w = base.waiting().find(call); if (w != base.waiting().end()) m_wait = static_cast<int>(w->second.size()); }
        CostOptions raw;
        CarView vA = base.view(base.cars()[st.car], base.now());
        vA.car_calls = base.cars()[st.car].car_call_counts();
        double tA = unary(b, vA, call, raw);

        std::map<int, int> dcount; if (auto w = base.waiting().find(call); w != base.waiting().end()) for (int idx : w->second) ++dcount[base.passengers()[idx].dest];
        int mdest = -1, mbest = -1; for (auto& [d, n] : dcount) if (n > mbest) { mbest = n; mdest = d; }
        double p_delta = 0.0;
        {

            std::vector<VCall> with{VCall{call, mdest, m_wait, 0}};
            PathResult rw = run_path_full(b, vA, with), ro = run_path_full(b, vA, {});
            for (auto& [g, cnt] : vA.car_calls) {
                double tw = rw.alight.count(g) ? rw.alight.at(g) : 1e6, to = ro.alight.count(g) ? ro.alight.at(g) : 1e6;
                p_delta += cnt * (tw - to);
            }
        }
        o << ",\"p\":" << p_on << ",\"m\":" << m_wait << ",\"tA\":" << tA << ",\"p_delta_A\":" << p_delta << ",\"room_A\":" << b.capacity - p_on;

        std::vector<int> cohort(passengers.size(), 3);
        for (int idx : base.cars()[st.car].onboard) cohort[idx] = 0;
        if (auto w = base.waiting().find(call); w != base.waiting().end()) for (int idx : w->second) cohort[idx] = 1;
        int nG[4] = {0, 0, 0, 0};
        for (size_t i = 0; i < passengers.size(); ++i) {
            const auto& bp = base.passengers()[i];
            if (cohort[i] == 3 && bp.arrival <= t_b) { cohort[i] = 2; if (bp.alighted < 0) ++nG[2]; }
            else if (cohort[i] != 2) ++nG[cohort[i]];
        }

        auto run_branch = [&](int mode) {
            Simulation s2(base, make_dispatcher("paper2017_ls_lk"));
            s2.set_max_time(4 * 3600.0);
            if (no_future) s2.drop_arrivals_after(t_b);
            if (branch_fixed_exclusive) {
                s2.set_fixed_assignment(true);
                s2.set_assigned_only_boarding(true);
            }
            if (mode > 0) {
                SimOptions::Intervention iv; iv.mode = mode == 1 ? SimOptions::Intervention::REASSIGN : SimOptions::Intervention::LATER;
                iv.call = call; iv.car = st.car; iv.fix_others = fix_others;
                if (mode == 1) iv.to_car = branch_to_car;
                s2.apply_intervention(iv);
            }
            s2.run();
            return s2;
        };
        Simulation none = run_branch(0);

        double worst = 0.0; int unserved = 0;
        for (size_t i = 0; i < passengers.size(); ++i) {
            const auto& p0 = op[i]; const auto& p1 = none.passengers()[i];
            if (no_future && p0.arrival > t_b) continue;
            if (p1.boarded < 0 || p1.alighted < 0) { ++unserved; continue; }
            worst = std::max(worst, std::max(std::abs(p0.boarded - p1.boarded), std::abs(p0.alighted - p1.alighted)));
        }
        o << ",\"no_future\":" << (no_future ? "true" : "false") << ",\"fix_others\":" << (fix_others ? "true" : "false");
        o << ",\"branch_fixed_exclusive\":" << (branch_fixed_exclusive ? "true" : "false");
        o << ",\"requested_car\":" << branch_to_car;
        if (!no_future && (worst >= 1e-9 || unserved > 0))
            throw std::runtime_error("counterfactual: the no-intervention branch does not reproduce the original run (worst diff " + std::to_string(worst) + ", unserved " + std::to_string(unserved) + ")");
        if (no_future && unserved > 0) throw std::runtime_error("counterfactual: drain branch left passengers unserved");
        o << ",\"none_matches_original\":" << (no_future ? "null" : "true") << ",\"none_worst_diff\":" << worst;
        o << ",\"n_cohort\":[" << nG[0] << "," << nG[1] << "," << nG[2] << "," << nG[3] << "]";
        for (int mode : {1, 2}) {
            if (mode == 1 && ((branch_to_car < 0 && !other_car) || branch_to_car == st.car)) { o << ",\"" << (mode == 1 ? "A" : "B") << "\":{\"feasible\":false}"; continue; }
            Simulation br = run_branch(mode);
            double dw[4] = {0, 0, 0, 0}, dr[4] = {0, 0, 0, 0}, ds[4] = {0, 0, 0, 0}, tot_w = 0, tot_r = 0, tot_s = 0;
            int uns = 0;
            for (size_t i = 0; i < passengers.size(); ++i) {
                const auto& p0 = none.passengers()[i]; const auto& p1 = br.passengers()[i];
                if (no_future && p0.arrival > t_b) continue;
                if (p1.boarded < 0 || p1.alighted < 0 || p0.boarded < 0 || p0.alighted < 0) { ++uns; continue; }
                int g = cohort[i];
                double ddw = (p1.boarded - p1.arrival) - (p0.boarded - p0.arrival), ddr = (p1.alighted - p1.boarded) - (p0.alighted - p0.boarded);
                dw[g] += ddw; dr[g] += ddr; ds[g] += ddw + ddr;
                tot_w += ddw; tot_r += ddr; tot_s += ddw + ddr;
            }
            if (uns > 0) throw std::runtime_error("counterfactual: an intervention branch left passengers unserved");
            if (std::abs(dw[0] + dw[1] + dw[2] + dw[3] - tot_w) > 1e-6 || std::abs(ds[0] + ds[1] + ds[2] + ds[3] - tot_s) > 1e-6)
                throw std::logic_error("counterfactual: cohort sums do not add up to the totals");
            Metrics m0 = none.metrics(), m1 = br.metrics();
            const auto& iv = br.intervention();
            if (iv.forbidden_violations > 0) throw std::logic_error("counterfactual: excluded car received the call");
            if (mode == 1) {

                int alt = -1; double first = 1e300;
                for (size_t i = 0; i < passengers.size(); ++i) if (cohort[i] == 1 && br.passengers()[i].boarded >= 0 && br.passengers()[i].boarded < first) { first = br.passengers()[i].boarded; alt = br.passengers()[i].car; }
                double tB = -1; int alt_load = -1, alt_calls = -1;
                if (alt >= 0) {
                    CarView vB = base.view(base.cars()[alt], base.now());
                    vB.car_calls = base.cars()[alt].car_call_counts();
                    tB = unary(b, vB, call, raw);
                    alt_load = static_cast<int>(base.cars()[alt].onboard.size());
                    alt_calls = static_cast<int>(base.cars()[alt].assigned.size());
                }
                double pred = m_wait * (tB - tA) - p_delta;
                o << ",\"alt_car\":" << alt << ",\"tB\":" << tB << ",\"alt_load\":" << alt_load << ",\"alt_calls\":" << alt_calls << ",\"pred_A\":" << pred;
            }
            o << ",\"" << (mode == 1 ? "A" : "B") << "\":{\"feasible\":true,\"unserved\":" << uns << ",\"released\":" << (iv.released ? "true" : "false")
              << ",\"d_wait\":[" << dw[0] << "," << dw[1] << "," << dw[2] << "," << dw[3] << "],\"d_ride\":[" << dr[0] << "," << dr[1] << "," << dr[2] << "," << dr[3] << "]"
              << ",\"d_system\":[" << ds[0] << "," << ds[1] << "," << ds[2] << "," << ds[3] << "],\"d_wait_total\":" << dw[0] + dw[1] + dw[2] + dw[3]
              << ",\"d_ride_total\":" << dr[0] + dr[1] + dr[2] + dr[3] << ",\"d_system_total\":" << ds[0] + ds[1] + ds[2] + ds[3]
              << ",\"awt_none\":" << m0.awt << ",\"awt\":" << m1.awt << ",\"att_none\":" << m0.att << ",\"att\":" << m1.att << "}";
        }
        o << "}\n";
        if (out) *out << o.str();
        std::cerr << o.str();
    }
    return 0;
}

std::string route_line(const Building& b, const Simulation& base, int c, int f, int h) {
    CarView v = base.view(base.cars()[c], base.now());
    auto direct = [&](int floor) -> double {
        if (!v.moving) return v.release + b.kin.travel(base.cars()[c].floor, floor);
        double dist = (b.pos(floor) - v.x) * v.dir;
        if (dist < v.v * v.v / (2.0 * b.kin.a_max) - 1e-9) return std::nan("");
        return b.kin.time_from_state(dist, v.v);
    };
    std::ostringstream o;
    o << std::setprecision(17) << "  routeB:     x=" << v.x << " v=" << v.v << " dir=" << v.dir << " moving=" << (v.moving ? 1 : 0) << " floor=" << base.cars()[c].floor
      << " release=" << v.release << " tB_f=" << direct(f) << " tB_h=" << direct(h) << " tB_0=" << direct(0)
      << " travel_fh=" << b.kin.travel(f, h) << " travel_h0=" << b.kin.travel(h, 0) << " travel_0h=" << b.kin.travel(0, h) << " h=" << h << "\n";
    return o.str();
}

int restricted_case(const Args& a, double& pred_out, double& actual_out, bool print) {
    int F = a.integer("floors", 10), p = a.integer("p", 6), m = a.integer("m", 2), f = a.integer("f", 6), bfl = a.integer("b", 0);
    int a_call = a.integer("a-call", -1), b_call = a.integer("b-call", -1);

    int h_call = a.integer("h", -1), m2 = a.integer("m2", 1);

    int q_on = a.integer("q", 0), bq = a.integer("bq", F - 2);
    double bdelay = a.num("bdelay", 0.0);
    bool mixed = a.has("mixed");
    bool overflow = a.has("overflow");
    bool basic = a_call < 0 && b_call < 0 && h_call < 0 && !mixed && !overflow;
    bool strict = a.has("strict-boarding");
    Building b = building_from(a, F);

    std::vector<Passenger> ps;
    int top = F - 1;

    double dep_A = b.kin.travel(0, top) + b.doors.dwell(p, 0), dep_B = b.kin.travel(0, bq) + b.doors.dwell(q_on, 0);
    double dep = q_on > 0 ? std::max(dep_A, dep_B) : dep_A;
    double t_a = dep - dep_A, tq = dep - dep_B + bdelay;
    for (int i = 0; i < p; ++i) { Passenger q; q.arrival = t_a; q.origin = top; q.dest = 0; ps.push_back(q); }
    double t_pos = 0.5;
    if (q_on > 0) { for (int i = 0; i < q_on; ++i) { Passenger q; q.arrival = tq; q.origin = bq; q.dest = 0; ps.push_back(q); } }
    else if (bfl > 0) { Passenger q; q.arrival = t_pos; q.origin = 0; q.dest = bfl; ps.push_back(q); }

    double T = dep + 1.0;
    if (a_call >= 0) { Passenger q; q.arrival = T - 0.5; q.origin = a_call; q.dest = 0; ps.push_back(q); }
    if (b_call >= 0) { Passenger q; q.arrival = T - 0.25; q.origin = b_call; q.dest = F - 1; ps.push_back(q); }
    if (h_call >= 0) for (int i = 0; i < m2; ++i) { Passenger q; q.arrival = T - 0.25; q.origin = h_call; q.dest = 0; ps.push_back(q); }
    for (int i = 0; i < m; ++i) { Passenger q; q.arrival = T; q.origin = f; q.dest = (mixed && (i % 2 == 1)) ? 1 : 0; ps.push_back(q); }
    SimOptions so = options_from(a);
    so.stop_log = true;

    so.fixed_assignment = !a.has("free-others");
    so.forced_assignment[HallKey{top, DOWN}] = 0;
    if (q_on > 0) so.forced_assignment[HallKey{bq, DOWN}] = 1;
    else if (bfl > 0) so.forced_assignment[HallKey{0, UP}] = 1;
    if (a_call >= 0) so.forced_assignment[HallKey{a_call, DOWN}] = 0;
    if (b_call >= 0) so.forced_assignment[HallKey{b_call, UP}] = 1;
    if (h_call >= 0) so.forced_assignment[HallKey{h_call, DOWN}] = 1;
    so.forced_assignment[HallKey{f, DOWN}] = 0;
    Simulation orig(b, 2, make_dispatcher("paper2017_ls_lk"), ps, nullptr, 4 * 3600.0, so);
    orig.run();

    const Simulation::StopRecord* st = nullptr;
    for (auto& r : orig.stop_log()) if (r.car == 0 && r.floor == f && r.dir == DOWN && r.boarded > 0) { st = &r; break; }
    if (!st) { if (print) std::cerr << "scenario failed: car 0 never picked up at floor " << f << " (call assigned elsewhere?)\n"; return 1; }
    if ((p + m > b.capacity || q_on + m > b.capacity) && !overflow) { if (print) std::cerr << "basic model precondition violated: p + m or q + m > capacity (use --overflow for the extension)\n"; return 1; }
    double t_b = st->flight_t0;
    SimOptions sb = so; sb.stop_log = false;
    Simulation base(b, 2, make_dispatcher("paper2017_ls_lk"), ps, nullptr, t_b, sb);
    base.run();
    HallKey call{f, DOWN};
    auto cit = base.calls().find(call);
    double el = 0, ac = 0;
    bool ok = base.car_in_flight_to(0, f, el, ac) && cit != base.calls().end() && cit->second.assigned == 0 && el < ac - 1e-9 && !base.call_locked_by_theta(call);
    if (!ok) { if (print) std::cerr << "branch infeasible (in flight " << base.car_in_flight_to(0, f, el, ac) << ", assigned " << (cit != base.calls().end() ? cit->second.assigned : -9) << ", elapsed " << el << "/" << ac << ", locked " << base.call_locked_by_theta(call) << ")\n"; return 1; }

    CarView vA = base.view(base.cars()[0], base.now());
    double dist_f = std::abs(b.pos(f) - vA.x), dist_0 = std::abs(b.pos(0) - vA.x);
    double tA = b.kin.time_from_state(dist_f, vA.v);
    double tA0 = b.kin.time_from_state(dist_0, vA.v);

    CarView vB = base.view(base.cars()[1], base.now());
    int q_b = static_cast<int>(base.cars()[1].onboard.size());
    double tB, tB0;
    if (vB.moving && q_b > 0) {

        if (!(vB.dir == DOWN && vB.x > b.pos(f) + 1e-9)) { if (print) std::cerr << "branch infeasible: car B moving but not descending above f (x=" << vB.x << ", dir " << vB.dir << ")\n"; return 1; }
        double eB = 0, aB = 0;
        if (!base.car_in_flight_to(1, 0, eB, aB) || eB >= aB - 1e-9) { if (print) std::cerr << "branch infeasible: car B not in a flight to the lobby before its deceleration\n"; return 1; }
        tB = b.kin.time_from_state(std::abs(b.pos(f) - vB.x), vB.v);
        tB0 = b.kin.time_from_state(std::abs(b.pos(0) - vB.x), vB.v);
    } else {
        tB = b.kin.travel(base.cars()[1].floor, f) + vB.release;
        tB0 = b.kin.travel(base.cars()[1].floor, 0) + vB.release;
        if (q_b > 0 && base.cars()[1].floor < f) { if (print) std::cerr << "branch infeasible: car B holds passengers below f\n"; return 1; }
    }
    if (q_b != q_on) { if (print) std::cerr << "scenario failed: car B carries " << q_b << " passengers at the branch, expected " << q_on << "\n"; return 1; }
    double delta_A = b.doors.dwell(m, 0) + tA + b.kin.travel(f, 0) - tA0;
    double delta_B = b.doors.dwell(m, 0) + tB + b.kin.travel(f, 0) - tB0;
    int p_on = static_cast<int>(base.cars()[0].onboard.size());
    double pred_G1 = -p_on * delta_A, pred_G2 = m * (tB - tA), pred_G3 = q_b * delta_B, pred = pred_G1 + pred_G2 + pred_G3;

    auto run_branch = [&](bool intervene) {
        Simulation s2(base, make_dispatcher("paper2017_ls_lk"));
        s2.set_max_time(4 * 3600.0);
        s2.drop_arrivals_after(t_b);
        s2.set_stop_log(a.has("trace-stops"));
        if (strict) s2.set_assigned_only_boarding(true);

        if (intervene) { SimOptions::Intervention iv; iv.mode = SimOptions::Intervention::REASSIGN; iv.call = call; iv.car = 0; iv.to_car = 1; s2.apply_intervention(iv); }
        s2.run();
        return s2;
    };
    Simulation none = run_branch(false), alt = run_branch(true);
    if (print && a.has("trace-stops")) {
        for (auto* sim : {&none, &alt}) {
            std::cout << (sim == &none ? "  none:" : "  alt: ");
            for (auto& r : sim->stop_log()) std::cout << " [car" << r.car << " f" << r.floor << (r.dir > 0 ? "^" : "v") << " t" << std::fixed << std::setprecision(1) << r.t0 << " +" << r.boarded << "/-" << r.alight << "]";
            std::cout << "\n";
        }
        std::cout << "  passengers (origin->dest arrival | none boarded/alighted car | alt boarded/alighted car):\n";
        for (size_t i = 0; i < ps.size(); ++i) {
            const auto& p0 = none.passengers()[i]; const auto& p1 = alt.passengers()[i];
            std::cout << "    " << p0.origin << "->" << p0.dest << " @" << p0.arrival << " | " << p0.boarded << "/" << p0.alighted << " car" << p0.car << " | " << p1.boarded << "/" << p1.alighted << " car" << p1.car << "\n";
        }
    }
    double dG[4] = {0, 0, 0, 0}, dGw[4] = {0, 0, 0, 0}, dGr[4] = {0, 0, 0, 0};
    for (size_t i = 0; i < ps.size(); ++i) {
        const auto& p0 = none.passengers()[i]; const auto& p1 = alt.passengers()[i];
        if (p0.arrival > t_b) continue;
        if (p0.boarded < 0 || p1.boarded < 0 || p0.alighted < 0 || p1.alighted < 0) { if (print) std::cerr << "unserved in a branch\n"; return 1; }
        int g = 2;
        if (p0.car == 0 && p0.boarded <= t_b) g = 0;
        else if (p0.origin == f && p0.arrival == T) g = 1;
        double dw = (p1.boarded - p1.arrival) - (p0.boarded - p0.arrival), dr = (p1.alighted - p1.boarded) - (p0.alighted - p0.boarded);
        dGw[g] += dw; dGr[g] += dr; dG[g] += dw + dr;
    }
    double total = dG[0] + dG[1] + dG[2];
    pred_out = pred; actual_out = total;
    if (print) std::cout << std::setprecision(6) << std::fixed
              << "state: p=" << p_on << " m=" << m << " f=" << f << " b=" << base.cars()[1].floor << " q=" << q_b << " branch t=" << t_b << " (A x=" << vA.x << " v=" << vA.v << "; B x=" << vB.x << " v=" << vB.v << ")"
              << (a_call >= 0 ? " +A-call@" + std::to_string(a_call) : "") << (b_call >= 0 ? " +B-call@" + std::to_string(b_call) : "")
              << (h_call >= 0 ? " +B-down-call@" + std::to_string(h_call) + "x" + std::to_string(m2) : "") << (mixed ? " +mixed" : "") << "\n"
              << "  prediction: tA=" << tA << " tB=" << tB << " delta_A=" << delta_A << " delta_B=" << delta_B << "  G1=-p*delta_A=" << pred_G1 << "  G2=m(tB-tA)=" << pred_G2 << "  G3=q*delta_B=" << pred_G3 << "  dJ=" << pred << "\n"
              << (h_call >= 0 ? route_line(b, base, 1, f, h_call) : std::string())
              << "  geometry:   tA0=" << tA0 << " tB0=" << tB0 << " travel_f0=" << b.kin.travel(f, 0) << " travel_0f=" << b.kin.travel(0, f) << " d0=" << b.doors.dwell(1, 0) - b.doors.transfer << " tau=" << b.doors.transfer << " capacity=" << b.capacity << "\n"
              << "  engine:     G1 (ride " << dGr[0] << ", wait " << dGw[0] << ") = " << dG[0] << "  G2 (wait " << dGw[1] << ", ride " << dGr[1] << ") = " << dG[1]
              << "  G3 = " << dG[2] << "  dJ=" << total << "\n"
              << "  |dJ - pred| = " << std::abs(total - pred) << "  |G1 - pred| = " << std::abs(dG[0] - pred_G1) << "  |G2 - pred| = " << std::abs(dG[1] - pred_G2) << "  |G3 - pred| = " << std::abs(dG[2] - pred_G3) << "\n";

    if (basic && (std::abs(total - pred) > 1e-6 || std::abs(dG[0] - pred_G1) > 1e-6 || std::abs(dG[1] - pred_G2) > 1e-6 || std::abs(dG[2] - pred_G3) > 1e-6)) {
        if (print) std::cerr << "BASIC IDENTITY FAILED\n";
        return 2;
    }
    return 0;
}

int cmd_census(const Args& a) {
    RunSpec r{a.integer("floors", 10), a.integer("cars", 3), a.integer("seed", 0), a.num("rate", 30.0), "paper2017_ls_lk"};
    std::string profile = a.get("profile", "downt");
    Building b = building_from(a, r.floors);
    unsigned long traffic_seed = static_cast<unsigned long>(r.seed) * 1000003UL + r.floors * 101UL + r.cars * 7UL + static_cast<unsigned long>(r.rate * 10);
    TrafficSpec spec = TrafficSpec::named(profile, r.floors, r.rate, a.num("population", 30.0), a.num("duration", 3600.0));
    auto passengers = profile == "interfloor" ? interfloor_traffic(r.floors, r.rate, a.num("population", 30.0), a.num("duration", 3600.0), traffic_seed)
                                              : profile_traffic(spec, traffic_seed);
    SimOptions so = options_from(a);
    if (!spec.phases.empty()) { so.win0 = spec.phases.back().t0; so.win1 = spec.phases.back().t1; }
    so.stop_log = true;
    Simulation orig(b, r.cars, make_dispatcher("paper2017_ls_lk"), passengers, nullptr, 4 * 3600.0, so);
    orig.run();
    std::vector<Simulation::StopRecord> cands;
    for (auto& st : orig.stop_log()) if (st.pickup_only && st.dir == DOWN && st.in_window && st.continuing > 0) cands.push_back(st);
    std::unique_ptr<std::ofstream> out;
    if (a.has("out")) out = std::make_unique<std::ofstream>(a.get("out", "census.jsonl"), std::ios::app);
    auto emit = [&](const std::string& line) { if (out) *out << line << "\n"; else std::cout << line << "\n"; };
    int n_feasible = 0, n_A_ok = 0, n_pairs = 0, n_states_with_B = 0;
    std::set<std::pair<double, int>> seen;
    for (auto& st : cands) {
        double t_b = st.flight_t0;
        if (!seen.insert({t_b, st.car}).second) continue;
        HallKey call{st.floor, st.dir};
        SimOptions sb = so; sb.stop_log = false;
        Simulation base(b, r.cars, make_dispatcher("paper2017_ls_lk"), passengers, nullptr, t_b, sb);
        base.run();
        double elapsed = 0, ac = 0;
        bool in_flight = base.car_in_flight_to(st.car, st.floor, elapsed, ac);
        auto cit = base.calls().find(call);
        bool assigned = cit != base.calls().end() && cit->second.assigned == st.car;
        if (!in_flight || !assigned || !(elapsed < ac - 1e-9) || base.call_locked_by_theta(call)) continue;
        ++n_feasible;
        const Car& A = base.cars()[st.car];
        CarView vA = base.view(A, base.now());
        int f = st.floor;

        bool A_lobby = true; for (int idx : A.onboard) if (base.passengers()[idx].dest != 0) A_lobby = false;
        bool A_single = A.assigned.size() == 1 && A.assigned.count(call);
        int p_on = A.load();
        int m_wait = 0, m_lobby = 0;
        if (auto w = base.waiting().find(call); w != base.waiting().end()) { m_wait = static_cast<int>(w->second.size()); for (int idx : w->second) if (base.passengers()[idx].dest == 0) ++m_lobby; }
        int unassigned_calls = 0; for (auto& [k, cs] : base.calls()) if (cs.assigned < 0) ++unassigned_calls;
        bool A_p_range = p_on >= 1 && p_on < b.capacity;
        bool A_ok = A_lobby && A_single && A_p_range;
        bool waiting_all_lobby = m_lobby == m_wait;
        if (A_ok) ++n_A_ok;
        double dist_f = std::abs(b.pos(f) - vA.x), dist_0 = std::abs(b.pos(0) - vA.x);
        double tA = b.kin.time_from_state(dist_f, vA.v), tA0 = b.kin.time_from_state(dist_0, vA.v);
        std::string head;
        {
            std::ostringstream o;
            o << std::setprecision(17) << "{\"profile\":\"" << profile << "\",\"floors\":" << r.floors << ",\"cars\":" << r.cars << ",\"rate\":" << r.rate << ",\"seed\":" << r.seed
              << ",\"branch_t\":" << t_b << ",\"car\":" << st.car << ",\"floor\":" << f << ",\"p\":" << p_on << ",\"m\":" << m_wait << ",\"m_lobby\":" << m_lobby
              << ",\"A_lobby\":" << (A_lobby ? "true" : "false") << ",\"A_single_call\":" << (A_single ? "true" : "false") << ",\"A_p_range\":" << (A_p_range ? "true" : "false")
              << ",\"A_ok\":" << (A_ok ? "true" : "false") << ",\"waiting_all_lobby\":" << (waiting_all_lobby ? "true" : "false")
              << ",\"unassigned_calls\":" << unassigned_calls << ",\"tA\":" << tA << ",\"tA0\":" << tA0 << ",\"travel_f0\":" << b.kin.travel(f, 0) << ",\"travel_0f\":" << b.kin.travel(0, f)
              << ",\"d0\":" << b.doors.open + b.doors.close << ",\"tau\":" << b.doors.transfer << ",\"capacity\":" << b.capacity;
            head = o.str();
        }

        std::vector<int> cohort(passengers.size(), -1);
        for (int idx : A.onboard) cohort[idx] = 0;
        if (auto w = base.waiting().find(call); w != base.waiting().end()) for (int idx : w->second) cohort[idx] = 1;
        for (size_t i = 0; i < passengers.size(); ++i) if (cohort[i] < 0 && base.passengers()[i].arrival <= t_b && base.passengers()[i].alighted < 0) cohort[i] = 3;
        bool any_B = false;
        for (auto& B : base.cars()) {
            if (B.id == st.car) continue;
            CarView vB = base.view(B, base.now());
            int q_b = B.load();
            bool B_lobby = true; for (int idx : B.onboard) if (base.passengers()[idx].dest != 0) B_lobby = false;
            bool B_no_calls = B.assigned.empty();
            bool B_not_full = q_b < b.capacity;

            std::string geom = "other";
            double tB = -1, tB0 = -1;
            if (q_b == 0 && !vB.moving) { geom = "empty_idle"; tB = b.kin.travel(B.floor, f) + vB.release; tB0 = b.kin.travel(B.floor, 0) + vB.release; }
            else if (q_b > 0 && vB.moving) {
                double eB = 0, aB = 0;
                double dist_fB = vB.x - b.pos(f), need = vB.v * vB.v / (2.0 * b.kin.a_max);
                if (vB.dir == DOWN && dist_fB > need + 1e-9 && base.car_in_flight_to(B.id, 0, eB, aB) && eB < aB - 1e-9) {
                    geom = "loaded_descending"; tB = b.kin.time_from_state(dist_fB, vB.v); tB0 = b.kin.time_from_state(std::abs(b.pos(0) - vB.x), vB.v);
                }
            } else if (q_b > 0 && !vB.moving && B.floor >= f) { geom = "loaded_at_rest_above"; tB = b.kin.travel(B.floor, f) + vB.release; tB0 = b.kin.travel(B.floor, 0) + vB.release; }
            bool B_geom = geom != "other";
            bool B_ok = B_no_calls && B_lobby && B_not_full && B_geom;
            std::string type = B_ok ? geom : "other";
            std::string why;
            if (!B_no_calls) why = "B has assigned calls";
            else if (!B_lobby) why = "B carries non-lobby passengers";
            else if (!B_not_full) why = "B full";
            else if (!B_geom) why = q_b == 0 ? "B empty but moving without calls" : vB.moving ? "B loaded but not descending above f before deceleration / cannot stop" : "B loaded and at rest below f";
            std::ostringstream o;
            o << head << std::setprecision(17) << ",\"B\":" << B.id << ",\"q\":" << q_b << ",\"B_type\":\"" << type << "\",\"B_geom\":\"" << geom << "\",\"B_reason\":\"" << why << "\""
              << ",\"B_no_calls\":" << (B_no_calls ? "true" : "false") << ",\"B_lobby\":" << (B_lobby ? "true" : "false") << ",\"B_not_full\":" << (B_not_full ? "true" : "false")
              << ",\"B_geom_ok\":" << (B_geom ? "true" : "false") << ",\"B_ok\":" << (B_ok ? "true" : "false");

            bool pair_ok = A_ok && B_ok;
            o << ",\"inside_model\":" << ((pair_ok && waiting_all_lobby) ? "true" : "false");
            if (pair_ok) {
                any_B = true; ++n_pairs;
                o << ",\"tB\":" << tB << ",\"tB0\":" << tB0;

                std::vector<int> coh = cohort;
                for (int idx : B.onboard) coh[idx] = 2;
                auto run_branch = [&](bool intervene) {
                    Simulation s2(base, make_dispatcher("paper2017_ls_lk"));
                    s2.set_max_time(4 * 3600.0);
                    s2.drop_arrivals_after(t_b);
                    s2.set_fixed_assignment(true);
                    s2.set_assigned_only_boarding(true);
                    if (intervene) { SimOptions::Intervention iv; iv.mode = SimOptions::Intervention::REASSIGN; iv.call = call; iv.car = st.car; iv.to_car = B.id; s2.apply_intervention(iv); }
                    s2.run();
                    return s2;
                };
                Simulation none = run_branch(false), alt = run_branch(true);
                double dG[4] = {0, 0, 0, 0}; int uns = 0;
                for (size_t i = 0; i < passengers.size(); ++i) {
                    if (coh[i] < 0) continue;
                    const auto& p0 = none.passengers()[i]; const auto& p1 = alt.passengers()[i];
                    if (p0.alighted < 0 || p1.alighted < 0) { ++uns; continue; }
                    dG[coh[i]] += p1.alighted - p0.alighted;
                }
                o << ",\"unserved\":" << uns << ",\"dJ_G1\":" << dG[0] << ",\"dJ_G2\":" << dG[1] << ",\"dJ_G3\":" << dG[2] << ",\"dJ_other\":" << dG[3]
                  << ",\"dJ_model_cohorts\":" << dG[0] + dG[1] + dG[2];
            }
            o << "}";
            emit(o.str());
        }
        if (any_B) ++n_states_with_B;
    }
    std::cerr << "census: " << cands.size() << " candidate stops, " << n_feasible << " feasible branch states, A inside the model in " << n_A_ok
              << ", states with at least one qualifying alternative car " << n_states_with_B << ", (state, B) pairs run " << n_pairs << "\n";
    return 0;
}

int cmd_restricted(const Args& a) {
    double pred = 0, actual = 0;
    return restricted_case(a, pred, actual, true);
}

int cmd_selftest(const Args& a) {
    int fails = 0;
    auto check = [&](bool ok, const std::string& what) {
        if (!ok) { ++fails; std::cerr << "FAIL: " << what << "\n"; }
    };
    Building b = building_from(a, 12);

    for (int d = 1; d < 12; ++d) {
        double dist = d * b.kin.floor_height;
        double t = b.kin.travel(0, d);
        double v = b.kin.v_max, acc = b.kin.a_max;
        double expect = dist >= v * v / acc ? dist / v + v / acc : 2.0 * std::sqrt(dist / acc);
        check(std::abs(t - expect - b.kin.settle) < 1e-9, "travel time formula d=" + std::to_string(d));
        auto p = b.kin.profile(dist);
        check(std::abs(p.position(p.moving_time()) - dist) < 1e-6, "profile reaches distance");
        check(std::abs(p.velocity(p.moving_time())) < 1e-9, "profile stops");
    }
    {
        auto p = b.kin.profile(8 * b.kin.floor_height);
        for (double tq = 0.5; tq < p.moving_time(); tq += 0.5) {
            double x = p.position(tq), v = p.velocity(tq);
            double rem = b.kin.time_from_state(p.d - x, v);
            check(std::abs(rem - (p.duration() - tq)) < 1e-6, "time_from_state consistent at t=" + std::to_string(tq));
        }
    }

    std::mt19937_64 rng(7);
    int negative = 0, tested = 0;
    for (int it = 0; it < 2000; ++it) {
        CarView car;
        car.committed = rng() % 12;
        car.x = b.pos(car.committed);
        car.dir = (rng() % 3 == 0) ? 0 : (rng() % 2 ? UP : DOWN);
        if (car.dir != 0 && rng() % 2) {
            int f = rng() % 12;
            if ((f - car.committed) * car.dir > 0) car.car_calls[f] = 1;
        }
        HallKey i{static_cast<int>(rng() % 12), rng() % 2 ? UP : DOWN};
        HallKey j{static_cast<int>(rng() % 12), rng() % 2 ? UP : DOWN};
        if ((i.dir == UP && i.floor == 11) || (i.dir == DOWN && i.floor == 0)) continue;
        if ((j.dir == UP && j.floor == 11) || (j.dir == DOWN && j.floor == 0)) continue;
        if (i == j) continue;
        CostOptions raw;
        double wi = unary(b, car, i, raw), wj = unary(b, car, j, raw);
        double w = pairwise(b, car, i, j, wi, wj);
        ++tested;
        if (w < -1e-6) ++negative;
    }
    check(negative == 0, "Lemma 1: negative pairwise terms");
    std::cerr << "Lemma 1 check: " << negative << " negative pairwise terms in " << tested << " random states\n";

    {
        std::mt19937_64 g(11);
        int checked = 0;
        for (int it = 0; it < 200; ++it) {
            int n = 2 + g() % 3, C = 2 + g() % 2;
            std::vector<std::vector<double>> u(n, std::vector<double>(C));
            std::vector<std::vector<std::vector<double>>> w(n, std::vector<std::vector<double>>(n, std::vector<double>(C, 0.0)));
            for (int i = 0; i < n; ++i) for (int c = 0; c < C; ++c) u[i][c] = 5.0 + (g() % 400) / 10.0;
            for (int i = 0; i < n; ++i) for (int j = i + 1; j < n; ++j) for (int c = 0; c < C; ++c) w[i][j][c] = w[j][i][c] = (g() % 300) / 10.0;
            int order_k = (it % 2) ? 3 : 0; double order_penalty = (it % 2) ? 5.0 : 0.0;
            std::vector<double> p(n);
            for (int i = 0; i < n; ++i) { double best = -1e300; for (int c = 0; c < C; ++c) { double s2 = u[i][c]; for (int j = 0; j < n; ++j) if (j != i) s2 += w[i][j][c]; best = std::max(best, s2); } p[i] = best; }

            std::vector<int> car_of(n, -1);
            std::vector<std::vector<int>> S(C);
            for (int step = 0; step < n; ++step) {
                double best_gain = -1e300; int bi = -1, bc = -1;
                for (int i = 0; i < n; ++i) { if (car_of[i] >= 0) continue; for (int c = 0; c < C; ++c) {
                    double gain = p[i] - u[i][c];
                    for (int j : S[c]) gain -= w[i][j][c];
                    if (order_k >= 2) { int m = static_cast<int>(S[c].size()); double comb = 1.0; for (int q = 1; q <= order_k - 1; ++q) comb = comb * (m - (order_k - 1) + q) / q; if (m >= order_k - 1) gain -= order_penalty * comb; }
                    if (gain > best_gain + 1e-12) { best_gain = gain; bi = i; bc = c; } } }
                std::vector<int> after = car_of; after[bi] = bc;
                double brute = f_value(after, u, w, p, C, order_k, order_penalty) - f_value(car_of, u, w, p, C, order_k, order_penalty);
                check(std::abs(brute - best_gain) < 1e-9, "greedy incremental gain equals f difference");
                car_of[bi] = bc; S[bc].push_back(bi); ++checked;
            }
            check(std::all_of(car_of.begin(), car_of.end(), [](int c) { return c >= 0; }), "greedy assigns every call once (matroid base)");
        }
        std::cerr << "greedy gain checks: " << checked << "\n";
    }

    {
        std::mt19937_64 g(23);
        int checked = 0, ls_better = 0, opt_better_than_ls = 0;
        for (int it = 0; it < 300; ++it) {
            int n = 2 + g() % 5, C = 2 + g() % 3;
            std::vector<std::vector<double>> u(n, std::vector<double>(C));
            std::vector<std::vector<std::vector<double>>> w(n, std::vector<std::vector<double>>(n, std::vector<double>(C, 0.0)));
            for (int i = 0; i < n; ++i) for (int c = 0; c < C; ++c) u[i][c] = 5.0 + (g() % 400) / 10.0;
            for (int i = 0; i < n; ++i) for (int j = i + 1; j < n; ++j) for (int c = 0; c < C; ++c) w[i][j][c] = w[j][i][c] = (g() % 300) / 10.0;
            CostOptions o; o.search = CostOptions::LS;
            std::vector<double> p(n);
            for (int i = 0; i < n; ++i) { double best = -1e300; for (int c = 0; c < C; ++c) { double s2 = u[i][c]; for (int j = 0; j < n; ++j) if (j != i) s2 += w[i][j][c]; best = std::max(best, s2); } p[i] = best; }
            std::vector<int> greedy_x(n, -1); std::vector<std::vector<int>> S(C);
            for (int step = 0; step < n; ++step) { double bg = -1e300; int bi = -1, bc = -1;
                for (int i = 0; i < n; ++i) { if (greedy_x[i] >= 0) continue; for (int c = 0; c < C; ++c) { double gain = p[i] - u[i][c]; for (int j : S[c]) gain -= w[i][j][c]; if (gain > bg + 1e-12) { bg = gain; bi = i; bc = c; } } }
                greedy_x[bi] = bc; S[bc].push_back(bi); }
            std::vector<int> ls_x = greedy_x, opt_x = greedy_x;
            local_search_public(ls_x, u, w, C, o);
            long states = exhaustive_public(opt_x, u, w, C, o);
            double gg = g_of_public(greedy_x, u, w, C, o), gl = g_of_public(ls_x, u, w, C, o), go = g_of_public(opt_x, u, w, C, o);
            check(gl <= gg + 1e-9, "local search never worsens g");
            if (states > 0) check(go <= gl + 1e-9, "exhaustive is at least as good as local search");
            check(std::all_of(ls_x.begin(), ls_x.end(), [&](int c) { return c >= 0 && c < C; }), "LS keeps a complete assignment");
            ++checked; ls_better += gl < gg - 1e-9; opt_better_than_ls += states > 0 && go < gl - 1e-9;
        }
        std::cerr << "search checks: " << checked << " instances; LS improved greedy in " << ls_better << ", exhaustive beat LS in " << opt_better_than_ls << "\n";
    }

    {
        Building bb = building_from(a, 6);
        for (double when : {0.5, bb.doors.open + bb.doors.transfer + 1.0}) {
            double t1 = bb.kin.travel(0, 2);
            std::vector<Passenger> ps;
            Passenger p0; p0.arrival = 0.0; p0.origin = 2; p0.dest = 4; ps.push_back(p0);
            Passenger p1; p1.arrival = t1 + when; p1.origin = 2; p1.dest = 5; ps.push_back(p1);
            SimOptions so; so.check_invariants = true;
            Simulation sim(bb, 1, make_dispatcher("pairwise"), ps, nullptr, 3600.0, so);
            sim.run();
            const auto& P = sim.passengers();
            double open_until = t1 + bb.doors.open + bb.doors.transfer;
            if (t1 + when < open_until) check(std::abs(P[1].boarded - P[1].arrival) < 1e-9, "arrival with doors open boards immediately");
            else check(P[1].boarded > P[1].arrival + bb.doors.open - 1e-9, "arrival while closing waits for a new door cycle");
            check(P[0].alighted > 0 && P[1].alighted > 0, "door-contract run completes");
        }
    }

    {
        Building bb = building_from(a, 6);
        struct Probe : Dispatcher {
            double seen = -1.0, release = -1.0;
            std::string name() const override { return "probe"; }
            std::map<HallKey, int> decide(const Building& b2, const std::vector<CarView>& cs, const std::vector<CallView>& hs,
                                          const HallKey& nc, DecisionTrace*) override {
                std::map<HallKey, int> out;
                for (auto& h : hs) out[h.key] = 0;
                if (!(nc == HallKey{3, UP})) return out;
                CostOptions raw;
                seen = unary(b2, cs[0], nc, raw);
                release = cs[0].release;
                return out;
            }
        };
        auto probe = std::make_unique<Probe>();
        Probe* pp = probe.get();
        auto prof = bb.kin.profile(3 * bb.kin.floor_height);
        std::vector<Passenger> ps;
        Passenger q0; q0.arrival = 0.0; q0.origin = 3; q0.dest = 1; ps.push_back(q0);
        Passenger q1; q1.arrival = prof.moving_time() + 0.25; q1.origin = 3; q1.dest = 5; ps.push_back(q1);
        SimOptions so; so.check_invariants = true;
        Simulation sim(bb, 1, std::move(probe), ps, nullptr, 3600.0, so);
        sim.run();
        check(pp->release > 0.2 && pp->release < 0.3, "decision during leveling sees the remaining settle (" + std::to_string(pp->release) + ")");

        check(pp->seen >= pp->release - 1e-9, "unary during leveling is at least the remaining settle");
    }

    {
        Building bb = building_from(a, 6);
        struct Probe : Dispatcher {
            std::vector<double> predicted;
            std::vector<double> at;
            std::string name() const override { return "probe"; }
            std::map<HallKey, int> decide(const Building& b2, const std::vector<CarView>& cs, const std::vector<CallView>& hs,
                                          const HallKey&, DecisionTrace*) override {
                std::map<HallKey, int> out;
                for (auto& h : hs) out[h.key] = 0;
                std::vector<VCall> vc;
                int id = 0, up_id = -1;
                for (auto& h : hs) {
                    int dest = h.key.dir == DOWN ? 0 : 4;
                    if (h.key == HallKey{2, UP}) up_id = id;
                    vc.push_back(VCall{h.key, dest, 1, id++});
                }
                if (up_id >= 0) {
                    PathResult r = run_path(b2, cs[0], vc, 0);
                    predicted.push_back(r.pickup[up_id]);
                }
                return out;
            }
        };
        auto probe = std::make_unique<Probe>();
        Probe* pp = probe.get();
        auto prof = bb.kin.profile(2 * bb.kin.floor_height);
        std::vector<Passenger> ps;
        Passenger q0; q0.arrival = 0.0; q0.origin = 2; q0.dest = 0; ps.push_back(q0);
        Passenger q1; q1.arrival = prof.moving_time() + 0.25; q1.origin = 2; q1.dest = 4; ps.push_back(q1);
        SimOptions so; so.check_invariants = true;
        Simulation sim(bb, 1, std::move(probe), ps, nullptr, 3600.0, so);
        sim.run();
        const auto& P = sim.passengers();
        double realized = P[1].boarded - P[1].arrival;
        check(!pp->predicted.empty() && std::abs(pp->predicted.front() - realized) < 1e-6,
              "leveling with a committed reversal: evaluator pickup " + std::to_string(pp->predicted.empty() ? -1.0 : pp->predicted.front()) +
                  " vs engine " + std::to_string(realized));
        check(realized > 5.0, "the up call is served after the down trip (realized " + std::to_string(realized) + ")");
    }

    for (int cars : {1, 2, 3}) {
        for (int seed = 1; seed <= 4; ++seed) {
            Building bb = building_from(a, 8);
            auto passengers = interfloor_traffic(8, 12.0, 30.0, 1800.0, 900 + seed * 7 + cars);
            SimOptions so; so.check_invariants = true; so.check_predictions = true;
            Simulation sim(bb, cars, make_dispatcher(cars == 1 ? "pairwise" : "paper2017"), passengers, nullptr, 4 * 3600.0, so);
            sim.run();
            const auto& ps = sim.predictions();
            check(ps.compared >= 10, "prediction check compared enough pickups (" + std::to_string(ps.compared) + ")");
            check(ps.mismatches == 0, "engine/evaluator mismatch: " + std::to_string(ps.mismatches) + "/" + std::to_string(ps.compared) +
                                          " worst " + std::to_string(ps.worst) + " s; first: " + ps.first_mismatch);
            if (seed == 1) std::cerr << "cars=" << cars << " " << predictions_text(ps) << "\n";
            check(ps.invalidated == ps.inv_redecided + ps.inv_joined + ps.inv_foreign_board + ps.inv_open_board + ps.inv_full_release + ps.inv_taken,
                  "invalidation reasons sum to the total");
        }
    }

    for (auto& m : {"collective", "eta", "unary", "pairwise", "pairwise_bonus", "paper2017", "paper2017_order4"}) {
        Args aa = a;
        aa.kv["check"] = "1";
        for (int cars : {2, 3}) {
            RunSpec r{10, cars, 1, 25.0, m};
            Metrics mt = run_once(aa, r, nullptr).metrics;
            check(mt.unserved == 0 && mt.unfinished == 0, std::string("all served and alighted under ") + m);
            check(mt.n > 0, "passengers generated");
            check(mt.episodes == mt.calls_registered, std::string("every call episode closed exactly once under ") + m + " (" + std::to_string(mt.episodes) + " vs " + std::to_string(mt.calls_registered) + ")");
            if (cars == 3)
                std::cerr << m << ": n=" << mt.n << " awt=" << mt.awt << " att=" << mt.att << " ajt=" << mt.ajt << " decisions=" << mt.decisions
                          << " reassign=" << mt.reassignments << " retries=" << mt.unassigned_retries << " us/decision=" << mt.decision_us_mean << "\n";
        }
    }

    {
        std::mt19937_64 g(31);
        Building bb = building_from(a, 10);
        int states = 0, assignments = 0, no_lock_states = 0;
        for (int it = 0; it < 300; ++it) {
            int C = 2 + static_cast<int>(g() % 2), n = 1 + static_cast<int>(g() % 3), L = static_cast<int>(g() % 4);
            std::vector<CarView> cars;
            for (int c = 0; c < C; ++c) cars.push_back(random_rest_car(g, bb, c));
            std::vector<CallView> calls;
            std::set<HallKey> used;
            for (int i = 0; i < n + L; ++i) {
                HallKey k = random_call(g, bb);
                if (used.count(k)) continue;
                used.insert(k);
                CallView cv; cv.key = k; cv.registered = static_cast<double>(i);
                if (i >= n) { cv.close_to_service = true; cv.assigned = static_cast<int>(g() % C); }
                else cv.assigned = (g() % 2) ? static_cast<int>(g() % C) : -1;
                calls.push_back(cv);
            }
            if (calls.empty() || calls.front().close_to_service) continue;
            HallKey nc = calls.front().key;
            CostOptions lk; lk.bonus = true; lk.capacity_penalty = true; lk.locked_aware = true;
            CostOptions plain = lk; plain.locked_aware = false;
            Matrices Mlk = build_matrices(bb, cars, calls, nc, lk), Mp = build_matrices(bb, cars, calls, nc, plain);
            int nn = static_cast<int>(Mlk.N.size());
            bool any_locked = false;
            for (auto& l : Mlk.locked) any_locked |= !l.empty();
            for (int i = 0; i < nn; ++i) for (int c = 0; c < C; ++c) {
                check(std::abs(Mlk.u[i][c] - (Mlk.u_raw[i][c] + Mlk.adj[i][c] + Mlk.u_lk[i][c])) < 1e-9, "u = u_raw + adj + u_lk");
                check(Mlk.u_raw[i][c] == Mp.u_raw[i][c] && Mlk.adj[i][c] == Mp.adj[i][c] && Mlk.w == Mp.w, "locked-aware changes only u_lk");
                if (!any_locked) check(Mlk.u[i][c] == Mp.u[i][c] && Mlk.u_lk[i][c] == 0.0, "no locked calls: u_lk = 0 and u bitwise equal to the plain u");
            }
            if (!any_locked) ++no_lock_states;

            CostOptions raw;
            double cst = 0.0;
            for (int c = 0; c < C; ++c) {
                auto& Lc = Mlk.locked[c];
                std::vector<double> ul(Lc.size());
                for (size_t l = 0; l < Lc.size(); ++l) { ul[l] = unary(bb, cars[c], Lc[l], raw); cst += ul[l]; }
                for (size_t l = 0; l < Lc.size(); ++l) for (size_t m2 = l + 1; m2 < Lc.size(); ++m2) cst += pairwise(bb, cars[c], Lc[l], Lc[m2], ul[l], ul[m2]);
            }
            std::vector<int> x(nn, 0);
            double dmin = 1e300, dmax = -1e300;
            while (true) {

                double full = 0.0;
                for (int c = 0; c < C; ++c) {
                    std::vector<int> free_here;
                    for (int i = 0; i < nn; ++i) if (x[i] == c) free_here.push_back(i);
                    for (int i : free_here) full += Mp.u[i][c];
                    for (size_t p1 = 0; p1 < free_here.size(); ++p1) for (size_t p2 = p1 + 1; p2 < free_here.size(); ++p2) full += Mp.w[free_here[p1]][free_here[p2]][c];
                    for (int i : free_here) for (auto& l : Mlk.locked[c]) {
                        double wl = unary(bb, cars[c], l, raw);
                        full += std::max(0.0, pairwise(bb, cars[c], Mlk.N[i].key, l, Mp.u_raw[i][c], wl));
                    }
                }
                full += cst;
                double d = full - g_of_public(x, Mlk.u, Mlk.w, C, lk);
                dmin = std::min(dmin, d); dmax = std::max(dmax, d);
                ++assignments;
                int q = 0;
                while (q < nn && ++x[q] == C) { x[q] = 0; ++q; }
                if (q == nn) break;
            }
            check(dmax - dmin < 1e-9, "locked-aware g equals the full quadratic up to a constant (spread " + std::to_string(dmax - dmin) + ")");
            check(std::abs(dmin - cst) < 1e-9, "the constant is the locked calls' own terms");
            ++states;

            for (auto name : {"paper2017_lk", "paper2017_ls_lk", "paper2017_ls_re2", "paper2017_ls_lk_re2"}) {
                auto out = make_dispatcher(name)->decide(bb, cars, calls, nc, nullptr);
                for (auto& cv : calls) if (cv.close_to_service) check(out.at(cv.key) == cv.assigned, std::string("locked assignment preserved by ") + name);
                for (auto& cv : calls) check(out.count(cv.key) == 1, "every call has an entry");
            }
            if (!any_locked) {
                check(make_dispatcher("paper2017_ls_lk")->decide(bb, cars, calls, nc, nullptr) == make_dispatcher("paper2017_ls")->decide(bb, cars, calls, nc, nullptr),
                      "no locked calls: ls_lk == ls");
                check(make_dispatcher("paper2017_lk")->decide(bb, cars, calls, nc, nullptr) == make_dispatcher("paper2017")->decide(bb, cars, calls, nc, nullptr),
                      "no locked calls: lk == paper2017");
            }
        }
        std::cerr << "locked-aware checks: " << states << " states (" << no_lock_states << " without locked calls), " << assignments << " assignments\n";
    }

    {
        std::mt19937_64 g(37);
        Building bb = building_from(a, 8);
        int exact_checks = 0, third_order = 0, third_nonzero = 0;
        double worst3 = 0.0;
        for (int it = 0; it < 150; ++it) {
            int C = 1 + static_cast<int>(g() % 2), n = 2 + static_cast<int>(g() % 2);
            std::vector<CarView> cars;
            for (int c = 0; c < C; ++c) cars.push_back(random_rest_car(g, bb, c));
            std::vector<CallView> calls;
            std::set<HallKey> used;
            for (int i = 0; i < n; ++i) {
                HallKey k = random_call(g, bb);
                if (used.count(k)) continue;
                used.insert(k);
                CallView cv; cv.key = k; cv.registered = static_cast<double>(i); cv.assigned = -1;
                calls.push_back(cv);
            }
            if (calls.size() < 2) continue;
            HallKey nc = calls.front().key;
            CostOptions o; o.bonus = true; o.capacity_penalty = true; o.locked_aware = true; o.reeval_candidates = 8;
            Matrices M = build_matrices(bb, cars, calls, nc, o);
            int nn = static_cast<int>(M.N.size());
            std::vector<std::vector<int>> dest_free;
            all_destination_combinations(bb, M, dest_free);
            std::vector<std::vector<std::vector<int>>> dest_locked(C);
            std::vector<int> x(nn, 0);
            while (true) {
                std::vector<int> cnt(C, 0);
                for (int i = 0; i < nn; ++i) ++cnt[x[i]];
                auto tot = path_cost_samples(bb, cars, M, x, dest_free, dest_locked, true);
                double mean = 0.0; for (double v : tot) mean += v; mean /= tot.size();
                double score = mean + reeval_adjustment(M, x, C, o);
                double gq = g_of_public(x, M.u, M.w, C, o);
                if (*std::max_element(cnt.begin(), cnt.end()) <= 2) { check(std::abs(score - gq) < 1e-7, "v2 score == g when no car has more than two calls (diff " + std::to_string(score - gq) + ")"); ++exact_checks; }
                else { ++third_order; if (std::abs(score - gq) > 1e-7) ++third_nonzero; worst3 = std::max(worst3, std::abs(score - gq)); }
                int q = 0;
                while (q < nn && ++x[q] == C) { x[q] = 0; ++q; }
                if (q == nn) break;
            }
        }
        std::cerr << "re-evaluation identities: " << exact_checks << " candidates with <= 2 calls per car (score == g); " << third_order
                  << " with 3 calls on a car, of which " << third_nonzero << " differ from g (largest |third-order term| " << worst3 << " s)\n";
        check(exact_checks > 100, "enough order-2 identity checks");
    }

    {
        Building bb; bb.floors = 10;
        CarView c0, c1; c0.id = 0; c0.dir = UP; c0.load = 11; c0.car_calls[8] = 1; c1.id = 1; c1.x = bb.pos(9); c1.committed = 9;
        CallView h; h.key = {0, UP}; h.registered = 1;
        std::map<std::string, int> got;
        for (auto name : {"paper2017_ls", "paper2017_ls_lk", "paper2017_ls_re2", "paper2017_ls_lk_re2", "paper2017_ls_re"}) {
            DecisionTrace t;
            auto out = make_dispatcher(name)->decide(bb, {c0, c1}, {h}, h.key, &t);
            got[name] = out.at(h.key);
            if (std::string(name).find("re2") != std::string::npos) {
                check(out.at(h.key) == 1, std::string("v2 re-evaluation assigns the idle far car under ") + name);
                check(t.reeval_removed == 0, "v2 re-evaluation: nothing removed after selection");
                check(t.reeval_candidates == 2 && t.reeval_scores.size() == 2 && t.reeval_scores[1] > t.reeval_scores[0] + 900.0, "v2 score carries the 1000 s capacity penalty");
            }
        }
        check(got["paper2017_ls"] == 1 && got["paper2017_ls_lk"] == 1, "LS assigns the idle far car");
        check(got["paper2017_ls_re"] == -1, "legacy v1 re-evaluation reproduces the review's defect (assignment removed)");

        CarView i0, i1; i0.id = 0; i1.id = 1; i0.x = i1.x = bb.pos(5); i0.committed = i1.committed = 5;
        CallView k; k.key = {2, UP}; k.registered = 1;
        DecisionTrace t;
        make_dispatcher("paper2017_ls_lk_re2")->decide(bb, {i0, i1}, {k}, k.key, &t);
        check(t.reeval_candidates == 2 && t.reeval_pick == 0 && std::abs(t.reeval_scores[0] - t.reeval_scores[1]) < 1e-12, "equal scores keep the search result");
    }

    {
        std::mt19937_64 g(41);
        Building bb = building_from(a, 10);
        int checked = 0, two_call_instances = 0, fig4_single = 0;
        double gap_sum = 0.0;
        for (int it = 0; it < 200; ++it) {
            int C = 1 + static_cast<int>(g() % 3);
            std::vector<CarView> cars;
            for (int c = 0; c < C; ++c) cars.push_back(random_rest_car(g, bb, c));
            std::vector<CallView> calls;
            std::set<HallKey> used;
            int n = 1 + static_cast<int>(g() % 4);
            for (int i = 0; i < n; ++i) {
                HallKey k = random_call(g, bb);
                if (used.count(k)) continue;
                used.insert(k);
                CallView cv; cv.key = k; cv.registered = 0.0; cv.waited = static_cast<double>(g() % 80); cv.assigned = -1;
                calls.push_back(cv);
            }
            std::sort(calls.begin(), calls.end(), [](const CallView& x, const CallView& y) { return x.key < y.key; });
            n = static_cast<int>(calls.size());
            std::vector<int> genes(n), est(n, 1);
            for (int i = 0; i < n; ++i) genes[i] = static_cast<int>(g() % C);
            CostOptions raw;
            GaOptions go; go.pen_weight = 0.0; go.pen_threshold = 1e9; go.route = GaOptions::COLLECTIVE;
            double f = Ga2022Dispatcher::fitness(bb, cars, calls, genes, est, go);

            {
                GaOptions gf = go; gf.route = GaOptions::FIG4;
                for (int c = 0; c < C; ++c) {
                    std::vector<int> on;
                    for (int i = 0; i < n; ++i) if (genes[i] == c) on.push_back(i);
                    if (on.size() != 1 || !cars[c].car_calls.empty()) continue;
                    std::vector<GaCall> gc{GaCall{calls[on[0]].key, 1, calls[on[0]].waited}};
                    double lc = Ga2022Dispatcher::lift_cost(bb, cars[c], gc, gf);
                    double u = unary(bb, cars[c], calls[on[0]].key, raw);
                    check(std::abs(lc - (u + bb.doors.open + bb.doors.transfer)) < 1e-6, "Fig. 4 route, one call, no car calls: cost = unary + door open + transfer (diff " + std::to_string(lc - u - bb.doors.open - bb.doors.transfer) + ")");
                    ++fig4_single;
                }
            }

            double ref = 0.0; bool ok = true;
            for (int c = 0; c < C && ok; ++c) {
                std::vector<int> on;
                for (int i = 0; i < n; ++i) if (genes[i] == c) on.push_back(i);
                if (on.empty()) continue;
                if (on.size() == 1) ref += unary(bb, cars[c], calls[on[0]].key, raw);
                else if (on.size() == 2) {
                    double ui = unary(bb, cars[c], calls[on[0]].key, raw), uj = unary(bb, cars[c], calls[on[1]].key, raw);
                    ref += ui + uj + pairwise(bb, cars[c], calls[on[0]].key, calls[on[1]].key, ui, uj);
                } else ok = false;
            }
            if (!ok) continue;

            bool single = true;
            for (int c = 0; c < C; ++c) { int cnt = 0; for (int i = 0; i < n; ++i) cnt += genes[i] == c; single &= cnt <= 1; }
            if (single) check(std::abs(f - ref) < 1e-6, "GA fitness equals the 2017 unary for one call per car (diff " + std::to_string(f - ref) + ")");
            else check(f <= ref + 1e-6, "GA fitness <= 2017 quadratic for two calls per car (diff " + std::to_string(f - ref) + ")");
            gap_sum += ref - f; if (!single) ++two_call_instances;

            GaOptions gp; gp.pen_weight = 2.0; gp.pen_threshold = 30.0; gp.route = GaOptions::COLLECTIVE;
            double fp = Ga2022Dispatcher::fitness(bb, cars, calls, genes, est, gp);
            double pen = 0.0;
            for (int c = 0; c < C; ++c) {
                std::vector<VCall> vc; std::vector<int> idx;
                for (int i = 0; i < n; ++i) if (genes[i] == c) { vc.push_back(VCall{calls[i].key, -1, 1, static_cast<int>(vc.size())}); idx.push_back(i); }
                if (vc.empty()) continue;
                PathResult r = run_path(bb, cars[c], vc, 0);
                for (size_t q = 0; q < vc.size(); ++q) pen += 2.0 * std::max(0.0, calls[idx[q]].waited + r.pickup[vc[q].id] - 30.0);
            }
            check(std::abs(fp - f - pen) < 1e-6, "GA penalty adds the weighted excess over the threshold");

            std::vector<int> est2(n); for (int i = 0; i < n; ++i) est2[i] = 1 + static_cast<int>(g() % 4);
            double fe = Ga2022Dispatcher::fitness(bb, cars, calls, genes, est2, go), fe_ref = 0.0;
            for (int c = 0; c < C; ++c) {
                std::vector<VCall> vc; std::vector<int> idx;
                for (int i = 0; i < n; ++i) if (genes[i] == c) { vc.push_back(VCall{calls[i].key, -1, 1, static_cast<int>(vc.size())}); idx.push_back(i); }
                if (vc.empty()) continue;
                PathResult r = run_path(bb, cars[c], vc, 0);
                for (size_t q = 0; q < vc.size(); ++q) fe_ref += r.pickup[vc[q].id] * est2[idx[q]];
            }
            check(std::abs(fe - fe_ref) < 1e-6, "GA estimation weights the pickup times");
            ++checked;
        }
        std::cerr << "GA fitness identity checks (collective-route variant): " << checked << " (" << two_call_instances << " with two calls on a car; mean quadratic - GA gap "
                  << (two_call_instances ? gap_sum / two_call_instances : 0.0) << " s = the first-served call's alighting stop); Fig. 4 single-call identities " << fig4_single << "\n";
    }

    {
        Building bb = building_from(a, 10);
        const double DOT = bb.doors.open, DCT = bb.doors.close, TR = bb.doors.transfer;
        GaOptions go; go.pen_weight = 0.0; go.pen_threshold = 1e9;
        auto stops_of = [](const GaRouteResult& r) { std::vector<int> v; for (auto& st : r.stops) v.push_back(st.floor); return v; };

        {
            CarView car; car.id = 0;
            std::vector<GaCall> calls{GaCall{{3, UP}, 1, 0.0}, GaCall{{6, UP}, 1, 0.0}};
            auto r = ga_route(bb, car, calls, go);
            double t3 = bb.kin.travel(0, 3) + DOT + TR, t6 = t3 + DCT + bb.kin.travel(3, 6) + DOT + TR;
            check(stops_of(r) == std::vector<int>{3, 6}, "route A: stops 3, 6");
            check(std::abs(r.first_pickup[0] - t3) < 1e-9 && std::abs(r.first_pickup[1] - t6) < 1e-9, "route A: boarding completion times");
            check(std::abs(r.ewt - (t3 + t6)) < 1e-9, "route A: EWT = t3 + t6");

            GaOptions gl = go; gl.ewt = GaOptions::LITERAL;
            std::vector<GaCall> c2{GaCall{{3, UP}, 2, 0.0}, GaCall{{6, UP}, 1, 0.0}};
            auto ra = ga_route(bb, car, c2, go), rl = ga_route(bb, car, c2, gl);
            double t3b = bb.kin.travel(0, 3) + DOT + 2 * TR, seg = DCT + bb.kin.travel(3, 6) + DOT + TR;
            check(std::abs(ra.ewt - (2 * t3b + (t3b + seg))) < 1e-9, "route A (2,1) ACCUMULATE: 2*t3 + t6");
            check(std::abs(rl.ewt - (2 * t3b + (2 * t3b + seg))) < 1e-9, "route A (2,1) LITERAL: acc carried into the next stop");
        }

        {
            GaOptions gl = go; gl.ewt = GaOptions::LITERAL;
            CarView car; car.id = 0; car.x = bb.pos(2); car.committed = 2; car.release = 10.0;
            std::vector<GaCall> calls{GaCall{{5, UP}, 1, 0.0}, GaCall{{8, UP}, 1, 0.0}};
            auto ra = ga_route(bb, car, calls, go), rl = ga_route(bb, car, calls, gl);
            check(std::abs(ra.ewt - rl.ewt) < 1e-9 && std::abs(ra.first_pickup[0] - (10.0 + bb.kin.travel(2, 5) + DOT + TR)) < 1e-9, "route A': literal == accumulate with a release of 10 s (one passenger per call)");
            CarView fly; fly.id = 0; fly.moving = true; fly.dir = UP; fly.committed = 3; fly.x = bb.pos(2) + 1.0; fly.v = 1.2;
            auto fa = ga_route(bb, fly, calls, go), fl = ga_route(bb, fly, calls, gl);
            check(std::abs(fa.ewt - fl.ewt) < 1e-9, "route A'': literal == accumulate from an in-flight state (one passenger per call)");
            CarView od; od.id = 0; od.x = bb.pos(5); od.committed = 5; od.dir = UP; od.doors_open = true; od.release = 4.2;
            auto oa = ga_route(bb, od, calls, go), ol = ga_route(bb, od, calls, gl);
            check(std::abs(oa.ewt - ol.ewt) < 1e-9, "route A''': literal == accumulate with an open-door boarding (one passenger per call)");
        }

        {
            CarView car; car.id = 0;
            std::vector<GaCall> calls{GaCall{{4, DOWN}, 1, 0.0}, GaCall{{7, UP}, 1, 0.0}};
            auto r = ga_route(bb, car, calls, go);
            check(stops_of(r) == std::vector<int>{4, 7}, "route B: Fig. 4 serves 4 (down) before 7 (up)");
            PathResult pr = run_path(bb, car, {VCall{{4, DOWN}, -1, 1, 0}, VCall{{7, UP}, -1, 1, 1}}, 0);
            check(pr.order == std::vector<int>{1, 0}, "route B: rule R serves 7 before 4 (documented difference)");
        }

        {
            CarView car; car.id = 0; car.x = bb.pos(5); car.committed = 5; car.dir = UP; car.load = 2; car.car_calls[9] = 1;
            std::vector<GaCall> calls{GaCall{{3, UP}, 1, 0.0}};
            auto r = ga_route(bb, car, calls, go);
            check(stops_of(r) == std::vector<int>{9, 3} && r.stops[0].alight == 2 && r.stops[1].board == 1, "route C: car call first, then the call behind");
            double t3 = bb.kin.travel(5, 9) + DOT + 2 * TR + DCT + bb.kin.travel(9, 3) + DOT + TR;
            check(std::abs(r.first_pickup[0] - t3) < 1e-9, "route C: pickup time");
        }

        {
            CarView car; car.id = 0; car.dir = UP; car.load = 12; car.car_calls[9] = 1;
            std::vector<GaCall> calls{GaCall{{3, UP}, 3, 0.0}};
            auto r = ga_route(bb, car, calls, go);
            check(stops_of(r) == std::vector<int>{3, 9, 3} && r.stops[0].board == 1 && r.stops[1].alight == 12 && r.stops[2].board == 2, "route D: partial boarding, unload, stop again");
            double ta = bb.kin.travel(0, 3) + DOT + TR, tb = ta + DCT + bb.kin.travel(3, 9) + DOT + 12 * TR + DCT + bb.kin.travel(9, 3) + DOT + 2 * TR;
            check(std::abs(r.contribution[0] - (ta * 1 + tb * 2)) < 1e-9 && std::abs(r.first_pickup[0] - ta) < 1e-9, "route D: contribution = t_a*1 + t_b*2");

            std::vector<GaCall> one{GaCall{{3, UP}, 1, 0.0}};
            check(stops_of(ga_route(bb, car, one, go)) == std::vector<int>{3, 9}, "route D': estimation 1 fits at once");

            Building b15 = bb; b15.capacity = 15;
            check(stops_of(ga_route(b15, car, calls, go)) == std::vector<int>{3, 9}, "route D'': larger capacity, one stop");
        }

        {
            Building b2 = bb; b2.capacity = 2;
            CarView car; car.id = 0;
            std::vector<GaCall> calls{GaCall{{2, UP}, 3, 0.0}, GaCall{{5, UP}, 1, 0.0}};
            auto r = ga_route(b2, car, calls, go);
            check(stops_of(r) == std::vector<int>{2, 2, 2, 5} && r.stops[0].board == 2 && r.stops[1].alight == 2 && r.stops[2].board == 1 && r.stops[3].board == 1 && r.unloading_events == 1,
                  "route E: forced unloading before the residual passenger boards");
        }

        {
            CarView car; car.id = 0; car.x = bb.pos(4); car.committed = 4; car.dir = UP; car.doors_open = true; car.release = 4.2;
            std::vector<GaCall> calls{GaCall{{4, UP}, 1, 0.0}, GaCall{{8, UP}, 1, 0.0}};
            auto r = ga_route(bb, car, calls, go);
            check(std::abs(r.first_pickup[0] - TR) < 1e-9, "route F: open-door boarding completes after the transfer");
            check(std::abs(r.first_pickup[1] - (4.2 + TR + bb.kin.travel(4, 8) + DOT + TR)) < 1e-9, "route F: next pickup after the extended dwell");
        }

        {
            std::mt19937_64 g(53);
            int n_ok = 0;
            for (int it = 0; it < 500; ++it) {
                Building bx = bb; bx.capacity = 1 + static_cast<int>(g() % 13);
                CarView car = random_rest_car(g, bx, 0);
                car.load = std::min(car.load, bx.capacity);
                if (car.car_calls.empty() && car.load > 0 && (g() % 2)) car.load = 0;
                std::vector<GaCall> calls;
                std::set<HallKey> used;
                int n = 1 + static_cast<int>(g() % 5);
                for (int i = 0; i < n; ++i) { HallKey k = random_call(g, bx); if (used.insert(k).second) calls.push_back(GaCall{k, 1 + static_cast<int>(g() % 4), 0.0}); }
                auto r = ga_route(bx, car, calls, go);
                bool all = true;
                for (double v : r.first_pickup) all &= v >= 0;
                check(all, "route G: every call is picked up");
                n_ok += all;
            }
            std::cerr << "Fig. 4 route random termination checks: " << n_ok << "/500\n";
        }
    }

    {
        std::mt19937_64 g(43);
        Building bb = building_from(a, 10);
        int states = 0, improved_over_seed = 0;
        GaOptions go; go.seed = 77;
        Ga2022Dispatcher d1(go, "ga"), d2(go, "ga");
        for (int it = 0; it < 150; ++it) {
            int C = 2 + static_cast<int>(g() % 3);
            std::vector<CarView> cars;
            for (int c = 0; c < C; ++c) cars.push_back(random_rest_car(g, bb, c));
            std::vector<CallView> calls;
            std::set<HallKey> used;
            int n = 1 + static_cast<int>(g() % 6);
            for (int i = 0; i < n; ++i) {
                HallKey k = random_call(g, bb);
                if (used.count(k)) continue;
                used.insert(k);
                CallView cv; cv.key = k; cv.registered = it * 10.0; cv.waited = static_cast<double>(g() % 40); cv.assigned = (g() % 2) ? static_cast<int>(g() % C) : -1;
                cv.close_to_service = cv.assigned >= 0 && (g() % 3 == 0);
                calls.push_back(cv);
            }
            HallKey nc = calls.front().key;
            DecisionTrace t1, t2;
            auto o1 = d1.decide(bb, cars, calls, nc, &t1);
            auto o2 = d2.decide(bb, cars, calls, nc, &t2);
            check(o1 == o2 && t1.ga_final_fit == t2.ga_final_fit, "GA decisions are deterministic for equal seeds");
            for (auto& cv : calls) {
                check(o1.count(cv.key) == 1 && o1.at(cv.key) >= 0 && o1.at(cv.key) < C, "GA assigns every call to a valid car");
                if (cv.close_to_service && !(cv.key == nc) && cv.assigned >= 0) check(o1.at(cv.key) == cv.assigned, "GA preserves locked assignments");
            }
            for (size_t k = 1; k < t1.ga_best_by_gen.size(); ++k) check(t1.ga_best_by_gen[k] <= t1.ga_best_by_gen[k - 1] + 1e-9, "GA best fitness never worsens (elitism)");
            check(t1.ga_best_by_gen.size() == static_cast<size_t>(go.gens + 1), "GA records the initial and every generation's best");
            check(t1.ga_final_fit <= t1.ga_seed_fit + 1e-9, "GA result is no worse than the seeding individual");
            check(t1.ga_evaluations == go.pop * (go.gens + 1), "GA evaluation count = pop * (gens + 1)");
            improved_over_seed += t1.ga_final_fit < t1.ga_seed_fit - 1e-9;
            ++states;
        }
        std::cerr << "GA decide checks: " << states << " states, result strictly better than the nearest-lift seed in " << improved_over_seed << "\n";
    }

    {
        Building bb = building_from(a, 10);

        std::vector<Passenger> ps;
        Passenger p1; p1.arrival = 0.0; p1.origin = 6; p1.dest = 8; ps.push_back(p1);
        Passenger p2; p2.arrival = 3.0; p2.origin = 2; p2.dest = 4; ps.push_back(p2);
        for (auto mode : {SimOptions::STABILITY, SimOptions::THETA}) {
            SimOptions so; so.lock_mode = mode; so.theta = 5.0; so.check_invariants = true;
            bool seen = false, locked = false;
            so.on_decision = [&](double t, int, const Building&, const std::vector<CarView>& cv, const std::vector<CallView>& calls, const HallKey& nc) {
                if (!(nc == HallKey{2, UP})) return;
                seen = true;
                for (auto& k : calls) if (k.key == HallKey{6, UP}) {
                    locked = k.close_to_service;
                    check(k.assigned == 0 && cv[0].moving && cv[0].load == 0, "scenario: car 0 is empty and in flight at t=" + std::to_string(t));
                }
            };
            Simulation sim(bb, 1, make_dispatcher("paper2017"), ps, nullptr, 3600.0, so);
            sim.run();
            check(seen, "scenario decision observed");
            if (mode == SimOptions::STABILITY) check(locked, "stability: an empty car en route locks its call");
            else check(!locked, "theta=5: the same call is not locked 3 s into a long flight");
            check(sim.metrics().unserved == 0 && sim.metrics().unfinished == 0, "scenario completes");
        }

        {
            std::vector<Passenger> qs;
            Passenger a0; a0.arrival = 0.0; a0.origin = 0; a0.dest = 9; qs.push_back(a0);
            Passenger a1; a1.arrival = 6.0; a1.origin = 7; a1.dest = 9; qs.push_back(a1);

            int q_floor = 1;
            for (double tq : {8.0, 12.0, 16.0, 20.0, 21.8, 22.4, 23.0, 23.6}) {
                Passenger q; q.arrival = tq; q.origin = q_floor; q.dest = 0; qs.push_back(q); ++q_floor;
                if (q_floor == 7) q_floor = 8;
            }
            SimOptions so; so.lock_mode = SimOptions::STABILITY; so.check_invariants = true;
            std::vector<std::pair<double, int>> obs;
            so.on_decision = [&](double t, int, const Building&, const std::vector<CarView>& cv, const std::vector<CallView>& calls, const HallKey&) {
                for (auto& k : calls) if (k.key == HallKey{7, UP} && k.assigned == 0 && (cv[0].moving || cv[0].arriving) && cv[0].load > 0) obs.emplace_back(t, k.close_to_service ? 1 : 0);
            };
            Simulation sim(bb, 1, make_dispatcher("paper2017"), qs, nullptr, 3600.0, so);
            sim.run();
            int unlocked_then_locked = 0; bool order_ok = true; int last = 0;
            for (auto& [t, l] : obs) { if (l < last) order_ok = false; last = l; }
            for (auto& [t, l] : obs) unlocked_then_locked += l;
            check(!obs.empty() && order_ok && unlocked_then_locked > 0 && unlocked_then_locked < static_cast<int>(obs.size()),
                  "stability: a loaded car locks its call only from the deceleration phase on (" + std::to_string(unlocked_then_locked) + "/" + std::to_string(obs.size()) + " locked)");
        }

        {
            auto passengers = interfloor_traffic(8, 20.0, 30.0, 600.0, 4242);
            SimOptions so; so.period = 0.25; so.lock_mode = SimOptions::STABILITY; so.check_invariants = true;
            int off_grid = 0, decisions = 0;
            std::set<double> arrival_times;
            for (auto& p : passengers) arrival_times.insert(p.arrival);
            so.on_decision = [&](double t, int, const Building&, const std::vector<CarView>&, const std::vector<CallView>&, const HallKey& nc) {
                ++decisions;
                double q = t / 0.25;
                if (std::abs(q - std::round(q)) > 1e-6) ++off_grid;
                check(nc.floor < 0, "periodic mode: decisions carry no new call");
            };
            Args aa = a;
            Simulation sim(bb, 2, dispatcher_from(aa, "ga2022_all", 5), passengers, nullptr, 4 * 3600.0, so);
            sim.run();
            check(decisions > 0 && off_grid == 0, "periodic mode: every decision is on the 0.25 s grid");
            check(sim.metrics().unserved == 0 && sim.metrics().unfinished == 0, "periodic mode completes");
        }
    }

    for (auto& m : {"ga2022_all", "ga2022_all_ewtp"}) {
        for (int orig : {0, 1}) {
            Args aa = a;
            aa.kv["check"] = "1"; aa.kv["check-pred"] = "1";
            if (orig) { aa.kv["lock"] = "stability"; aa.kv["period"] = "0.25"; }
            for (int cars : {2, 3}) {
                RunSpec r{10, cars, 1, 25.0, m};
                RunResult rr = run_once(aa, r, nullptr);
                check(rr.metrics.unserved == 0 && rr.metrics.unfinished == 0, std::string("all served and alighted under ") + m + (orig ? " (stability, 0.25 s)" : " (theta, events)"));
                check(rr.predictions.mismatches == 0, std::string("prediction check under ") + m + ": " + std::to_string(rr.predictions.mismatches) + " mismatches");
                if (cars == 3)
                    std::cerr << m << (orig ? " [stability, 0.25 s]" : " [theta=5, events]") << ": n=" << rr.metrics.n << " awt=" << rr.metrics.awt << " att=" << rr.metrics.att
                              << " ajt=" << rr.metrics.ajt << " decisions=" << rr.metrics.decisions << " reassign=" << rr.metrics.reassignments
                              << " us/decision=" << rr.metrics.decision_us_mean << " pred compared=" << rr.predictions.compared << "\n";
            }
        }
    }

    {
        Building bb = building_from(a, 10);
        auto passengers = interfloor_traffic(10, 25.0, 30.0, 1800.0, 777);
        SimOptions so; so.check_invariants = true;
        Simulation A(bb, 3, make_dispatcher("paper2017_ls_lk"), passengers, nullptr, 600.0, so);
        A.run();
        Simulation B(A, make_dispatcher("paper2017_ls_lk"));
        A.set_max_time(4 * 3600.0); B.set_max_time(4 * 3600.0);
        A.run(); B.run();
        Metrics ma = A.metrics(), mb = B.metrics();
        check(ma.n == mb.n && ma.awt == mb.awt && ma.att == mb.att && ma.unserved == 0 && mb.unserved == 0 && ma.reassignments == mb.reassignments,
              "a mid-run clone continues identically to the original");
        Simulation C0(bb, 3, make_dispatcher("paper2017_ls_lk"), passengers, nullptr, 600.0, so);
        C0.run();
        Simulation C(C0, make_dispatcher("paper2017_ls_lk"));
        C.set_max_time(4 * 3600.0);
        C.drop_arrivals_after(600.0);
        C.resample_waiting_destinations(5);
        C.run();
        double cw = C.replay_cost(600.0, false), cs = C.replay_cost(600.0, true);
        check(cw >= 0 && cw < 1e5 && cs >= cw, "replay without further arrivals serves everybody present (finite cost, system >= wait)");
        int late = 0; for (auto& p : C.passengers()) if (p.arrival > 600.0 && p.boarded >= 0) ++late;
        check(late == 0, "dropped arrivals never board in the replay");
        for (auto m : {"ro_nofuture_wait", "ro_future_sys_h30"}) {
            Args aa = a; aa.kv["check"] = "1";
            RunSpec r{8, 2, 1, 25.0, m};
            Metrics mt = run_once(aa, r, nullptr).metrics;
            check(mt.unserved == 0 && mt.unfinished == 0 && mt.rollout_decisions > 0, std::string("rollout policy completes: ") + m);
        }
        std::cerr << "rollout machinery checks ok (clone identical, replay cost wait " << cw << " system " << cs << ")\n";
    }

    {
        Building b3 = building_from(a, 3); b3.capacity = 1;

        std::vector<Passenger> ps;
        Passenger p1; p1.arrival = 0.0; p1.origin = 1; p1.dest = 0; ps.push_back(p1);
        Passenger p2; p2.arrival = 0.1; p2.origin = 1; p2.dest = 0; ps.push_back(p2);
        SimOptions so; so.check_invariants = true;
        Simulation sim(b3, 1, make_dispatcher("paper2017_ls_lk"), ps, nullptr, 3600.0, so);
        sim.run();
        Metrics m = sim.metrics();
        check(m.unserved == 0 && m.calls_registered == 1 && m.episodes == 1 && m.episodes_2 == 1 && m.episodes_1 == 0,
              "F1: an episode cleared at the second visit counts two attempts (calls " + std::to_string(m.calls_registered) + ", episodes " + std::to_string(m.episodes) +
                  ", 1/2/3+ " + std::to_string(m.episodes_1) + "/" + std::to_string(m.episodes_2) + "/" + std::to_string(m.episodes_3p) + ")");
        check(m.full_events == 1 && m.full_left == 1 && m.passed_over_passengers == 1, "F1: one full event leaving one passenger behind");

        Building bb = building_from(a, 3);
        std::vector<Passenger> qs;
        Passenger q0; q0.arrival = 0.0; q0.origin = 2; q0.dest = 0; qs.push_back(q0);
        Passenger q1; q1.arrival = 1.0; q1.origin = 1; q1.dest = 0; qs.push_back(q1);
        Passenger q2; q2.arrival = 19.0; q2.origin = 1; q2.dest = 0; qs.push_back(q2);
        Simulation sim2(bb, 1, make_dispatcher("paper2017_ls_lk"), qs, nullptr, 3600.0, so);
        sim2.run();
        Metrics m2 = sim2.metrics();
        int open_boarders = 0; for (auto& p : sim2.passengers()) if (p.id == 2 && p.boarded >= 0 && p.boarded - p.arrival < 0.5) ++open_boarders;
        check(open_boarders == 1, "F2 scenario: the third passenger boards through the open doors");
        check(std::abs(m2.onboard_dwell_ps - 7.4) < 1e-6 && std::abs(m2.pickup_only_onboard_ps - 7.4) < 1e-6,
              "F2: the open-door extension counts for the continuing passenger (got " + std::to_string(m2.onboard_dwell_ps) + ")");
        check(m2.stops == 3 && m2.pickup_only_stops == 2 && m2.dropoff_only_stops == 1 && m2.mixed_stops == 0, "F2: stop classification");
        check(m2.episodes == m2.calls_registered, "F2: every call episode is closed once");
        std::cerr << "phenomenon metric regression cases ok\n";
    }

    {
        int cases = 0, failed = 0;
        for (int p : {1, 3, 6, 10}) for (int m : {1, 2, 3}) for (int f : {3, 5, 7}) for (int bfl : {0, 1, 8}) for (int strict : {0, 1}) {
            Args aa = a;
            aa.kv["floors"] = "10"; aa.kv["p"] = std::to_string(p); aa.kv["m"] = std::to_string(m); aa.kv["f"] = std::to_string(f); aa.kv["b"] = std::to_string(bfl);
            if (strict) aa.kv["strict-boarding"] = "1";
            double pred = 0, actual = 0;
            int rc = restricted_case(aa, pred, actual, false);
            if (rc == 1) continue;
            ++cases;
            if (rc != 0) { ++failed; std::cerr << "restricted basic identity failed: p" << p << " m" << m << " f" << f << " b" << bfl << (strict ? " strict" : "") << " pred " << pred << " actual " << actual << "\n"; }
        }
        check(cases >= 100 && failed == 0, "restricted basic identity holds in every constructible case (" + std::to_string(cases) + " cases, " + std::to_string(failed) + " failed)");

        int cases_b = 0, failed_b = 0;
        for (int q : {1, 4, 11}) for (int p : {1, 3, 6, 10}) for (int m : {1, 2}) for (int bdelay : {0, 5}) for (int strict : {0, 1}) {
            Args aa = a;
            aa.kv["floors"] = "10"; aa.kv["p"] = std::to_string(p); aa.kv["m"] = std::to_string(m); aa.kv["f"] = "6"; aa.kv["q"] = std::to_string(q); aa.kv["bq"] = "8";
            aa.kv["bdelay"] = std::to_string(bdelay);
            if (strict) aa.kv["strict-boarding"] = "1";
            double pred = 0, actual = 0;
            int rc = restricted_case(aa, pred, actual, false);
            if (rc == 1) continue;
            ++cases_b;
            if (rc != 0) { ++failed_b; std::cerr << "restricted M_B identity failed: p" << p << " q" << q << " m" << m << " bdelay" << bdelay << (strict ? " strict" : "") << " pred " << pred << " actual " << actual << "\n"; }
        }
        check(cases_b >= 40 && failed_b == 0, "restricted M_B identity (with q delta_B) holds in every constructible case (" + std::to_string(cases_b) + " cases, " + std::to_string(failed_b) + " failed)");
        std::cerr << "restricted basic identity: " << cases << " constructible cases, " << failed << " failures; M_B: " << cases_b << " cases, " << failed_b << " failures\n";
    }

    {
        Building bb = building_from(a, 10);
        auto passengers = interfloor_traffic(10, 30.0, 30.0, 1800.0, 0);
        int reassigned_default = -1;
        for (int fixed : {0, 1}) for (int stability : {0, 1}) {
            SimOptions so; so.check_invariants = true; so.fixed_assignment = fixed;
            if (stability) { so.lock_mode = SimOptions::STABILITY; so.period = 0.25; }
            Simulation sim(bb, 3, make_dispatcher("paper2017_ls_lk"), passengers, nullptr, 4 * 3600.0, so);
            sim.run();
            Metrics m = sim.metrics();
            std::string tag = std::string(stability ? "stability+period" : "theta") + (fixed ? ", fixed" : ", default");
            check(m.unserved == 0 && m.unfinished == 0, "fixed-assignment contract: everybody is served (" + tag + ")");
            if (fixed) check(m.reassignments == 0, "fixed-assignment contract: no call changes car (" + tag + ", " + std::to_string(m.reassignments) + " reassignments)");
            else if (!stability) reassigned_default = m.reassignments;
        }
        check(reassigned_default > 0, "the default rules do reassign on the same traffic (" + std::to_string(reassigned_default) + "): the contract check is not vacuous");

        std::vector<Passenger> ps;
        { Passenger q; q.arrival = 0.0; q.origin = 0; q.dest = 8; ps.push_back(q); }
        for (int i = 0; i < 20; ++i) { Passenger q; q.arrival = 60.0; q.origin = 9; q.dest = 0; ps.push_back(q); }
        SimOptions so; so.check_invariants = true; so.fixed_assignment = true;
        so.forced_assignment[HallKey{0, UP}] = 1;
        so.forced_assignment[HallKey{9, DOWN}] = 0;
        Simulation sim(bb, 2, make_dispatcher("paper2017_ls_lk"), ps, nullptr, 4 * 3600.0, so);
        sim.run();
        Metrics m = sim.metrics();
        int car0 = 0; for (auto& q : sim.passengers()) if (q.origin == 9 && q.car == 0) ++car0;
        check(m.unserved == 0 && m.full_events >= 1 && m.reassignments == 0 && car0 == 20,
              "fixed-assignment contract: a full car keeps the call and returns (full events " + std::to_string(m.full_events) + ", boarded car 0: " + std::to_string(car0) + "/20)");
        SimOptions so2 = so; so2.fixed_assignment = false;
        Simulation sim2(bb, 2, make_dispatcher("paper2017_ls_lk"), ps, nullptr, 4 * 3600.0, so2);
        sim2.run();
        int car0b = 0; for (auto& q : sim2.passengers()) if (q.origin == 9 && q.car == 0) ++car0b;
        check(car0b < 20, "the default rules hand the released call to another car in the same scenario (" + std::to_string(car0b) + "/20 on car 0): the full-car check is not vacuous");

        int moved_once = 0, interventions = 0;
        for (double t_branch : {300.0, 600.0, 900.0, 1200.0}) {
            SimOptions so3; so3.check_invariants = true; so3.fixed_assignment = true;
            Simulation s3(bb, 3, make_dispatcher("paper2017_ls_lk"), passengers, nullptr, t_branch, so3);
            s3.run();
            HallKey target{-1, 0}; int car = -1;
            for (auto& [k, st] : s3.calls())
                if (st.assigned >= 0 && !s3.call_locked_by_theta(k)) { target = k; car = st.assigned; break; }
            if (car < 0) continue;
            Simulation s4(s3, make_dispatcher("paper2017_ls_lk"));
            s4.set_max_time(4 * 3600.0);
            SimOptions::Intervention iv; iv.mode = SimOptions::Intervention::REASSIGN; iv.call = target; iv.car = car;
            s4.apply_intervention(iv);
            ++interventions;
            auto it = s4.calls().find(target);
            if (it != s4.calls().end() && it->second.assigned >= 0 && it->second.assigned != car) ++moved_once;
            s4.run();
            Metrics m4 = s4.metrics();
            check(m4.reassignments == 0 && m4.unserved == 0, "fixed-assignment contract with an intervention (3 cars, branch " + std::to_string(int(t_branch)) + " s): the target call moves once and nothing moves afterwards (" + std::to_string(m4.reassignments) + " reassignments)");
        }
        check(interventions >= 2 && moved_once == interventions, "the intervention itself moved the target call away from the excluded car in every case (" + std::to_string(moved_once) + "/" + std::to_string(interventions) + ")");
        std::cerr << "fixed-assignment contract ok (full-car case: fixed " << car0 << "/20 on car 0, default " << car0b << "/20; 3-car interventions " << interventions << ")\n";
    }

    {
        Building bb; bb.floors = 10;
        CarView car; car.x = bb.pos(9); car.committed = 9; car.dir = DOWN;
        car.load = 6; car.car_calls[0] = 6;
        HallKey hall{6, DOWN};
        double charge = onboard_stop_externality(bb, car, hall);
        auto without = run_path_full(bb, car, {});
        auto with = run_path_full(bb, car, {VCall{hall, 0, 1, 0}});
        check(std::abs(charge - 6*(with.alight.at(0)-without.alight.at(0))) < 1e-9,
              "observed stop externality: isolated insertion matches route");
        car.car_calls[0] = 1;
        check(onboard_stop_externality(bb, car, hall) == charge,
              "observed stop externality: destination multiplicities are not read");
        car.load = bb.capacity;
        check(onboard_stop_externality(bb, car, hall) == 0,
              "observed stop externality: full car cannot add a pickup");
        car.load = 6; car.car_calls[7] = 1;
        check(onboard_stop_externality(bb, car, hall) == 0,
              "observed stop externality: outside the before-first-drop case, abstain");
    }
    std::cerr << (fails ? "SELFTEST FAILED" : "selftest ok") << "\n";
    return fails ? 1 : 0;
}

}

int main(int argc, char** argv) {
    try {
        Args a = parse(argc, argv);
        if (a.cmd == "run") return cmd_run(a);
        if (a.cmd == "grid") return cmd_grid(a);
        if (a.cmd == "costs") return cmd_costs(a);
        if (a.cmd == "selftest") return cmd_selftest(a);
        if (a.cmd == "reeval-check") return cmd_reeval_check(a);
        if (a.cmd == "counterfactual") return cmd_counterfactual(a);
        if (a.cmd == "census") return cmd_census(a);
        if (a.cmd == "restricted") return cmd_restricted(a);
        std::cerr << "unknown command " << a.cmd << "\n";
        return 2;
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
}
