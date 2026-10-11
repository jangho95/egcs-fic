#pragma once
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <queue>
#include <set>
#include <string>
#include <vector>

#include "dispatchers.hpp"

namespace egcs2017 {

struct TrafficSpec;

struct Passenger {
    int id = 0;
    double arrival = 0.0;
    int origin = 0, dest = 0;
    double boarded = -1.0, alighted = -1.0;
    int car = -1;
    int dir() const { return dest > origin ? UP : DOWN; }
};

struct Flight {
    double t0 = 0.0, x0 = 0.0, v0 = 0.0;
    int dir = 0;
    int target = 0;
    bool flip = false;
    Kinematics::Profile prof;
    double end() const { return t0 + prof.duration(); }
};

struct Car {
    enum State { IDLE, MOVING, DWELL };
    int id = 0;
    State state = IDLE;
    int floor = 0;
    int dir = 0;
    Flight flight;
    double dwell_end = 0.0;
    int token = 0;
    std::map<int, std::vector<int>> car_calls;
    std::vector<int> onboard;
    std::set<HallKey> assigned;

    double stop_t0 = -1.0;
    int stop_dir = 0, stop_onboard_arrival = 0, stop_alight = 0, stop_boarded = 0;
    double last_flight_t0 = 0.0, stop_flight_t0 = 0.0;
    int load() const { return static_cast<int>(onboard.size()); }
    std::map<int, int> car_call_counts() const {
        std::map<int, int> m;
        for (auto& [f, v] : car_calls) m[f] = static_cast<int>(v.size());
        return m;
    }
};

struct CallState {
    double registered = 0.0;
    int assigned = -1;
    long episode = 0;
    bool ever_assigned = false;
};

class Trace;

struct Metrics {
    int assist_issued = 0;
    double assist_us_total = 0;
    int n = 0, unserved = 0, unfinished = 0, decisions = 0;
    double awt = 0, p95_wait = 0, max_wait = 0, att = 0, ajt = 0, p95_journey = 0;
    double decision_us_mean = 0, decision_us_p95 = 0, decision_us_max = 0;
    double sim_end = 0;
    int reassignments = 0, unassigned_retries = 0;
    double frac_over_60 = 0;

    int search_decisions = 0, search_changed_decisions = 0, search_exhaustive_decisions = 0, search_suboptimal_greedy = 0;
    int search_iter_cap_hits = 0;
    double search_g_reduction_mean = 0;

    double us_matrix_mean = 0, us_greedy_mean = 0, us_search_mean = 0, us_search_p95 = 0, us_search_max = 0;

    int reeval_decisions = 0, reeval_changed_decisions = 0;
    int reeval_removed_decisions = 0;
    double reeval_candidates_mean = 0, us_reeval_mean = 0, us_reeval_p95 = 0, us_reeval_max = 0;

    int rollout_decisions = 0, rollout_changed = 0;
    double us_rollout_mean = 0;

    int full_events = 0, full_left = 0;
    double full_extra_wait_ps = 0;
    int passed_over_passengers = 0, passed_over_max = 0;
    int episodes = 0, episodes_1 = 0, episodes_2 = 0, episodes_3p = 0;

    int stops = 0, pickup_only_stops = 0, dropoff_only_stops = 0, mixed_stops = 0;
    double onboard_dwell_ps = 0, pickup_only_onboard_ps = 0;

    int stops_win = 0, pickup_only_stops_win_down = 0, pickup_only_stops_win_up = 0;
    double onboard_dwell_ps_win = 0, pickup_only_onboard_ps_win_down = 0, pickup_only_onboard_ps_win_up = 0;
    int calls_registered = 0, idle_assignments = 0;
    double empty_travel_ps = 0;
    double queue_max = 0, queue_avg = 0;
    double drain_time = 0;
};

struct SimOptions {
    bool idle_lobby = false;
    double assist_age = -1.0;
    bool full_load_bypass = false;
    bool button_info = true;
    double theta = 5.0;
    double retry_dt = 1.0;
    bool check_invariants = false;

    enum Lock { THETA, STABILITY } lock_mode = THETA;
    double period = 0.0;
    bool check_predictions = false;

    struct Rollout {
        bool enabled = false;

        enum Future { NONE, TRUE_ARRIVALS, SAMPLED_TRUE, SAMPLED_EST } future = NONE;
        bool system = false;
        double horizon = 30.0;
        int samples = 16;
        double est_window = 300.0;
        double true_rate = 0.0;
        const TrafficSpec* spec = nullptr;
        std::string base = "paper2017_ls_lk";
        unsigned long seed = 1;

        bool diag = false;
        std::string diag_path;
        int diag_samples = 16;
        std::vector<double> diag_windows;
    } rollout;

    double win0 = 0.0, win1 = 1e300;
    bool stop_log = false;

    std::map<HallKey, int> forced_assignment;

    bool assigned_only_boarding = false;

    bool fixed_assignment = false;

    struct Intervention {
        bool active = false;
        enum Mode { REASSIGN, LATER } mode = REASSIGN;
        HallKey call{0, 0};
        long episode = -1;
        int car = -1;
        std::vector<int> onboard_at_branch;
        bool released = false;
        int forbidden_violations = 0;
        bool fix_others = false;
        int to_car = -1;

        bool redispatching = false;
    } intervention;

    std::function<void(double, int, const Building&, const std::vector<CarView>&, const std::vector<CallView>&, const HallKey&)> on_decision;
};

struct PredictionStats {
    int predicted = 0, invalidated = 0, mixed_excluded = 0, compared = 0, mismatches = 0;

    int inv_redecided = 0;
    int inv_joined = 0;
    int inv_foreign_board = 0;
    int inv_open_board = 0;
    int inv_full_release = 0;
    int inv_taken = 0;
    double worst = 0.0;
    std::string first_mismatch;
    void merge(const PredictionStats& o) {
        predicted += o.predicted; invalidated += o.invalidated; mixed_excluded += o.mixed_excluded; compared += o.compared;
        mismatches += o.mismatches; inv_redecided += o.inv_redecided; inv_joined += o.inv_joined; inv_foreign_board += o.inv_foreign_board;
        inv_open_board += o.inv_open_board; inv_full_release += o.inv_full_release; inv_taken += o.inv_taken;
        if (o.worst > worst) worst = o.worst;
        if (first_mismatch.empty()) first_mismatch = o.first_mismatch;
    }
};

class Simulation {
public:
    Simulation(Building b, int n_cars, std::unique_ptr<Dispatcher> dispatcher, std::vector<Passenger> passengers,
               Trace* trace = nullptr, double max_time = 4 * 3600.0, SimOptions options = {});
    void run();
    Metrics metrics() const;

    Simulation(const Simulation& o, std::unique_ptr<Dispatcher> dispatcher);
    struct StopRecord { double t0, duration, flight_t0; int car, floor, dir, continuing, boarded, alight; bool pickup_only, in_window; };
    const std::vector<StopRecord>& stop_log() const { return stop_log_; }
    void set_stop_log(bool on) { opt_.stop_log = on; }
    void set_assigned_only_boarding(bool on) { opt_.assigned_only_boarding = on; }
    void set_fixed_assignment(bool on) { opt_.fixed_assignment = on; }

    bool car_in_flight_to(int car, int floor, double& elapsed, double& accel_cruise) const;
    bool call_locked_by_theta(const HallKey& k) const;
    void apply_intervention(const SimOptions::Intervention& iv);
    SimOptions::Intervention& intervention() { return opt_.intervention; }

    void drop_arrivals_after(double t);
    void resample_waiting_destinations(unsigned long seed);
    double replay_cost(double arrival_cutoff, bool system) const;
    void set_max_time(double t) { max_time_ = t; }
    void inject_arrivals(const std::vector<Passenger>& extra);
    double estimated_rate(double window) const;
    const std::vector<Passenger>& passengers() const { return passengers_; }
    const std::vector<Car>& cars() const { return cars_; }
    double now() const { return now_; }

    double car_x(const Car& c, double t) const;
    double car_v(const Car& c, double t) const;
    int committed_floor(const Car& c, double t) const;
    CarView view(const Car& c, double t) const;
    const std::map<HallKey, CallState>& calls() const { return calls_; }
    const std::map<HallKey, std::vector<int>>& waiting() const { return waiting_; }
    const auto& auxiliary_targets() const { return assist_; }
    const PredictionStats& predictions() const { return pred_stats_; }

private:
    enum Kind { ARRIVAL = 0, FLIGHT_END = 1, DWELL_END = 2, RETRY = 3, TICK = 4 };
    struct Event {
        double t; int kind; int car; int token; int idx; long seq;
        bool operator>(const Event& o) const {
            if (t != o.t) return t > o.t;
            if (kind != o.kind) return kind > o.kind;
            return seq > o.seq;
        }
    };
    void push(double t, int kind, int car, int token, int idx);
    void on_arrival(int idx);
    void on_flight_end(Car& c);
    void on_dwell_end(Car& c);
    void dispatch(const HallKey& new_call);
    void schedule_retry();
    void coordinate_assist();
    int assist_issued_ = 0;
    double assist_us_total_ = 0;
    void schedule_tick();
    bool locked_by_stability(const Car& c, const HallKey& k) const;
    void check_invariants() const;
    enum InvReason { REDECIDED, JOINED, FOREIGN_BOARD, OPEN_BOARD, FULL_RELEASE, TAKEN };
    void predict(int car);
    void invalidate_predictions(int car, InvReason why);
    void count_invalidation(InvReason why, int n = 1);
    void compare_prediction(const HallKey& key, int car);
    bool doors_open(const Car& c, double t) const;
    void plan(Car& c);
    void start_flight(Car& c, int target, bool flip);
    void stop(Car& c);
    int board(Car& c, const HallKey& key);
    bool anything_ahead(const Car& c, int d) const;
    void sample_until(double t);

    Building b_;
    std::vector<Car> cars_;
    std::unique_ptr<Dispatcher> dispatcher_;
    std::vector<Passenger> passengers_;
    std::map<HallKey, CallState> calls_;
    std::map<int, std::pair<HallKey, long>> assist_;
    std::map<HallKey, std::vector<int>> waiting_;
    std::priority_queue<Event, std::vector<Event>, std::greater<Event>> events_;
    long seq_ = 0;
    double now_ = 0.0;
    double max_time_;
    Trace* trace_;
    double next_sample_ = 0.0;
    std::vector<double> decision_us_;
    std::vector<DecisionTrace> search_log_;
    int reassignments_ = 0;
    int decisions_ = 0;
    int busy_car_ = -1;
    int plan_depth_ = 0;
    SimOptions opt_;
    bool retry_pending_ = false;
    int retries_ = 0;
    size_t arrivals_left_ = 0;
    bool tick_pending_ = false;
    int ticks_ = 0;
    struct Pred { double t_abs; int car; };
    std::map<HallKey, Pred> preds_;
    PredictionStats pred_stats_;

    struct PassStat { int passed = 0; double first_pass = -1.0; };
    std::vector<PassStat> pstat_;
    std::map<HallKey, int> episode_stops_;
    double queue_integral_ = 0.0, queue_last_t_ = 0.0, last_arrival_ = 0.0, last_alight_ = 0.0;
    int full_events_ = 0, full_left_ = 0, episodes_ = 0, episodes_1_ = 0, episodes_2_ = 0, episodes_3p_ = 0;
    int stops_ = 0, pickup_only_stops_ = 0, dropoff_only_stops_ = 0, mixed_stops_ = 0, stops_win_ = 0, po_stops_win_down_ = 0, po_stops_win_up_ = 0;
    int calls_registered_ = 0, idle_assignments_ = 0;
    double onboard_dwell_ps_ = 0, pickup_only_onboard_ps_ = 0, onboard_dwell_ps_win_ = 0, po_ps_win_down_ = 0, po_ps_win_up_ = 0, empty_travel_ps_ = 0, queue_max_ = 0;
    void finish_stop(Car& c);
    std::vector<StopRecord> stop_log_;
    void check_intervention_release();
    int waiting_total() const { int n = 0; for (auto& [k, q] : waiting_) n += static_cast<int>(q.size()); return n; }
    void queue_tick();
    void queue_tick_to(double t);
    int rollout_decisions_ = 0, rollout_changed_ = 0;
    std::vector<double> rollout_us_;
    std::vector<std::pair<double, int>> boardings_;
    std::unique_ptr<std::ofstream> diag_out_;

    std::vector<std::vector<double>> candidate_costs(const std::map<HallKey, int>& base, const HallKey& nc, SimOptions::Rollout::Future future,
                                                     int worlds, double window, double rate);
    std::map<HallKey, int> rollout_choice(const std::map<HallKey, int>& base_assignment, const HallKey& new_call, int& chosen, int& base_car);
    void rollout_diagnostic(const std::map<HallKey, int>& base_assignment, const HallKey& new_call);
};

}
