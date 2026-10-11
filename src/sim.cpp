#include "sim.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <iomanip>
#include <numeric>
#include <random>
#include <sstream>
#include <stdexcept>

#include "trace.hpp"
#include "traffic.hpp"

namespace egcs2017 {

Simulation::Simulation(Building b, int n_cars, std::unique_ptr<Dispatcher> dispatcher,
                       std::vector<Passenger> passengers, Trace* trace, double max_time, SimOptions options)
    : b_(b), dispatcher_(std::move(dispatcher)), passengers_(std::move(passengers)), max_time_(max_time), trace_(trace), opt_(options) {
    if (opt_.assist_age >= 0 && (opt_.assigned_only_boarding || opt_.fixed_assignment || opt_.check_predictions))
        throw std::invalid_argument("assist requires nonexclusive, reassignable service; prediction checker unsupported");
    if (!std::isfinite(opt_.assist_age)) throw std::invalid_argument("assist age must be finite");
    if (b_.floors < 3) throw std::invalid_argument("interfloor traffic needs at least 3 floors");
    if (n_cars < 1) throw std::invalid_argument("at least one car");
    if (b_.capacity < 1) throw std::invalid_argument("capacity must be >= 1");
    if (b_.kin.v_max <= 0 || b_.kin.a_max <= 0 || b_.kin.floor_height <= 0 || b_.kin.settle < 0)
        throw std::invalid_argument("kinematic parameters must be positive (settle >= 0)");
    if (b_.doors.open < 0 || b_.doors.close < 0 || b_.doors.transfer < 0) throw std::invalid_argument("door times must be >= 0");
    if (opt_.theta < 0 || opt_.retry_dt <= 0) throw std::invalid_argument("theta >= 0 and retry_dt > 0");
    for (auto& p : passengers_)
        if (p.origin < 0 || p.origin >= b_.floors || p.dest < 0 || p.dest >= b_.floors || p.origin == p.dest)
            throw std::invalid_argument("passenger with invalid origin/destination");
    cars_.resize(n_cars);
    for (int i = 0; i < n_cars; ++i) { cars_[i].id = i; cars_[i].floor = 0; }
    std::sort(passengers_.begin(), passengers_.end(), [](const Passenger& a, const Passenger& c) { return a.arrival < c.arrival; });
    for (size_t i = 0; i < passengers_.size(); ++i) {
        passengers_[i].id = static_cast<int>(i);
        push(passengers_[i].arrival, ARRIVAL, -1, 0, static_cast<int>(i));
    }
    arrivals_left_ = passengers_.size();
    pstat_.assign(passengers_.size(), PassStat{});
    if (opt_.period < 0) throw std::invalid_argument("period must be >= 0");
    if (opt_.period > 0) schedule_tick();
    if (trace_) next_sample_ = 0.0;
}

void Simulation::schedule_tick() {
    if (opt_.period <= 0 || tick_pending_) return;
    tick_pending_ = true;
    push(now_ + opt_.period, TICK, -1, 0, -1);
}

bool Simulation::locked_by_stability(const Car& c, const HallKey& k) const {
    if (c.state != Car::MOVING) return false;
    if (c.flight.target != k.floor) return false;
    int service_dir = c.flight.flip ? -c.flight.dir : c.flight.dir;
    if (service_dir != k.dir) return false;
    bool decelerating = now_ - c.flight.t0 >= c.flight.prof.t_acc + c.flight.prof.t_cruise - 1e-9;
    bool empty = c.onboard.empty() && c.car_calls.empty();
    return decelerating || empty;
}

Simulation::Simulation(const Simulation& o, std::unique_ptr<Dispatcher> dispatcher)
    : b_(o.b_), cars_(o.cars_), dispatcher_(std::move(dispatcher)), passengers_(o.passengers_), calls_(o.calls_), assist_(o.assist_), waiting_(o.waiting_),
      events_(o.events_), seq_(o.seq_), now_(o.now_), max_time_(o.max_time_), trace_(nullptr), next_sample_(o.next_sample_),
      reassignments_(o.reassignments_), decisions_(o.decisions_), busy_car_(o.busy_car_), plan_depth_(0), opt_(o.opt_),
      retry_pending_(o.retry_pending_), retries_(o.retries_), arrivals_left_(o.arrivals_left_), tick_pending_(o.tick_pending_), ticks_(o.ticks_),
      pstat_(o.pstat_), episode_stops_(o.episode_stops_), queue_integral_(o.queue_integral_), queue_last_t_(o.queue_last_t_),
      last_arrival_(o.last_arrival_), last_alight_(o.last_alight_), full_events_(o.full_events_), full_left_(o.full_left_),
      episodes_(o.episodes_), episodes_1_(o.episodes_1_), episodes_2_(o.episodes_2_), episodes_3p_(o.episodes_3p_),
      stops_(o.stops_), pickup_only_stops_(o.pickup_only_stops_), dropoff_only_stops_(o.dropoff_only_stops_), mixed_stops_(o.mixed_stops_),
      stops_win_(o.stops_win_), po_stops_win_down_(o.po_stops_win_down_), po_stops_win_up_(o.po_stops_win_up_),
      calls_registered_(o.calls_registered_), idle_assignments_(o.idle_assignments_), onboard_dwell_ps_(o.onboard_dwell_ps_),
      pickup_only_onboard_ps_(o.pickup_only_onboard_ps_), onboard_dwell_ps_win_(o.onboard_dwell_ps_win_), po_ps_win_down_(o.po_ps_win_down_),
      po_ps_win_up_(o.po_ps_win_up_), empty_travel_ps_(o.empty_travel_ps_), queue_max_(o.queue_max_), boardings_(o.boardings_) {
    assist_issued_ = o.assist_issued_; assist_us_total_ = o.assist_us_total_;
    opt_.rollout.enabled = false;
    opt_.on_decision = nullptr;
    opt_.check_predictions = false;
}

void Simulation::drop_arrivals_after(double t) {
    std::priority_queue<Event, std::vector<Event>, std::greater<Event>> kept;
    while (!events_.empty()) {
        Event e = events_.top(); events_.pop();
        if (e.kind == ARRIVAL && e.t > t) {
            --arrivals_left_;
            passengers_[e.idx].arrival = std::numeric_limits<double>::infinity();
            continue;
        }
        kept.push(e);
    }
    events_.swap(kept);
}

void Simulation::resample_waiting_destinations(unsigned long seed) {

    std::mt19937_64 rng(seed);
    for (auto& [k, q] : waiting_) {
        std::vector<int> side;
        for (int f = 0; f < b_.floors; ++f) if ((f - k.floor) * k.dir > 0) side.push_back(f);
        for (int idx : q) passengers_[idx].dest = side[rng() % side.size()];
    }
}

double Simulation::replay_cost(double arrival_cutoff, bool system) const {
    double cost = 0.0;
    for (auto& p : passengers_) {
        if (p.arrival > arrival_cutoff) continue;
        if (p.boarded < 0) { cost += 1e6; continue; }
        cost += p.boarded - p.arrival;
        if (system) cost += (p.alighted >= 0 ? p.alighted : now_) - p.boarded;
    }
    return cost;
}

void Simulation::inject_arrivals(const std::vector<Passenger>& extra) {
    for (Passenger p : extra) {
        p.id = static_cast<int>(passengers_.size());
        p.boarded = -1.0; p.alighted = -1.0; p.car = -1;
        passengers_.push_back(p);
        pstat_.push_back(PassStat{});
        push(p.arrival, ARRIVAL, -1, 0, p.id);
        ++arrivals_left_;
    }
}

double Simulation::estimated_rate(double window) const {

    int n = 0;
    for (auto it = boardings_.rbegin(); it != boardings_.rend() && it->first > now_ - window; ++it) n += it->second;
    return window > 0 ? n / window : 0.0;
}

static std::vector<Passenger> sample_world(std::mt19937_64& rng, double now, double horizon, double rate, int floors) {
    std::vector<Passenger> out;
    if (rate <= 0.0) return out;
    std::exponential_distribution<double> gap(rate);
    std::uniform_int_distribution<int> floor(1, floors - 1);
    double t = now + gap(rng);
    while (t <= now + horizon) {
        Passenger p;
        p.arrival = t; p.origin = floor(rng);
        do { p.dest = floor(rng); } while (p.dest == p.origin);
        out.push_back(p);
        t += gap(rng);
    }
    return out;
}

std::vector<std::vector<double>> Simulation::candidate_costs(const std::map<HallKey, int>& base, const HallKey& nc,
                                                             SimOptions::Rollout::Future future, int worlds, double window, double rate) {
    const auto& ro = opt_.rollout;
    const int C = static_cast<int>(cars_.size());
    if (future == SimOptions::Rollout::NONE || future == SimOptions::Rollout::TRUE_ARRIVALS) worlds = 1;
    if (future == SimOptions::Rollout::SAMPLED_EST) rate = estimated_rate(window);
    double cutoff = future == SimOptions::Rollout::NONE ? now_ : now_ + ro.horizon;
    unsigned long base_seed = ro.seed * 1000003UL + static_cast<unsigned long>(decisions_) * 7919UL + 13UL;
    std::vector<std::vector<double>> cost(worlds, std::vector<double>(C, 0.0));
    for (int s = 0; s < worlds; ++s) {

        std::mt19937_64 wrng(base_seed + 104729UL * static_cast<unsigned long>(s));
        std::vector<Passenger> extra;
        if (future == SimOptions::Rollout::SAMPLED_TRUE && ro.spec) extra = profile_arrivals(*ro.spec, wrng, now_, now_ + ro.horizon);
        else if (future == SimOptions::Rollout::SAMPLED_TRUE || future == SimOptions::Rollout::SAMPLED_EST)
            extra = sample_world(wrng, now_, ro.horizon, rate, b_.floors);

        unsigned long dest_seed = worlds == 1 ? base_seed : wrng();
        for (int c = 0; c < C; ++c) {
            std::map<HallKey, int> a = base;
            a[nc] = c;
            Simulation clone(*this, make_dispatcher(ro.base));

            clone.drop_arrivals_after(future == SimOptions::Rollout::TRUE_ARRIVALS ? cutoff : now_);
            clone.inject_arrivals(extra);
            clone.resample_waiting_destinations(dest_seed);
            std::set<int> affected;
            for (auto& [k, car] : a) {
                auto it = clone.calls_.find(k);
                if (it == clone.calls_.end()) continue;
                int old = it->second.assigned;
                if (old == car) continue;
                if (old >= 0) { clone.cars_[old].assigned.erase(k); affected.insert(old); }
                it->second.assigned = car;
                if (car >= 0) { clone.cars_[car].assigned.insert(k); affected.insert(car); }
            }
            for (int cc : affected) clone.plan(clone.cars_[cc]);
            clone.schedule_retry();
            clone.run();
            cost[s][c] = clone.replay_cost(cutoff, ro.system);
        }
    }
    return cost;
}

std::map<HallKey, int> Simulation::rollout_choice(const std::map<HallKey, int>& base, const HallKey& nc, int& chosen, int& base_car) {
    const auto& ro = opt_.rollout;
    base_car = base.count(nc) ? base.at(nc) : -1;
    chosen = base_car;
    auto cost = candidate_costs(base, nc, ro.future, ro.samples, ro.est_window, ro.true_rate);
    const int C = static_cast<int>(cars_.size());
    std::vector<double> mean(C, 0.0);
    for (auto& row : cost) for (int c = 0; c < C; ++c) mean[c] += row[c] / cost.size();
    double best = std::numeric_limits<double>::infinity();
    for (int c = 0; c < C; ++c)

        if (mean[c] < best - 1e-9 || (std::abs(mean[c] - best) <= 1e-9 && c == base_car)) { best = mean[c]; chosen = c; }
    std::map<HallKey, int> out = base;
    out[nc] = chosen;
    return out;
}

void Simulation::rollout_diagnostic(const std::map<HallKey, int>& base, const HallKey& nc) {
    const auto& ro = opt_.rollout;
    if (!diag_out_) diag_out_ = std::make_unique<std::ofstream>(ro.diag_path, std::ios::app);
    int base_car = base.count(nc) ? base.at(nc) : -1;
    auto t0 = std::chrono::steady_clock::now();
    auto none = candidate_costs(base, nc, SimOptions::Rollout::NONE, 1, 0, 0);
    auto t1 = std::chrono::steady_clock::now();
    auto truef = candidate_costs(base, nc, SimOptions::Rollout::TRUE_ARRIVALS, 1, 0, 0);
    auto t2 = std::chrono::steady_clock::now();
    auto sampled = candidate_costs(base, nc, SimOptions::Rollout::SAMPLED_TRUE, ro.diag_samples, 0, ro.true_rate);
    auto t3 = std::chrono::steady_clock::now();
    std::ostringstream o;
    o << std::setprecision(10);
    o << "{\"t\":" << now_ << ",\"decision\":" << decisions_ << ",\"base_car\":" << base_car << ",\"waiting_calls\":" << calls_.size()
      << ",\"n_waiting\":";
    int nw = 0; for (auto& [k, q] : waiting_) nw += static_cast<int>(q.size());
    o << nw << ",\"true_rate\":" << ro.true_rate << ",\"us_none\":" << std::chrono::duration<double, std::micro>(t1 - t0).count()
      << ",\"us_true\":" << std::chrono::duration<double, std::micro>(t2 - t1).count() << ",\"us_sampled\":" << std::chrono::duration<double, std::micro>(t3 - t2).count();
    auto mat = [&](const char* name, const std::vector<std::vector<double>>& m) {
        o << ",\"" << name << "\":[";
        for (size_t s = 0; s < m.size(); ++s) { o << (s ? "," : "") << "["; for (size_t c = 0; c < m[s].size(); ++c) o << (c ? "," : "") << m[s][c]; o << "]"; }
        o << "]";
    };
    mat("none", none); mat("true", truef); mat("sampled", sampled);
    o << ",\"est\":{";
    bool first = true;
    for (double w : ro.diag_windows) {
        double rate = estimated_rate(w);
        auto est = candidate_costs(base, nc, SimOptions::Rollout::SAMPLED_EST, ro.diag_samples, w, 0);
        o << (first ? "" : ",") << "\"" << w << "\":{\"rate\":" << rate;
        mat("cost", est);
        o << "}";
        first = false;
    }
    o << "}}\n";
    *diag_out_ << o.str();
}

void Simulation::push(double t, int kind, int car, int token, int idx) {
    events_.push(Event{t, kind, car, token, idx, seq_++});
}

double Simulation::car_x(const Car& c, double t) const {
    if (c.state != Car::MOVING) return b_.pos(c.floor);
    double dt = t - c.flight.t0;
    if (dt >= c.flight.prof.moving_time()) return b_.pos(c.flight.target);
    return c.flight.x0 + c.flight.dir * c.flight.prof.position(dt);
}

bool Simulation::doors_open(const Car& c, double t) const {
    return c.state == Car::DWELL && t < c.dwell_end - b_.doors.close - 1e-9;
}

double Simulation::car_v(const Car& c, double t) const {
    if (c.state != Car::MOVING) return 0.0;
    return c.flight.prof.velocity(t - c.flight.t0);
}

int Simulation::committed_floor(const Car& c, double t) const {
    if (c.state != Car::MOVING) return c.floor;
    double x = car_x(c, t), v = car_v(c, t);
    double s = b_.kin.stop_distance(v);
    double h = b_.kin.floor_height;
    int f;
    if (c.flight.dir == UP) f = static_cast<int>(std::ceil((x + s) / h - 1e-7));
    else f = static_cast<int>(std::floor((x - s) / h + 1e-7));
    f = std::clamp(f, 0, b_.floors - 1);

    if ((f - c.flight.target) * c.flight.dir > 0) f = c.flight.target;
    return f;
}

CarView Simulation::view(const Car& c, double t) const {
    CarView v;
    v.id = c.id;
    v.x = car_x(c, t);
    v.v = car_v(c, t);

    bool settling = c.state == Car::MOVING && t - c.flight.t0 >= c.flight.prof.moving_time() - 1e-9;
    v.moving = c.state == Car::MOVING && !settling;
    v.arriving = settling;

    v.dir = v.moving ? c.flight.dir : settling ? (c.flight.flip ? -c.flight.dir : c.flight.dir) : c.dir;
    v.committed = committed_floor(c, t);
    if (settling) v.release = std::max(0.0, c.flight.end() - t);
    else if (c.state == Car::DWELL) v.release = std::max(0.0, c.dwell_end - t);
    v.doors_open = doors_open(c, t);
    v.load = c.load();
    if (opt_.button_info) { for (auto& [f, ps] : c.car_calls) v.car_calls[f] = 1; }
    else v.car_calls = c.car_call_counts();
    v.assigned.assign(c.assigned.begin(), c.assigned.end());
    return v;
}

void Simulation::run() {
    while (!events_.empty()) {
        if (events_.top().t > max_time_) break;
        Event e = events_.top();
        events_.pop();
        if (trace_) sample_until(e.t);
        queue_tick_to(e.t);
        now_ = e.t;
        if (opt_.intervention.active) check_intervention_release();
        const int registrations_before = calls_registered_;
        if (e.kind == ARRIVAL) { --arrivals_left_; on_arrival(e.idx); }
        else if (e.kind == TICK) {
            tick_pending_ = false;
            ++ticks_;
            if (!calls_.empty()) dispatch(no_new_call());
            bool busy = std::any_of(cars_.begin(), cars_.end(), [](const Car& c) { return c.state != Car::IDLE; });
            if (arrivals_left_ > 0 || !calls_.empty() || busy) schedule_tick();
        }
        else if (e.kind == RETRY) {
            retry_pending_ = false;
            for (auto& [k, st] : calls_)
                if (st.assigned < 0) { ++retries_; dispatch(k); break; }
            schedule_retry();
        } else {
            Car& c = cars_[e.car];
            if (e.token != c.token) continue;
            if (e.kind == FLIGHT_END) on_flight_end(c);
            else on_dwell_end(c);
        }

        if ((opt_.assist_age >= 0 || opt_.idle_lobby) && (e.kind != ARRIVAL || calls_registered_ != registrations_before)) {
            auto start = std::chrono::steady_clock::now(); coordinate_assist();
            assist_us_total_ += std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now()-start).count();
        }
        if (opt_.check_invariants) check_invariants();
    }
    dispatcher_->observe_learning(now_, queue_integral_, true);
}

void Simulation::coordinate_assist() {
    std::vector<int> cancel;
    for (auto it = assist_.begin(); it != assist_.end();) {
        const auto& c = cars_[it->first];
        auto cs = calls_.find(it->second.first);
        if (cs == calls_.end() || cs->second.episode != it->second.second ||
            !c.assigned.empty() || c.load() != 0) {
            cancel.push_back(it->first); it = assist_.erase(it);
        } else ++it;
    }
    for (int id : cancel) plan(cars_[id]);
    for (auto& c : cars_) {
        if (c.state != Car::IDLE || c.load() || !c.assigned.empty() || assist_.count(c.id)) continue;
        bool found = false; HallKey best; long episode = 0; double oldest = -1;
        for (const auto& [k, st] : calls_) {
            if (opt_.assist_age < 0 || st.assigned < 0 || st.assigned == c.id || now_ - st.registered < opt_.assist_age) continue;
            bool covered = false;
            for (const auto& [id, target] : assist_) if (target.first == k && target.second == st.episode) covered = true;
            if (covered) continue;

            bool loading = false;
            for (const auto& o : cars_) if (o.floor == k.floor && o.dir == k.dir && doors_open(o, now_) && o.load() < b_.capacity) loading = true;
            if (loading) continue;
            double age = now_ - st.registered;
            if (!found || age > oldest) { found = true; best = k; episode = st.episode; oldest = age; }
        }
        if (found) {
            assist_[c.id] = {best, episode}; ++assist_issued_;
            if (trace_) trace_->event(now_, "assist", "\"car\":" + std::to_string(c.id) + ",\"f\":" + std::to_string(best.floor) + ",\"episode\":" + std::to_string(episode));
            plan(c);
        } else if (opt_.idle_lobby && c.floor != 0) {
            start_flight(c, 0, false);
        }
    }
}

void Simulation::queue_tick() { queue_tick_to(now_); }
void Simulation::queue_tick_to(double t) {
    int n = waiting_total();
    queue_integral_ += n * (t - queue_last_t_);
    queue_last_t_ = t;
    if (n > queue_max_) queue_max_ = n;
}

void Simulation::schedule_retry() {
    if (retry_pending_ || opt_.period > 0) return;
    for (auto& [k, st] : calls_)
        if (st.assigned < 0) { retry_pending_ = true; push(now_ + opt_.retry_dt, RETRY, -1, 0, -1); return; }
}

void Simulation::check_invariants() const {
    std::set<std::pair<HallKey,long>> helper_episodes;
    for (const auto& [id, target] : assist_) {
        auto cs = calls_.find(target.first);
        if (cs == calls_.end() || cs->second.episode != target.second || cars_[id].load() || !cars_[id].assigned.empty())
            throw std::logic_error("stale auxiliary service request");
        if (!helper_episodes.insert(target).second) throw std::logic_error("duplicate auxiliary responder");
    }
    for (auto& [k, q] : waiting_) {
        if (q.empty()) continue;

        bool open_here = false;
        for (auto& c : cars_) if (doors_open(c, now_) && c.floor == k.floor && c.dir == k.dir && c.load() < b_.capacity) open_here = true;
        if (!calls_.count(k) && !open_here) throw std::logic_error("waiting passengers without a registered call");
    }
    for (auto& [k, st] : calls_) {
        if (!waiting_.count(k) || waiting_.at(k).empty()) throw std::logic_error("registered call without waiting passengers");
        if (st.assigned >= 0 && !cars_[st.assigned].assigned.count(k)) throw std::logic_error("assigned car does not hold the call");
    }
    for (auto& c : cars_)
        for (auto& k : c.assigned)
            if (!calls_.count(k) || calls_.at(k).assigned != c.id) throw std::logic_error("stale assignment on a car");
}

void Simulation::sample_until(double t) {
    while (next_sample_ <= t + 1e-9) {
        std::string cars = "[";
        for (size_t i = 0; i < cars_.size(); ++i) {
            const Car& c = cars_[i];
            cars += (i ? "," : "");
            cars += "{\"x\":" + fmt(car_x(c, next_sample_)) + ",\"v\":" + fmt(car_v(c, next_sample_) * (c.state == Car::MOVING ? c.flight.dir : 1)) +
                    ",\"dir\":" + std::to_string(c.state == Car::MOVING ? c.flight.dir : c.dir) + ",\"load\":" + std::to_string(c.load()) +
                    ",\"state\":" + std::to_string(static_cast<int>(c.state)) +
                    ",\"target\":" + std::to_string(c.state == Car::MOVING ? c.flight.target : c.floor) + ",\"cc\":[";
            bool first = true;
            for (auto& [f, v] : c.car_calls) { cars += (first ? "" : ",") + std::to_string(f); first = false; }
            cars += "]}";
        }
        cars += "]";
        std::string calls = "[";
        bool first = true;
        for (auto& [k, st] : calls_) {
            int w = 0;
            if (auto it = waiting_.find(k); it != waiting_.end()) w = static_cast<int>(it->second.size());
            calls += (first ? "" : ",") + std::string("[") + std::to_string(k.floor) + "," + std::to_string(k.dir) + "," +
                     std::to_string(st.assigned) + "," + std::to_string(w) + "]";
            first = false;
        }
        calls += "]";
        trace_->sample(next_sample_, cars, calls);
        next_sample_ += trace_->sample_dt();
    }
}

void Simulation::on_arrival(int idx) {
    Passenger& p = passengers_[idx];
    HallKey key{p.origin, p.dir()};
    waiting_[key].push_back(idx);
    last_arrival_ = std::max(last_arrival_, now_);
    if (opt_.check_predictions && waiting_[key].size() > 1) {

        count_invalidation(JOINED, static_cast<int>(preds_.erase(key)));
        if (auto cs = calls_.find(key); cs != calls_.end() && cs->second.assigned >= 0) invalidate_predictions(cs->second.assigned, JOINED);
    }
    if (trace_) trace_->event(now_, "arrive", "\"p\":" + std::to_string(idx) + ",\"o\":" + std::to_string(p.origin) + ",\"d\":" + std::to_string(p.dest));

    for (auto& c : cars_)
        if (doors_open(c, now_) && c.floor == key.floor && c.dir == key.dir && c.load() < b_.capacity && !calls_.count(key)) {
            int n = board(c, key);
            if (n > 0) {
                c.dwell_end += b_.doors.transfer * n;
                c.stop_boarded += n;
                ++c.token;
                push(c.dwell_end, DWELL_END, c.id, c.token, -1);
                if (opt_.check_predictions) invalidate_predictions(c.id, OPEN_BOARD);
                if (trace_) trace_->event(now_, "board_open", "\"car\":" + std::to_string(c.id) + ",\"f\":" + std::to_string(key.floor) + ",\"n\":" + std::to_string(n));
                return;
            }
        }
    if (!calls_.count(key)) {
        calls_[key] = CallState{now_, -1, static_cast<long>(calls_registered_)};
        ++calls_registered_;
        episode_stops_[key] = 0;
        if (trace_) trace_->event(now_, "call", "\"f\":" + std::to_string(key.floor) + ",\"dir\":" + std::to_string(key.dir));
        if (opt_.period > 0) schedule_tick();
        else dispatch(key);
    }
}

void Simulation::dispatch(const HallKey& new_call) {
    std::vector<CarView> views;
    for (auto& c : cars_) views.push_back(view(c, now_));
    std::vector<CallView> cviews;
    for (auto& [k, st] : calls_) {
        CallView cv;
        cv.key = k; cv.registered = st.registered; cv.assigned = st.assigned;
        cv.waiting = static_cast<int>(waiting_[k].size());
        cv.waited = now_ - st.registered;
        cv.close_to_service = false;
        {
            const auto& iv = opt_.intervention;
            if (iv.active && !iv.released && k == iv.call && st.episode == iv.episode) {
                if (iv.mode == SimOptions::Intervention::REASSIGN) cv.forbidden_car = iv.car;
                else if (st.assigned == iv.car) cv.close_to_service = true;
            }

            if (iv.active && !iv.released && iv.fix_others && !(k == iv.call) && st.assigned >= 0) cv.close_to_service = true;
        }
        if (st.assigned >= 0 && !(k == new_call)) {
            if (opt_.lock_mode == SimOptions::STABILITY) {

                cv.close_to_service = cv.close_to_service || locked_by_stability(cars_[st.assigned], k);
            } else {

                CostOptions raw;
                double eta = unary(b_, views[st.assigned], k, raw);
                if (eta <= opt_.theta) cv.close_to_service = true;
            }
        }
        if (opt_.fixed_assignment && st.assigned >= 0) {

            const auto& iv = opt_.intervention;
            bool target = iv.active && iv.redispatching && k == iv.call && st.episode == iv.episode;
            if (!target) cv.close_to_service = true;
        }
        cviews.push_back(cv);
    }
    DecisionTrace dt;
    dt.t = now_;
    if (opt_.on_decision) opt_.on_decision(now_, decisions_, b_, views, cviews, new_call);
    auto t0 = std::chrono::steady_clock::now();
    dispatcher_->observe_learning(now_, queue_integral_, false);
    auto assignment = dispatcher_->decide(b_, views, cviews, new_call, &dt);
    if (opt_.intervention.active && opt_.intervention.redispatching && opt_.intervention.to_car >= 0 && calls_.count(opt_.intervention.call))
        assignment[opt_.intervention.call] = opt_.intervention.to_car;
    if (!opt_.forced_assignment.empty()) {

        for (auto& [k, car] : opt_.forced_assignment) {
            auto it = calls_.find(k);
            if (it == calls_.end() || it->second.ever_assigned) continue;
            if (opt_.intervention.active && k == opt_.intervention.call) continue;
            assignment[k] = car;
        }
    }
    if (opt_.rollout.enabled && new_call.floor >= 0 && calls_.count(new_call) && busy_car_ < 0 && opt_.period <= 0) {
        auto tr0 = std::chrono::steady_clock::now();
        if (opt_.rollout.diag) {
            rollout_diagnostic(assignment, new_call);
        } else {
            int chosen = -1, base_car = -1;
            assignment = rollout_choice(assignment, new_call, chosen, base_car);
            if (chosen != base_car) ++rollout_changed_;
        }
        ++rollout_decisions_;
        rollout_us_.push_back(std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - tr0).count());
    }
    auto t1 = std::chrono::steady_clock::now();
    if (dt.search_mode > 0 || dt.us_matrix > 0.0) {
        DecisionTrace light;
        light.t = dt.t; light.g_greedy = dt.g_greedy; light.g_final = dt.g_final; light.search_mode = dt.search_mode;
        light.search_iterations = dt.search_iterations; light.search_changed = dt.search_changed; light.search_cap_hit = dt.search_cap_hit;
        light.us_matrix = dt.us_matrix; light.us_greedy = dt.us_greedy; light.us_search = dt.us_search;
        light.reeval_candidates = dt.reeval_candidates; light.reeval_pick = dt.reeval_pick; light.reeval_removed = dt.reeval_removed;
        light.us_reeval = dt.us_reeval;
        search_log_.push_back(light);
    }
    double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
    decision_us_.push_back(us);
    ++decisions_;
    {
        auto& iv = opt_.intervention;
        if (iv.active && !iv.released && iv.mode == SimOptions::Intervention::REASSIGN) {
            auto ait = assignment.find(iv.call);
            auto cit = calls_.find(iv.call);
            if (ait != assignment.end() && ait->second == iv.car && cit != calls_.end() && cit->second.episode == iv.episode) {

                ++iv.forbidden_violations;
                throw std::logic_error("counterfactual: the excluded car received the call");
            }
        }
    }
    std::set<int> affected;
    for (auto& [k, car] : assignment) {
        auto it = calls_.find(k);
        if (it == calls_.end()) continue;
        int old = it->second.assigned;
        if (old == car) continue;
        if (old >= 0) { cars_[old].assigned.erase(k); affected.insert(old); }
        if (old >= 0 && !(k == new_call)) ++reassignments_;
        if (car >= 0 && k == new_call && cars_[car].state == Car::IDLE && cars_[car].assigned.empty() && cars_[car].car_calls.empty()) {
            ++idle_assignments_;
            empty_travel_ps_ += b_.kin.travel(cars_[car].floor, k.floor) * static_cast<double>(waiting_[k].size());
        }
        it->second.assigned = car;
        if (car >= 0) { it->second.ever_assigned = true; cars_[car].assigned.insert(k); affected.insert(car); }
        if (trace_)
            trace_->event(now_, car < 0 ? "unassign" : old >= 0 ? "reassign" : "assign",
                          "\"f\":" + std::to_string(k.floor) + ",\"dir\":" + std::to_string(k.dir) + ",\"car\":" + std::to_string(car) +
                              ",\"prev\":" + std::to_string(old));
    }
    if (trace_) { dt.micros = us; trace_->decision(dt); }
    for (int c : affected)
        if (c != busy_car_) plan(cars_[c]);
    schedule_retry();
    if (opt_.check_predictions)
        for (int c : affected) predict(c);
}

void Simulation::predict(int car) {
    Car& c = cars_[car];
    for (auto& k : c.assigned) count_invalidation(REDECIDED, static_cast<int>(preds_.erase(k)));
    if (busy_car_ == car) return;

    CarView v = view(c, now_);
    v.car_calls = c.car_call_counts();
    std::vector<VCall> vc;
    int id = 0;
    for (auto& k : c.assigned) {
        auto it = waiting_.find(k);
        if (it == waiting_.end() || it->second.empty()) return;
        int dest = passengers_[it->second.front()].dest;
        for (int idx : it->second)
            if (passengers_[idx].dest != dest) { ++pred_stats_.mixed_excluded; return; }
        vc.push_back(VCall{k, dest, static_cast<int>(it->second.size()), id++});
    }
    if (vc.empty()) return;
    PathResult r = run_path(b_, v, vc, 0);
    for (auto& call : vc)
        if (r.pickup[call.id] >= 0) { preds_[call.key] = Pred{now_ + r.pickup[call.id], car}; ++pred_stats_.predicted; }
}

void Simulation::count_invalidation(InvReason why, int n) {
    if (n <= 0) return;
    pred_stats_.invalidated += n;
    switch (why) {
        case REDECIDED: pred_stats_.inv_redecided += n; break;
        case JOINED: pred_stats_.inv_joined += n; break;
        case FOREIGN_BOARD: pred_stats_.inv_foreign_board += n; break;
        case OPEN_BOARD: pred_stats_.inv_open_board += n; break;
        case FULL_RELEASE: pred_stats_.inv_full_release += n; break;
        case TAKEN: pred_stats_.inv_taken += n; break;
    }
}

void Simulation::invalidate_predictions(int car, InvReason why) {
    for (auto it = preds_.begin(); it != preds_.end();) {
        if (it->second.car == car) { it = preds_.erase(it); count_invalidation(why); }
        else ++it;
    }
}

void Simulation::compare_prediction(const HallKey& key, int car) {
    auto it = preds_.find(key);
    if (it == preds_.end()) return;
    if (it->second.car != car) count_invalidation(TAKEN);
    if (it->second.car == car) {
        double diff = std::abs(it->second.t_abs - now_);
        ++pred_stats_.compared;
        if (diff > 1e-6) {
            ++pred_stats_.mismatches;
            pred_stats_.worst = std::max(pred_stats_.worst, diff);
            if (pred_stats_.first_mismatch.empty())
                pred_stats_.first_mismatch = "call (" + std::to_string(key.floor) + "," + std::to_string(key.dir) + ") car " + std::to_string(car) +
                                             " predicted " + std::to_string(it->second.t_abs) + " realized " + std::to_string(now_);
        }
    }
    preds_.erase(it);
}

bool Simulation::anything_ahead(const Car& c, int d) const {
    for (auto& [f, v] : c.car_calls) if ((f - c.floor) * d > 0) return true;
    for (auto& k : c.assigned) if ((k.floor - c.floor) * d > 0) return true;
    return false;
}

void Simulation::start_flight(Car& c, int target, bool flip) {
    double x = car_x(c, now_), v = car_v(c, now_);
    int dir = c.state == Car::MOVING ? c.flight.dir : (target > c.floor ? UP : DOWN);
    double dist = std::abs(b_.pos(target) - x);
    Flight f;
    f.t0 = now_; f.x0 = x; f.v0 = v; f.dir = dir; f.target = target; f.flip = flip;
    f.prof = b_.kin.profile(dist, v);
    bool was_moving = c.state == Car::MOVING;
    int prev_target = was_moving ? c.flight.target : c.floor;
    c.flight = f;
    c.state = Car::MOVING;
    c.dir = dir;
    c.last_flight_t0 = now_;
    ++c.token;
    push(f.end(), FLIGHT_END, c.id, c.token, -1);
    if (trace_)
        trace_->event(now_, was_moving ? "retarget" : "depart",
                      "\"car\":" + std::to_string(c.id) + ",\"from\":" + std::to_string(prev_target) + ",\"to\":" + std::to_string(target) +
                          ",\"eta\":" + fmt(f.end()));
}

void Simulation::plan(Car& c) {
    if (c.state == Car::DWELL) return;
    if (c.state == Car::MOVING && now_ - c.flight.t0 >= c.flight.prof.moving_time() - 1e-9) return;
    struct Depth { int& d; Depth(int& dd) : d(dd) { ++d; } ~Depth() { --d; } } dg{plan_depth_};
    if (plan_depth_ > 40) {
        std::string msg = "plan recursion: car " + std::to_string(c.id) + " floor " + std::to_string(c.floor) + " dir " + std::to_string(c.dir) +
                          " load " + std::to_string(c.load()) + " state " + std::to_string(static_cast<int>(c.state)) + " assigned:";
        for (auto& k : c.assigned) msg += " (" + std::to_string(k.floor) + "," + std::to_string(k.dir) + ")w" + std::to_string(waiting_.count(k) ? waiting_.at(k).size() : 0) +
                                          "own" + std::to_string(calls_.count(k) ? calls_.at(k).assigned : -9);
        msg += " carcalls:";
        for (auto& [f, v] : c.car_calls) msg += " " + std::to_string(f) + "x" + std::to_string(v.size());
        throw std::runtime_error(msg);
    }
    std::vector<HallKey> pending(c.assigned.begin(), c.assigned.end());
    if (auto ai = assist_.find(c.id); ai != assist_.end()) {
        auto cs = calls_.find(ai->second.first);
        if (cs != calls_.end() && cs->second.episode == ai->second.second && c.assigned.empty() && c.load() == 0)
            pending.push_back(ai->second.first);
    }
    {
        const auto& iv = opt_.intervention;
        if (iv.active && !iv.released && iv.mode == SimOptions::Intervention::LATER && iv.car == c.id) {
            auto it = calls_.find(iv.call);
            if (it != calls_.end() && it->second.episode == iv.episode)
                pending.erase(std::remove(pending.begin(), pending.end(), iv.call), pending.end());
        }
    }
    auto cc = c.car_call_counts();

    if (opt_.full_load_bypass && c.load() >= b_.capacity) pending.clear();
    if (c.state == Car::MOVING) {
        int committed = committed_floor(c, now_);
        Move m = next_move(committed, c.flight.dir, cc, pending);
        int target = committed;
        bool flip = false;
        if (m.kind == Move::LEG || m.kind == Move::TAIL_OPP) { target = m.target; flip = m.kind == Move::TAIL_OPP; }
        if (target != c.flight.target || flip != c.flight.flip) start_flight(c, target, flip);
        return;
    }

    if (c.load() >= b_.capacity)
        pending.erase(std::remove_if(pending.begin(), pending.end(), [&](const HallKey& k) { return k.floor == c.floor; }), pending.end());
    for (int guard = 0; guard < 4; ++guard) {
        if (cc.empty() && pending.empty()) { c.dir = 0; c.state = Car::IDLE; return; }
        int dir = c.dir != 0 ? c.dir : initial_direction(c.floor, cc, pending);
        Move m = next_move(c.floor, dir, cc, pending);
        if (m.kind == Move::HALT) { c.dir = 0; c.state = Car::IDLE; return; }
        if (m.kind == Move::TAIL_BEHIND) { c.dir = m.dir; continue; }
        c.dir = dir;
        if (m.target == c.floor) {
            if (m.kind == Move::TAIL_OPP) c.dir = -dir;
            stop(c);
            return;
        }
        start_flight(c, m.target, m.kind == Move::TAIL_OPP);
        return;
    }
    throw std::runtime_error("plan did not settle");
}

int Simulation::board(Car& c, const HallKey& key) {

    auto it = waiting_.find(key);
    if (it == waiting_.end()) return 0;
    if (opt_.assigned_only_boarding) {
        auto cs = calls_.find(key);
        if (cs != calls_.end() && cs->second.assigned != c.id) return 0;
    }
    if (opt_.check_predictions && !it->second.empty()) {
        compare_prediction(key, c.id);

        auto cs = calls_.find(key);
        if (cs == calls_.end() || cs->second.assigned != c.id) invalidate_predictions(c.id, FOREIGN_BOARD);
    }
    int n = 0;
    auto& q = it->second;
    while (!q.empty() && c.load() < b_.capacity) {
        int idx = q.front();
        q.erase(q.begin());
        Passenger& p = passengers_[idx];
        p.boarded = now_;
        p.car = c.id;
        c.onboard.push_back(idx);
        c.car_calls[p.dest].push_back(idx);
        ++n;
        if (trace_) trace_->event(now_, "board", "\"p\":" + std::to_string(idx) + ",\"car\":" + std::to_string(c.id) + ",\"f\":" + std::to_string(key.floor));
    }
    if (n > 0) { assist_.erase(c.id); dispatcher_->observe_boarding(now_, key.floor, key.dir, n); boardings_.emplace_back(now_, n); }
    if (q.empty()) {

        if (auto es = episode_stops_.find(key); es != episode_stops_.end()) {
            int k = es->second;
            ++episodes_; if (k <= 1) ++episodes_1_; else if (k == 2) ++episodes_2_; else ++episodes_3p_;
            episode_stops_.erase(es);
        }
        waiting_.erase(it);
        calls_.erase(key);
        std::vector<int> canceled;
        for (auto ai = assist_.begin(); ai != assist_.end();) {
            if (ai->second.first == key) { canceled.push_back(ai->first); ai = assist_.erase(ai); }
            else ++ai;
        }
        for (int id : canceled) if (id != c.id && id != busy_car_) plan(cars_[id]);
        if (trace_) trace_->event(now_, "clear", "\"f\":" + std::to_string(key.floor) + ",\"dir\":" + std::to_string(key.dir) + ",\"car\":" + std::to_string(c.id));

        for (auto& o : cars_)
            if (o.assigned.erase(key) && o.id != c.id) {
                if (opt_.check_predictions) invalidate_predictions(o.id, TAKEN);
                if (o.id != busy_car_) plan(o);
            }
    }
    return n;
}

void Simulation::stop(Car& c) {
    busy_car_ = c.id;
    struct Reset { int& r; ~Reset() { r = -1; } } reset{busy_car_};
    int f = c.floor, d = c.dir;
    int alight = 0;
    const int onboard_before = c.load();
    if (auto it = c.car_calls.find(f); it != c.car_calls.end()) {
        for (int idx : it->second) {
            passengers_[idx].alighted = now_;
            last_alight_ = std::max(last_alight_, now_);
            c.onboard.erase(std::find(c.onboard.begin(), c.onboard.end(), idx));
            if (trace_) trace_->event(now_, "alight", "\"p\":" + std::to_string(idx) + ",\"car\":" + std::to_string(c.id) + ",\"f\":" + std::to_string(f));
        }
        alight = static_cast<int>(it->second.size());
        c.car_calls.erase(it);
    }
    auto pass_over = [&](const HallKey& k) {
        auto w = waiting_.find(k);
        if (w == waiting_.end() || w->second.empty()) return 0;
        for (int idx : w->second) { auto& ps = pstat_[idx]; if (ps.passed == 0) ps.first_pass = now_; ++ps.passed; }
        return static_cast<int>(w->second.size());
    };

    auto attempt = [&](const HallKey& k) { if (calls_.count(k)) { auto es = episode_stops_.find(k); if (es != episode_stops_.end()) ++es->second; } };
    attempt(HallKey{f, d});
    int boarded = board(c, HallKey{f, d});
    int left = pass_over(HallKey{f, d});
    if (!anything_ahead(c, d) && c.assigned.count(HallKey{f, -d})) {
        c.dir = d = -d;
        attempt(HallKey{f, d});
        boarded += board(c, HallKey{f, d});
        left += pass_over(HallKey{f, d});
    }
    if (left > 0 && c.load() >= b_.capacity) { ++full_events_; full_left_ += left; }
    double dwell = b_.doors.dwell(boarded, alight);
    if (dwell > 0.0) {

        c.stop_t0 = now_; c.stop_dir = d; c.stop_onboard_arrival = onboard_before; c.stop_alight = alight; c.stop_boarded = boarded;
        c.stop_flight_t0 = c.last_flight_t0;
    }
    if (trace_) trace_->event(now_, "stop", "\"car\":" + std::to_string(c.id) + ",\"f\":" + std::to_string(f) + ",\"dir\":" + std::to_string(d) +
                                            ",\"board\":" + std::to_string(boarded) + ",\"alight\":" + std::to_string(alight) + ",\"dwell\":" + fmt(dwell));

    for (int dd : {d, -d}) {
        HallKey k{f, dd};
        auto it = waiting_.find(k);
        if (it != waiting_.end() && !it->second.empty() && c.assigned.count(k) && c.load() >= b_.capacity) {
            if (opt_.fixed_assignment) {

                if (trace_) trace_->event(now_, "full", "\"car\":" + std::to_string(c.id) + ",\"f\":" + std::to_string(f) + ",\"dir\":" + std::to_string(dd) + ",\"kept\":true");
                continue;
            }
            for (auto& o : cars_) o.assigned.erase(k);
            calls_[k].assigned = -1;
            if (opt_.check_predictions) invalidate_predictions(c.id, FULL_RELEASE);
            if (trace_) trace_->event(now_, "full", "\"car\":" + std::to_string(c.id) + ",\"f\":" + std::to_string(f) + ",\"dir\":" + std::to_string(dd));
            if (opt_.period > 0) schedule_tick();
            else dispatch(k);
        }
    }
    if (dwell <= 0.0) {
        c.state = Car::IDLE;
        plan(c);
        return;
    }
    c.state = Car::DWELL;
    c.dwell_end = now_ + dwell;
    ++c.token;
    push(c.dwell_end, DWELL_END, c.id, c.token, -1);
}

void Simulation::on_flight_end(Car& c) {
    c.floor = c.flight.target;
    c.state = Car::IDLE;
    if (c.flight.flip) c.dir = -c.flight.dir;
    else c.dir = c.flight.dir;
    if (trace_) trace_->event(now_, "arrive_car", "\"car\":" + std::to_string(c.id) + ",\"f\":" + std::to_string(c.floor));
    stop(c);
}

void Simulation::finish_stop(Car& c) {
    if (c.stop_t0 < 0.0) return;
    double duration = now_ - c.stop_t0;
    int continuing = c.stop_onboard_arrival - c.stop_alight;
    bool pickup_only = c.stop_boarded > 0 && c.stop_alight == 0;
    bool dropoff_only = c.stop_boarded == 0;
    bool in_win = c.stop_t0 >= opt_.win0 && c.stop_t0 < opt_.win1;
    ++stops_;
    if (pickup_only) ++pickup_only_stops_; else if (dropoff_only) ++dropoff_only_stops_; else ++mixed_stops_;
    onboard_dwell_ps_ += duration * continuing;
    if (pickup_only) pickup_only_onboard_ps_ += duration * continuing;
    if (in_win) {
        ++stops_win_;
        onboard_dwell_ps_win_ += duration * continuing;
        if (pickup_only && c.stop_dir == DOWN) { ++po_stops_win_down_; po_ps_win_down_ += duration * continuing; }
        if (pickup_only && c.stop_dir == UP) { ++po_stops_win_up_; po_ps_win_up_ += duration * continuing; }
    }
    if (opt_.stop_log) stop_log_.push_back(StopRecord{c.stop_t0, duration, c.stop_flight_t0, c.id, c.floor, c.stop_dir, continuing, c.stop_boarded, c.stop_alight, pickup_only, in_win});
    c.stop_t0 = -1.0;
}

bool Simulation::car_in_flight_to(int car, int floor, double& elapsed, double& accel_cruise) const {
    const Car& c = cars_[car];
    if (c.state != Car::MOVING || c.flight.target != floor) return false;
    elapsed = now_ - c.flight.t0;
    accel_cruise = c.flight.prof.t_acc + c.flight.prof.t_cruise;
    return true;
}

bool Simulation::call_locked_by_theta(const HallKey& k) const {
    auto it = calls_.find(k);
    if (it == calls_.end() || it->second.assigned < 0) return false;
    CostOptions raw;
    return unary(b_, view(cars_[it->second.assigned], now_), k, raw) <= opt_.theta;
}

void Simulation::apply_intervention(const SimOptions::Intervention& iv) {
    opt_.intervention = iv;
    opt_.intervention.active = true;
    opt_.intervention.released = false;
    opt_.intervention.forbidden_violations = 0;
    { auto it = calls_.find(iv.call); opt_.intervention.episode = it != calls_.end() ? it->second.episode : -1; }
    Car& c = cars_[iv.car];
    opt_.intervention.onboard_at_branch = c.onboard;
    if (iv.mode == SimOptions::Intervention::REASSIGN) {

        auto it = calls_.find(iv.call);
        if (it != calls_.end() && it->second.assigned == iv.car) { c.assigned.erase(iv.call); it->second.assigned = -1; }
        plan(c);
        opt_.intervention.redispatching = true;
        dispatch(iv.call);
        opt_.intervention.redispatching = false;
    } else {
        plan(c);
    }
}

void Simulation::check_intervention_release() {
    auto& iv = opt_.intervention;
    if (!iv.active || iv.released) return;
    if (iv.mode == SimOptions::Intervention::REASSIGN) {
        auto it = calls_.find(iv.call);
        if (it == calls_.end() || it->second.episode != iv.episode) iv.released = true;
    } else {
        bool all_off = true;
        for (int idx : iv.onboard_at_branch) if (passengers_[idx].alighted < 0) all_off = false;
        if (all_off) { iv.released = true; if (cars_[iv.car].state != Car::DWELL && busy_car_ != iv.car) plan(cars_[iv.car]); }
    }
}

void Simulation::on_dwell_end(Car& c) {

    finish_stop(c);
    c.state = Car::IDLE;
    plan(c);
}

Metrics Simulation::metrics() const {
    Metrics m;
    m.assist_issued = assist_issued_; m.assist_us_total = assist_us_total_;
    std::vector<double> waits, journeys, rides;
    for (auto& p : passengers_) {
        ++m.n;
        if (p.boarded < 0) { ++m.unserved; continue; }
        waits.push_back(p.boarded - p.arrival);
        if (p.alighted < 0) { ++m.unfinished; continue; }
        journeys.push_back(p.alighted - p.arrival);
        rides.push_back(p.alighted - p.boarded);
    }
    auto q = [](std::vector<double> v, double qq) {
        if (v.empty()) return 0.0;
        std::sort(v.begin(), v.end());
        size_t k = static_cast<size_t>(std::lround(qq * (v.size() - 1)));
        return v[std::min(k, v.size() - 1)];
    };
    if (!waits.empty()) {
        m.awt = std::accumulate(waits.begin(), waits.end(), 0.0) / waits.size();
        m.p95_wait = q(waits, 0.95);
        m.max_wait = *std::max_element(waits.begin(), waits.end());
        m.frac_over_60 = std::count_if(waits.begin(), waits.end(), [](double w) { return w > 60.0; }) / static_cast<double>(waits.size());
    }
    if (!journeys.empty()) {
        m.ajt = std::accumulate(journeys.begin(), journeys.end(), 0.0) / journeys.size();
        m.att = std::accumulate(rides.begin(), rides.end(), 0.0) / rides.size();
        m.p95_journey = q(journeys, 0.95);
    }
    m.decisions = decisions_;
    if (!decision_us_.empty()) {
        m.decision_us_mean = std::accumulate(decision_us_.begin(), decision_us_.end(), 0.0) / decision_us_.size();
        m.decision_us_p95 = q(decision_us_, 0.95);
        m.decision_us_max = *std::max_element(decision_us_.begin(), decision_us_.end());
    }
    m.sim_end = now_;
    m.reassignments = reassignments_;
    m.unassigned_retries = retries_;
    double red = 0.0;
    std::vector<double> us_s;
    double us_m = 0.0, us_g = 0.0;
    int timed = 0;
    for (auto& d : search_log_) {
        if (d.us_matrix > 0.0) { ++timed; us_m += d.us_matrix; us_g += d.us_greedy; us_s.push_back(d.us_search); }
        if (d.search_mode == 0) continue;
        ++m.search_decisions;
        if (d.search_changed > 0) ++m.search_changed_decisions;
        if (d.search_cap_hit) ++m.search_iter_cap_hits;
        if (d.search_mode == 2) { ++m.search_exhaustive_decisions; if (d.g_final < d.g_greedy - 1e-9) ++m.search_suboptimal_greedy; }
        red += d.g_greedy - d.g_final;
    }
    if (m.search_decisions) m.search_g_reduction_mean = red / m.search_decisions;
    if (timed) {
        m.us_matrix_mean = us_m / timed; m.us_greedy_mean = us_g / timed;
        m.us_search_mean = std::accumulate(us_s.begin(), us_s.end(), 0.0) / us_s.size();
        m.us_search_p95 = q(us_s, 0.95);
        m.us_search_max = *std::max_element(us_s.begin(), us_s.end());
    }
    std::vector<double> us_r;
    double cands = 0.0;
    for (auto& d : search_log_) {
        if (d.reeval_candidates <= 0) continue;
        ++m.reeval_decisions;
        if (d.reeval_pick > 0) ++m.reeval_changed_decisions;
        if (d.reeval_removed > 0) ++m.reeval_removed_decisions;
        cands += d.reeval_candidates;
        us_r.push_back(d.us_reeval);
    }
    m.full_events = full_events_; m.full_left = full_left_;
    for (size_t i = 0; i < passengers_.size(); ++i) {
        const auto& ps = pstat_[i];
        if (ps.passed > 0) {
            ++m.passed_over_passengers;
            m.passed_over_max = std::max(m.passed_over_max, ps.passed);
            if (passengers_[i].boarded >= 0) m.full_extra_wait_ps += passengers_[i].boarded - ps.first_pass;
        }
    }
    m.episodes = episodes_; m.episodes_1 = episodes_1_; m.episodes_2 = episodes_2_; m.episodes_3p = episodes_3p_;
    m.stops = stops_; m.pickup_only_stops = pickup_only_stops_; m.dropoff_only_stops = dropoff_only_stops_; m.mixed_stops = mixed_stops_;
    m.onboard_dwell_ps = onboard_dwell_ps_; m.pickup_only_onboard_ps = pickup_only_onboard_ps_;
    m.stops_win = stops_win_; m.pickup_only_stops_win_down = po_stops_win_down_; m.pickup_only_stops_win_up = po_stops_win_up_;
    m.onboard_dwell_ps_win = onboard_dwell_ps_win_; m.pickup_only_onboard_ps_win_down = po_ps_win_down_; m.pickup_only_onboard_ps_win_up = po_ps_win_up_;
    m.calls_registered = calls_registered_; m.idle_assignments = idle_assignments_; m.empty_travel_ps = empty_travel_ps_;
    m.queue_max = queue_max_;
    m.queue_avg = now_ > 0 ? queue_integral_ / now_ : 0.0;
    m.drain_time = std::max(0.0, last_alight_ - last_arrival_);
    m.rollout_decisions = rollout_decisions_;
    m.rollout_changed = rollout_changed_;
    if (!rollout_us_.empty()) m.us_rollout_mean = std::accumulate(rollout_us_.begin(), rollout_us_.end(), 0.0) / rollout_us_.size();
    if (m.reeval_decisions) {
        m.reeval_candidates_mean = cands / m.reeval_decisions;
        m.us_reeval_mean = std::accumulate(us_r.begin(), us_r.end(), 0.0) / us_r.size();
        m.us_reeval_p95 = q(us_r, 0.95);
        m.us_reeval_max = *std::max_element(us_r.begin(), us_r.end());
    }
    return m;
}

}
