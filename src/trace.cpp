#include "trace.hpp"

#include <cmath>
#include <cstdio>

namespace egcs2017 {

std::string fmt(double x) {
    if (!std::isfinite(x)) return "null";
    char buf[32];
    std::snprintf(buf, sizeof buf, "%.7g", x);

    return buf;
}

std::string json_key(const HallKey& k) { return "[" + std::to_string(k.floor) + "," + std::to_string(k.dir) + "]"; }

Trace::Trace(const std::string& path, double sample_dt, int max_decisions)
    : out_(path), dt_(sample_dt), max_decisions_(max_decisions) {
    if (!out_) throw std::runtime_error("cannot open trace file " + path);
}

Trace::~Trace() = default;

void Trace::meta(const std::string& body) { meta_ = body; }

void Trace::event(double t, const std::string& type, const std::string& fields) {
    events_ += (first_event_ ? "" : ",");
    first_event_ = false;
    events_ += "{\"t\":" + fmt(t) + ",\"type\":\"" + type + "\"" + (fields.empty() ? "" : "," + fields) + "}";
}

void Trace::sample(double t, const std::string& cars_json, const std::string& calls_json) {
    samples_ += (first_sample_ ? "" : ",");
    first_sample_ = false;
    samples_ += "{\"t\":" + fmt(t) + ",\"cars\":" + cars_json + ",\"calls\":" + calls_json + "}";
}

void Trace::decision(const DecisionTrace& d) {
    if (!wants_decision()) return;
    ++decisions_written_;
    std::string s = "{\"t\":" + fmt(d.t) + ",\"method\":\"" + d.method + "\",\"new_call\":" + json_key(d.new_call);
    s += ",\"calls\":[";
    for (size_t i = 0; i < d.calls.size(); ++i) s += (i ? "," : "") + json_key(d.calls[i]);
    s += "],\"cars\":[";
    for (size_t i = 0; i < d.cars.size(); ++i) s += (i ? "," : "") + std::to_string(d.cars[i]);
    s += "],\"unary\":[";
    for (size_t i = 0; i < d.unary.size(); ++i) {
        s += (i ? "," : "") + std::string("[");
        for (size_t c = 0; c < d.unary[i].size(); ++c) s += (c ? "," : "") + fmt(d.unary[i][c]);
        s += "]";
    }
    s += "],\"n_locked\":" + std::to_string(d.n_locked) + ",\"unary_lk\":[";
    for (size_t i = 0; i < d.unary_lk.size(); ++i) {
        s += (i ? "," : "") + std::string("[");
        for (size_t c = 0; c < d.unary_lk[i].size(); ++c) s += (c ? "," : "") + fmt(d.unary_lk[i][c]);
        s += "]";
    }
    s += "],\"pair\":[";
    for (size_t i = 0; i < d.pair.size(); ++i) {
        s += (i ? "," : "") + std::string("[");
        for (size_t j = 0; j < d.pair[i].size(); ++j) {
            s += (j ? "," : "") + std::string("[");
            for (size_t c = 0; c < d.pair[i][j].size(); ++c) s += (c ? "," : "") + fmt(d.pair[i][j][c]);
            s += "]";
        }
        s += "]";
    }
    s += "],\"p\":[";
    for (size_t i = 0; i < d.p.size(); ++i) s += (i ? "," : "") + fmt(d.p[i]);
    s += "],\"order\":[";
    for (size_t i = 0; i < d.order.size(); ++i) {
        auto [ci, cc, g] = d.order[i];
        s += (i ? "," : "") + std::string("[") + std::to_string(ci) + "," + std::to_string(cc) + "," + fmt(g) + "]";
    }
    s += "],\"before\":[";
    bool first = true;
    for (auto& [k, c] : d.before) { s += (first ? "" : ",") + std::string("[") + std::to_string(k.floor) + "," + std::to_string(k.dir) + "," + std::to_string(c) + "]"; first = false; }
    s += "],\"after\":[";
    first = true;
    for (auto& [k, c] : d.after) { s += (first ? "" : ",") + std::string("[") + std::to_string(k.floor) + "," + std::to_string(k.dir) + "," + std::to_string(c) + "]"; first = false; }
    s += "],\"micros\":" + fmt(d.micros) + ",\"g_greedy\":" + fmt(d.g_greedy) + ",\"g_final\":" + fmt(d.g_final) +
         ",\"search_mode\":" + std::to_string(d.search_mode) + ",\"search_iterations\":" + std::to_string(d.search_iterations) +
         ",\"search_changed\":" + std::to_string(d.search_changed);
    if (d.method.rfind("batch_", 0) == 0)
        s += ",\"batch\":{\"eligible\":" + std::to_string(d.batch_eligible) + ",\"changed\":" +
             std::to_string(d.batch_changed) + ",\"delta_wait\":" + fmt(d.batch_wait) + ",\"delta_journey\":" + fmt(d.batch_journey) + ",\"global_complete\":" + (d.batch_global_complete?"true":"false") + ",\"global_proposed\":" + (d.batch_global_proposed?"true":"false") + ",\"global_states\":" + std::to_string(d.batch_global_states) + ",\"single_delta_journey\":" + fmt(d.batch_single_journey) + "}";
    if (d.reeval_candidates > 0) {

        s += ",\"reeval\":{\"candidates\":" + std::to_string(d.reeval_candidates) + ",\"pick\":" + std::to_string(d.reeval_pick) +
             ",\"removed\":" + std::to_string(d.reeval_removed) + ",\"us\":" + fmt(d.us_reeval) + ",\"scores\":[";
        for (size_t i = 0; i < d.reeval_scores.size(); ++i) s += (i ? "," : "") + fmt(d.reeval_scores[i]);
        s += "],\"se\":[";
        for (size_t i = 0; i < d.reeval_se.size(); ++i) s += (i ? "," : "") + fmt(d.reeval_se[i]);
        s += "],\"se_diff\":[";
        for (size_t i = 0; i < d.reeval_se_diff.size(); ++i) s += (i ? "," : "") + fmt(d.reeval_se_diff[i]);
        s += "],\"g\":[";
        for (size_t i = 0; i < d.reeval_g.size(); ++i) s += (i ? "," : "") + fmt(d.reeval_g[i]);
        s += "]}";
    }
    if (!d.ga_best_by_gen.empty()) {

        s += ",\"ga\":{\"best_by_gen\":[";
        for (size_t i = 0; i < d.ga_best_by_gen.size(); ++i) s += (i ? "," : "") + fmt(d.ga_best_by_gen[i]);
        s += "],\"seed_fit\":" + fmt(d.ga_seed_fit) + ",\"lastbest_fit\":" + fmt(d.ga_lastbest_fit) + ",\"final_fit\":" + fmt(d.ga_final_fit) +
             ",\"evaluations\":" + std::to_string(d.ga_evaluations) + ",\"locked\":[";
        for (size_t i = 0; i < d.ga_locked.size(); ++i) s += (i ? "," : "") + std::to_string(d.ga_locked[i]);
        s += "],\"estimation\":[";
        for (size_t i = 0; i < d.ga_estimation.size(); ++i) s += (i ? "," : "") + std::to_string(d.ga_estimation[i]);
        s += "]}";
    }
    s += ",\"us_matrix\":" + fmt(d.us_matrix) + ",\"us_greedy\":" + fmt(d.us_greedy) + ",\"us_search\":" + fmt(d.us_search) + "}";
    decisions_ += (first_decision_ ? "" : ",") + s;
    first_decision_ = false;
}

void Trace::finish(const std::string& metrics_json) {
    out_ << "{\"meta\":{" << meta_ << "},\"metrics\":" << metrics_json << ",\"events\":[" << events_
         << "],\"samples\":[" << samples_ << "],\"decisions\":[" << decisions_ << "]}\n";
    out_.close();
}

}
