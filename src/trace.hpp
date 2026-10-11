#pragma once
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include "dispatchers.hpp"

namespace egcs2017 {

class Trace {
public:
    Trace(const std::string& path, double sample_dt, int max_decisions);
    ~Trace();
    void meta(const std::string& json_object_body);
    void event(double t, const std::string& type, const std::string& fields);
    void sample(double t, const std::string& cars_json, const std::string& calls_json);
    void decision(const DecisionTrace& d);
    void finish(const std::string& metrics_json);
    double sample_dt() const { return dt_; }
    bool wants_decision() const { return decisions_written_ < max_decisions_; }

private:
    std::ofstream out_;
    double dt_;
    int max_decisions_;
    int decisions_written_ = 0;
    bool first_event_ = true, first_sample_ = true, first_decision_ = true;
    std::string events_, samples_, decisions_;
    std::string meta_;
};

std::string json_key(const HallKey& k);
std::string fmt(double x);

}
