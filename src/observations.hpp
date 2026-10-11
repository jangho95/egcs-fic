#pragma once
#include <iomanip>
#include <sstream>
#include <vector>
#include "model.hpp"

namespace egcs2017 {
struct ObservedCall {
    CallView view;
    bool prior_service_possible = false;
};

inline std::string observation_json(const Building& b, double t,
                                    const std::vector<CarView>& cars,
                                    const std::vector<ObservedCall>& calls) {
    std::ostringstream o; o << std::setprecision(17);
    o << "{\"t\":" << t << ",\"floors\":" << b.floors << ",\"capacity\":" << b.capacity
      << ",\"floor_height\":" << b.kin.floor_height << ",\"v_max\":" << b.kin.v_max
      << ",\"a_max\":" << b.kin.a_max << ",\"settle\":" << b.kin.settle
      << ",\"door_open\":" << b.doors.open << ",\"door_close\":" << b.doors.close
      << ",\"transfer\":" << b.doors.transfer << ",\"cars\":[";
    for (size_t i=0;i<cars.size();++i) {
        const auto& v=cars[i];
        o << (i?",":"") << "{\"id\":" << v.id << ",\"x\":" << v.x << ",\"v\":" << v.v
          << ",\"dir\":" << v.dir << ",\"moving\":" << (v.moving?"true":"false")
          << ",\"arriving\":" << (v.arriving?"true":"false") << ",\"committed\":" << v.committed
          << ",\"release\":" << v.release << ",\"doors_open\":" << (v.doors_open?"true":"false")
          << ",\"load\":" << v.load << ",\"buttons\":[";
        bool first=true;
        for (const auto& entry:v.car_calls) {o << (first?"":",") << entry.first;first=false;}
        o << "]}";
    }
    o << "],\"calls\":[";
    for (size_t i=0;i<calls.size();++i) {
        const auto& c=calls[i];
        o << (i?",":"") << "{\"floor\":" << c.view.key.floor << ",\"dir\":" << c.view.key.dir
          << ",\"registered\":" << c.view.registered << ",\"assigned\":" << c.view.assigned
          << ",\"theta_locked\":" << (c.view.close_to_service?"true":"false")
          << ",\"prior_service_possible\":" << (c.prior_service_possible?"true":"false") << "}";
    }
    o << "]}";return o.str();
}
}
