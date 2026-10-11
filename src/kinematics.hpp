#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace egcs2017 {

struct Kinematics {
    double v_max = 1.6;
    double a_max = 0.8;
    double settle = 0.5;
    double floor_height = 3.5;

    double stop_distance(double v) const { return v * v / (2.0 * a_max); }

    struct Profile {
        double d = 0, v0 = 0, vp = 0, t_acc = 0, t_cruise = 0, t_dec = 0, settle = 0;
        double duration() const { return t_acc + t_cruise + t_dec + settle; }
        double moving_time() const { return t_acc + t_cruise + t_dec; }

        double position(double t) const {
            if (t <= 0) return 0.0;
            if (t >= moving_time()) return d;
            double a = t_acc > 0 ? (vp - v0) / t_acc : 0.0;
            if (t <= t_acc) return v0 * t + 0.5 * a * t * t;
            double x = v0 * t_acc + 0.5 * a * t_acc * t_acc;
            t -= t_acc;
            if (t <= t_cruise) return x + vp * t;
            x += vp * t_cruise;
            t -= t_cruise;
            double dec = t_dec > 0 ? vp / t_dec : 0.0;
            return x + vp * t - 0.5 * dec * t * t;
        }
        double velocity(double t) const {
            if (t <= 0) return v0;
            if (t >= moving_time()) return 0.0;
            double a = t_acc > 0 ? (vp - v0) / t_acc : 0.0;
            if (t <= t_acc) return v0 + a * t;
            t -= t_acc;
            if (t <= t_cruise) return vp;
            t -= t_cruise;
            double dec = t_dec > 0 ? vp / t_dec : 0.0;
            return std::max(0.0, vp - dec * t);
        }
    };

    Profile profile(double d, double v0 = 0.0) const {
        if (d < 0) throw std::invalid_argument("negative distance");
        Profile p;
        p.d = d; p.v0 = v0; p.settle = settle;
        if (d <= 1e-12 && v0 <= 1e-12) { p.settle = 0.0; return p; }
        double need = stop_distance(v0);
        if (d < need) {
            if (need - d > 1e-6) throw std::invalid_argument("cannot stop within distance");
            d = need;
        }

        double vp = std::sqrt(std::max(0.0, (2.0 * a_max * d + v0 * v0) / 2.0));
        if (vp > v_max) {
            vp = v_max;
            double dist_acc = (vp * vp - v0 * v0) / (2.0 * a_max);
            double dist_dec = vp * vp / (2.0 * a_max);
            p.t_cruise = (d - dist_acc - dist_dec) / vp;
        }
        p.vp = vp;
        p.t_acc = (vp - v0) / a_max;
        p.t_dec = vp / a_max;
        return p;
    }

    double travel(int from, int to) const {
        if (from == to) return 0.0;
        return profile(std::abs(from - to) * floor_height).duration();
    }

    double time_from_state(double dist, double v0) const {
        if (dist <= 1e-12 && v0 <= 1e-12) return 0.0;
        return profile(dist, v0).duration();
    }
};

}
