#pragma once
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include "cost.hpp"

namespace egcs2017 {

struct DecisionTrace {
    int batch_eligible = 0, batch_changed = 0;
    bool batch_global_complete=false, batch_global_proposed=false;
    long batch_global_states=0;
    double batch_single_journey=0;
    double batch_wait = 0, batch_journey = 0;
    double t = 0.0;
    std::string method;
    HallKey new_call{0, 0};
    std::vector<HallKey> calls;
    std::vector<int> cars;
    std::vector<std::vector<double>> unary;
    std::vector<std::vector<double>> unary_lk;
    int n_locked = 0;
    std::vector<std::vector<std::vector<double>>> pair;
    std::vector<double> p;
    std::vector<std::tuple<int, int, double>> order;
    std::map<HallKey, int> before, after;
    double micros = 0.0;

    double g_greedy = 0.0, g_final = 0.0;
    int search_iterations = 0;
    int search_changed = 0;
    int search_mode = 0;
    bool search_cap_hit = false;
    double us_matrix = 0, us_greedy = 0, us_search = 0;
    int reeval_candidates = 0, reeval_pick = -1;
    double us_reeval = 0;
    std::vector<double> reeval_scores;
    std::vector<double> reeval_se;
    std::vector<double> reeval_se_diff;

    std::vector<double> reeval_g;
    int reeval_removed = 0;

    std::vector<double> ga_best_by_gen;
    double ga_seed_fit = -1.0, ga_lastbest_fit = -1.0, ga_final_fit = -1.0;
    int ga_evaluations = 0;
    std::vector<int> ga_locked;
    std::vector<int> ga_estimation;
};

struct Matrices {
    std::vector<CallView> N;
    std::vector<std::vector<HallKey>> locked;
    std::vector<std::vector<double>> u, u_raw, adj, u_lk;
    std::vector<std::vector<std::vector<double>>> w;
};
Matrices build_matrices(const Building& b, const std::vector<CarView>& cars, const std::vector<CallView>& calls,
                        const HallKey& new_call, const CostOptions& opt);
double onboard_stop_externality(const Building&, const CarView&, const HallKey&);

std::vector<double> path_cost_samples(const Building& b, const std::vector<CarView>& cars, const Matrices& M,
                                      const std::vector<int>& car_of, const std::vector<std::vector<int>>& dest_free,
                                      const std::vector<std::vector<std::vector<int>>>& dest_locked, bool with_locked);

double reeval_adjustment(const Matrices& M, const std::vector<int>& car_of, int C, const CostOptions& opt);

std::vector<std::vector<int>> reeval_candidates(const std::vector<int>& search_result, const std::vector<int>& greedy_result,
                                                const std::vector<std::vector<double>>& u,
                                                const std::vector<std::vector<std::vector<double>>>& w, int C, const CostOptions& opt);
void common_destination_samples(const Building& b, const Matrices& M, double first_registered, const HallKey& nc, int K,
                                std::vector<std::vector<int>>& dest_free, std::vector<std::vector<std::vector<int>>>& dest_locked);

class Dispatcher {
public:
    virtual ~Dispatcher() = default;
    virtual std::string name() const = 0;

    virtual std::map<HallKey, int> decide(const Building& b, const std::vector<CarView>& cars,
                                          const std::vector<CallView>& calls, const HallKey& new_call,
                                          DecisionTrace* trace) = 0;

    virtual void observe_boarding(double, int, int, int) {}

    virtual void observe_learning(double, double, bool) {}
};

inline HallKey no_new_call() { return HallKey{-1, 0}; }

class CollectiveDispatcher : public Dispatcher {
public:
    std::string name() const override { return "collective"; }
    std::map<HallKey, int> decide(const Building&, const std::vector<CarView>&, const std::vector<CallView>&,
                                  const HallKey&, DecisionTrace*) override;
};

class EtaDispatcher : public Dispatcher {
public:
    std::string name() const override { return "eta"; }
    std::map<HallKey, int> decide(const Building&, const std::vector<CarView>&, const std::vector<CallView>&,
                                  const HallKey&, DecisionTrace*) override;
};

class SubmodularDispatcher : public Dispatcher {
public:
    explicit SubmodularDispatcher(CostOptions opt, std::string label = "submodular") : opt_(opt), label_(std::move(label)) {}
    std::string name() const override { return label_; }
    std::map<HallKey, int> decide(const Building&, const std::vector<CarView>&, const std::vector<CallView>&,
                                  const HallKey&, DecisionTrace*) override;
    const CostOptions& options() const { return opt_; }

private:
    CostOptions opt_;
    std::string label_;
};

std::unique_ptr<Dispatcher> make_dispatcher(const std::string& method);

double g_of_public(const std::vector<int>& car_of, const std::vector<std::vector<double>>& u,
                   const std::vector<std::vector<std::vector<double>>>& w, int C, const CostOptions& opt);
int local_search_public(std::vector<int>& car_of, const std::vector<std::vector<double>>& u,
                        const std::vector<std::vector<std::vector<double>>>& w, int C, const CostOptions& opt);
std::vector<int> greedy_public(const std::vector<std::vector<double>>& u, const std::vector<std::vector<std::vector<double>>>& w, int C,
                               const CostOptions& opt);
long exhaustive_public(std::vector<int>& car_of, const std::vector<std::vector<double>>& u,
                       const std::vector<std::vector<std::vector<double>>>& w, int C, const CostOptions& opt);

}
