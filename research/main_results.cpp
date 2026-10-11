#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <numeric>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

using Key = std::tuple<int, int, int, int>;
using Cell = std::tuple<int, int, int>;
using Values = std::array<double, 5>;
using Grid = std::map<std::string, std::map<Key, Values>>;
const std::array<std::string, 5> metrics{"awt", "p95_wait", "att", "ajt", "frac_over_60"};

std::vector<std::string> split(const std::string& text) {
    std::vector<std::string> fields;
    std::istringstream stream(text);
    std::string field;
    while (std::getline(stream, field, ',')) fields.push_back(field);
    return fields;
}

Grid read(const std::string& path) {
    std::ifstream input(path);
    if (!input) throw std::runtime_error("missing CSV: " + path);
    std::string line;
    std::getline(input, line);
    const auto header = split(line);
    std::map<std::string, size_t> columns;
    for (size_t i = 0; i < header.size(); ++i) columns.emplace(header[i], i);
    Grid grid;
    while (std::getline(input, line)) {
        if (line.empty()) continue;
        const auto fields = split(line);
        if (fields.size() != header.size()) throw std::runtime_error("invalid CSV row");
        auto field = [&](const std::string& name) -> const std::string& { return fields.at(columns.at(name)); };
        if (std::stoi(field("unserved")) || std::stoi(field("unfinished"))) throw std::runtime_error("undrained run");
        Key key{std::stoi(field("floors")), std::stoi(field("cars")), std::stoi(field("rate")), std::stoi(field("seed"))};
        Values values{};
        for (size_t i = 0; i < metrics.size(); ++i) values[i] = std::stod(field(metrics[i]));
        if (!grid[field("method")].emplace(key, values).second) throw std::runtime_error("duplicate run");
    }
    for (const auto& [method, rows] : grid) {
        (void)method;
        if (rows.size() != 2250) throw std::runtime_error("expected 75 cells and 30 seeds");
        for (int f : {8, 10, 12}) for (int c : {2, 3, 4, 5, 6}) for (int r : {10, 15, 20, 25, 30})
            for (int seed = 50000; seed < 50030; ++seed)
                if (!rows.contains(Key{f, c, r, seed})) throw std::runtime_error("incomplete confirmation grid");
    }
    return grid;
}

Values level(const std::map<Key, Values>& rows) {
    Values result{};
    for (const auto& [key, values] : rows) {
        (void)key;
        for (size_t i = 0; i < result.size(); ++i) result[i] += values[i] / rows.size();
    }
    return result;
}

void values_json(std::ostream& out, const Values& values) {
    out << '{';
    for (size_t i = 0; i < values.size(); ++i) out << (i ? "," : "") << '"' << metrics[i] << "\":" << values[i];
    out << '}';
}

void contrast(std::ostream& out, const std::map<Key, Values>& left, const std::map<Key, Values>& right) {
    out << '{';
    for (size_t metric = 0; metric < metrics.size(); ++metric) {
        std::array<double, 30> differences{};
        for (const auto& [key, values] : left)
            differences[std::get<3>(key) - 50000] += (values[metric] - right.at(key)[metric]) / 75;
        const double mean = std::accumulate(differences.begin(), differences.end(), 0.0) / 30;
        double sum = 0;
        for (double value : differences) sum += (value - mean) * (value - mean);
        const double width = 2.045229642132703 * std::sqrt(sum / 29 / 30);
        out << (metric ? "," : "") << '"' << metrics[metric] << "\":{\"mean\":" << mean
            << ",\"ci95\":[" << mean - width << ',' << mean + width << "],\"df\":29}";
    }
    out << '}';
}

void contrasts(std::ostream& out, const Grid& grid) {
    const std::array<std::array<std::string, 3>, 4> names{{
        {"correction_greedy", "paper2017_lk", "paper2017"},
        {"correction_search", "paper2017_ls_lk", "paper2017_ls"},
        {"search_uncorrected", "paper2017_ls", "paper2017"},
        {"search_corrected", "paper2017_ls_lk", "paper2017_lk"}}};
    out << '{';
    for (size_t i = 0; i < names.size(); ++i) {
        out << (i ? "," : "") << '"' << names[i][0] << "\":";
        contrast(out, grid.at(names[i][1]), grid.at(names[i][2]));
    }
    out << '}';
}

int main(int argc, char** argv) {
    try {
        if (argc != 6) throw std::runtime_error("usage: main_results THETA5 GA PERIODIC THETA0 THETA2");
        auto grid = read(argv[1]);
        auto ga = read(argv[2]);
        auto periodic = read(argv[3]);
        std::cout << std::setprecision(17) << "{\"analysis_unit\":\"seed mean over 75 cells\",\"seeds\":[50000,50029],\"theta\":5,\"levels\":{";
        bool first = true;
        for (const auto& [method, rows] : grid) {
            std::cout << (first ? "" : ",") << '"' << method << "\":";
            first = false;
            values_json(std::cout, level(rows));
        }
        std::cout << ",\"ga2022_all\":";
        values_json(std::cout, level(ga.at("ga2022_all")));
        std::cout << ",\"corrected_periodic\":";
        values_json(std::cout, level(periodic.at("paper2017_ls_lk")));
        std::cout << "},\"components\":";
        contrasts(std::cout, grid);
        std::cout << ",\"corrected_minus_ga\":";
        contrast(std::cout, grid.at("paper2017_ls_lk"), ga.at("ga2022_all"));
        std::cout << ",\"corrected_minus_insertion\":";
        contrast(std::cout, grid.at("paper2017_ls_lk"), grid.at("sequential_greedy"));
        std::cout << ",\"periodic_minus_event\":";
        contrast(std::cout, periodic.at("paper2017_ls_lk"), grid.at("paper2017_ls_lk"));
        std::cout << ",\"threshold_components\":{\"0\":";
        contrasts(std::cout, read(argv[4]));
        std::cout << ",\"2\":";
        contrasts(std::cout, read(argv[5]));
        std::cout << "},\"cell_awt\":[";
        first = true;
        for (int f : {8, 10, 12}) for (int c : {2, 3, 4, 5, 6}) for (int r : {10, 15, 20, 25, 30}) {
            std::cout << (first ? "" : ",") << "{\"floors\":" << f << ",\"cars\":" << c << ",\"rate\":" << r;
            first = false;
            for (const auto& [method, rows] : grid) {
                double sum = 0;
                for (int seed = 50000; seed < 50030; ++seed) sum += rows.at(Key{f, c, r, seed})[0] / 30;
                std::cout << ",\"" << method << "\":" << sum;
            }
            std::cout << '}';
        }
        std::cout << "]}\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 2;
    }
}
