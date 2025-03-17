#ifndef MIPPP_CLI_SCIP_TRAITS_HPP
#define MIPPP_CLI_SCIP_TRAITS_HPP

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "mippp/solvers/abstract_cli_solver_wrapper.hpp"

namespace fhamonic {
namespace mippp {

struct cli_scip_solver : public abstract_cli_solver_wrapper {
    static inline std::filesystem::path exec_path = "scip";
    static auto call(const std::string & params) {
#ifdef WIN32
        auto cmd =
            (std::ostringstream{} << "call " << exec_path << " " << params)
                .str();
#else
        auto cmd = (std::ostringstream{} << exec_path << " " << params).str();
#endif
        return std::system(cmd.c_str());
    }
    static bool is_available() {
#ifdef WIN32
        return call("-c quit 1> NUL 2> NUL") == 0;
#else
        return call("-c quit 1> /dev/null 2> /dev/null") == 0;
#endif
    }

    [[nodiscard]] cli_scip_solver(const auto & model)
        : abstract_cli_solver_wrapper(model) {}

    ~cli_scip_solver() {}

    void set_loglevel(int loglevel) noexcept {
        // if(!loglevel_index.has_value()) {
        //     loglevel_index.emplace(parameters.size());
        //     parameters.emplace_back();
        // }
        // parameters[loglevel_index.value()] =
        //     "LogToConsole=" + std::to_string(loglevel);
    }
    void set_timeout(int timeout_s) noexcept {
        if(!timeout_index.has_value()) {
            timeout_index.emplace(parameters.size());
            parameters.emplace_back();
        }
        parameters[timeout_index.value()] =
            "-c \"set limits time " + std::to_string(timeout_s) + '\"';
    }
    void set_mip_optimality_gap(double precision) noexcept {
        if(!mip_optimality_gap_index.has_value()) {
            mip_optimality_gap_index.emplace(parameters.size());
            parameters.emplace_back();
        }
        parameters[mip_optimality_gap_index.value()] =
            "-c \"set limits gap " + double_to_string(precision) + '\"';
    }
    void set_feasability_tolerance(double precision) noexcept {
        if(!feasability_tolerance_index.has_value()) {
            feasability_tolerance_index.emplace(parameters.size());
            parameters.emplace_back();
        }
        parameters[feasability_tolerance_index.value()] =
            "-c \"set numerics feastol " + double_to_string(precision) + '\"';
    }
    void add_param(const std::string & param) {
        parameters.emplace_back(param);
    }

private:
    bool skip(std::ifstream & myfile, int count = 1) {
        std::string s;
        for(int i = 0; i < count; ++i)
            if(!(myfile >> s)) return false;
        return true;
    }

public:
    int optimize() noexcept {
        std::ostringstream solver_cmd;
        solver_cmd << "-c \"read " << lp_path << '\"';
        for(auto && param : parameters) {
            solver_cmd << ' ' << param;
        }
        solver_cmd << " -c optimize -c \"write solution " << sol_path
                   << "\" -c quit > " << log_path;
        auto ret = call(solver_cmd.str().c_str());
        if(ret != 0) return ret;

        solution.clear();
        solution.resize(num_variables, 0);

        std::ifstream sol_file;
        auto flags = sol_file.exceptions();
        sol_file.exceptions(sol_file.exceptions() | std::ios::failbit);
        sol_file.open(sol_path);
        sol_file.exceptions(flags);

        skip(sol_file, 7);
        sol_file >> objective_value;
        std::string var;
        double value;
        while(sol_file >> var >> value) {
            solution[var_name_to_id.at(var)] = value;
            skip(sol_file, 1);
        }

        return ret;
    }

    [[nodiscard]] std::vector<double> get_solution() const noexcept {
        return solution;
    }
    [[nodiscard]] double get_objective_value() const noexcept {
        return objective_value;
    }
    [[nodiscard]] std::string name() const noexcept { return "SCIP"; }
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_CLI_SCIP_TRAITS_HPP