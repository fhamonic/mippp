#ifndef MIPPP_CLI_SCIP_TRAITS_HPP
#define MIPPP_CLI_SCIP_TRAITS_HPP

#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace fhamonic {
namespace mippp {

struct cli_scip_solver_wrapper {
    std::size_t nb_variables;
    std::unordered_map<std::string, std::size_t> var_name_to_id;
    double objective_value;
    std::vector<double> solution;

    std::filesystem::path lp_path;
    std::filesystem::path sol_path;
    std::filesystem::path log_path;

    std::vector<std::string> parameters;
    std::optional<std::size_t> loglevel_index;
    std::optional<std::size_t> timeout_index;

    [[nodiscard]] cli_scip_solver_wrapper(const auto & model) {
        using var = typename std::decay_t<decltype(model)>::var;
        nb_variables = model.nb_variables();
        for(auto var_id : model.variables()) {
            auto optional_name = model.name(var(var_id));
            if(!optional_name.has_value()) continue;
            var_name_to_id[optional_name.value()] =
                static_cast<std::size_t>(var_id);
        }
        auto tmp_dir =
            std::filesystem::temp_directory_path() / "mippp" /
            (std::ostringstream{} << std::this_thread::get_id()).str();
        std::filesystem::create_directories(tmp_dir);
        lp_path = tmp_dir / "model.lp";
        sol_path = tmp_dir / "solution.sol";
        log_path = tmp_dir / "output.log";

        std::ofstream lp_file(lp_path);
        lp_file << model << std::endl;
    }

    ~cli_scip_solver_wrapper() {}

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
            (std::ostringstream{} << "-c \"set limits time " << timeout_s << '\"').str();
    }
    void set_mip_gap(double precision) noexcept {
        parameters[timeout_index.value()] =
            (std::ostringstream{} << "-c \"set limits gap " << precision << '\"').str();
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
        solver_cmd << "scip -c \"read " << lp_path << '\"';
        for(auto && param : parameters) {
            solver_cmd << ' ' << param;
        }
        solver_cmd << " -c optimize -c \"write solution " << sol_path
                   << "\" -c quit > " << log_path;
        auto cmd = solver_cmd.str();
        std::cout << cmd << std::endl;
        auto ret = std::system(cmd.c_str());
        solution.clear();
        solution.resize(nb_variables, 0);

        std::ifstream sol_file(sol_path);
        if(sol_file.is_open()) {
            skip(sol_file, 7);
            sol_file >> objective_value;
            std::string var;
            double value;
            while(sol_file >> var >> value) {
                solution[var_name_to_id.at(var)] = value;
                skip(sol_file, 1);
            }
        } else
            std::cout << "Unable to open solution file at " +
                             sol_path.generic_string();

        return ret;
    }

    [[nodiscard]] std::vector<double> get_solution() const noexcept {
        return solution;
    }
    [[nodiscard]] double get_objective_value() const noexcept {
        return objective_value;
    }
};

struct cli_scip_traits {
    enum opt_sense : int { min = -1, max = 1 };
    enum var_category : char { continuous = 0, integer = 1, binary = 2 };
    enum ret_code : int { success = 0, infeasible = 1, timeout = 2 };

    using solver_wrapper = cli_scip_solver_wrapper;

    static cli_scip_solver_wrapper build(const auto & model) {
        cli_scip_solver_wrapper scip(model);
        return scip;
    }
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_CLI_SCIP_TRAITS_HPP