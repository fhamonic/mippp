#ifndef MIPPP_ABSTRACT_CLI_SOLVER_WRAPPER_HPP
#define MIPPP_ABSTRACT_CLI_SOLVER_WRAPPER_HPP

#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "mippp/solvers/abstract_solver_wrapper.hpp"

namespace fhamonic {
namespace mippp {

struct cli_solver_model_traits {
    using var_id_t = int;
    using constraint_id_t = std::size_t;
    using scalar_t = double;

    enum opt_sense : int { min = -1, max = 1 };
    enum var_category : char { continuous = 0, integer = 1, binary = 2 };

    static constexpr scalar_t minus_infinity =
        std::numeric_limits<scalar_t>::lowest();
    static constexpr scalar_t infinity = std::numeric_limits<scalar_t>::max();
};

class abstract_cli_solver_wrapper : public abstract_solver_wrapper {
public:
    enum ret_code : int { success = 0, infeasible = 1, timeout = 2 };
protected:
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

    [[nodiscard]] abstract_cli_solver_wrapper(const auto & model) {
        using var = typename std::decay_t<decltype(model)>::var;
        nb_variables = model.nb_variables();
        for(auto var_id : model.variables()) {
            var_name_to_id[model.name(var(var_id))] =
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

public:
    [[nodiscard]] virtual std::string name() const = 0;
    [[nodiscard]] std::filesystem::path solution_path() const noexcept {
        return sol_path;
    }
    [[nodiscard]] std::filesystem::path logs_path() const noexcept {
        return log_path;
    }
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_ABSTRACT_CLI_SOLVER_WRAPPER_HPP