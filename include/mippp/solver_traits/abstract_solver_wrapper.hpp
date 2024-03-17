#ifndef MIPPP_ABSTRACT_SOLVER_WRAPPER_HPP
#define MIPPP_ABSTRACT_SOLVER_WRAPPER_HPP

#include <string>
#include <vector>

namespace fhamonic {
namespace mippp {

struct abstract_solver_wrapper {
    [[nodiscard]] abstract_solver_wrapper(){};
    [[nodiscard]] abstract_solver_wrapper(const auto & model){};

    [[nodiscard]] abstract_solver_wrapper(const abstract_solver_wrapper &) =
        default;
    [[nodiscard]] abstract_solver_wrapper(abstract_solver_wrapper &&) = default;

    virtual ~abstract_solver_wrapper() {}

    virtual void set_loglevel(int loglevel) = 0;
    virtual void set_timeout(int timeout_s) = 0;
    virtual void set_mip_gap(double precision) = 0;
    virtual void add_param(const std::string & param) = 0;

    virtual int optimize() = 0;
    [[nodiscard]] virtual std::vector<double> get_solution() const = 0;
    [[nodiscard]] virtual double get_objective_value() const = 0;
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_ABSTRACT_SOLVER_WRAPPER_HPP