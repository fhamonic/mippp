#ifndef MIPPP_LINKED_CBC_TRAITS_HPP
#define MIPPP_LINKED_CBC_TRAITS_HPP

#include <algorithm>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "coin/CbcModel.hpp"  // deprecated interface, use CbcMain0 and CbcMain1
#include "coin/CbcSolver.hpp"
#include "coin/OsiClpSolverInterface.hpp"

#include "mippp/detail/infinity_helper.hpp"
#include "mippp/solvers/abstract_solver_wrapper.hpp"

namespace fhamonic {
namespace mippp {

struct linked_cbc_model_traits {
    using variable_id_t = int;
    using constraint_id_t = std::size_t;
    using scalar_t = double;

    enum opt_sense : int { min = 1, max = -1 };
    enum var_category : char { continuous = 0, integer = 1, binary = 2 };

    static constexpr scalar_t minus_infinity =
        detail::minus_infinity_or_lowest<scalar_t>();
    static constexpr scalar_t infinity = detail::infinity_or_max<scalar_t>();
};

struct linked_cbc_solver : public abstract_solver_wrapper {
    enum ret_code : int { success = 0, infeasible = 1, timeout = 4 };

    CbcModel model;
    std::vector<std::string> parameters = {"cbc"};
    std::optional<std::size_t> loglevel_index;
    std::optional<std::size_t> timeout_index;

    [[nodiscard]] explicit linked_cbc_solver(const auto & m_model) {
        using traits = typename std::decay_t<decltype(m_model)>::solver_traits;
        typename traits::opt_sense sense = m_model.optimization_sense();
        int num_vars = static_cast<int>(m_model.num_variables());
        double const * obj = m_model.column_coefs();
        double const * col_lb = m_model.column_lower_bounds();
        double const * col_ub = m_model.column_upper_bounds();
        typename traits::var_category const * vtype = m_model.column_types();
        int num_rows = static_cast<int>(m_model.num_constraints());
        int num_elems = static_cast<int>(m_model.num_entries());
        int const * row_begins = m_model.row_begins();
        int const * indices = m_model.var_entries();
        double const * coefs = m_model.coef_entries();
        double const * row_lb = m_model.row_lower_bounds();
        double const * row_ub = m_model.row_upper_bounds();

        double * col_lb_copy = new double[num_vars];
        std::copy(col_lb, col_lb + num_vars, col_lb_copy);
        double * col_ub_copy = new double[num_vars];
        std::copy(col_ub, col_ub + num_vars, col_ub_copy);
        double * obj_copy = new double[num_vars];
        std::copy(obj, obj + num_vars, obj_copy);

        double * row_lb_copy = new double[num_rows];
        std::copy(row_lb, row_lb + num_rows, row_lb_copy);
        double * row_ub_copy = new double[num_rows];
        std::copy(row_ub, row_ub + num_rows, row_ub_copy);

        OsiSolverInterface * solver = new OsiClpSolverInterface;

        int * row_begins_copy = new int[num_rows + 1];
        std::copy(row_begins, row_begins + num_rows, row_begins_copy);
        row_begins_copy[num_rows] = num_elems;  // thats dumb
        CoinPackedMatrix * matrix =
            new CoinPackedMatrix(false, num_vars, num_rows, num_elems, coefs,
                                 indices, row_begins_copy, nullptr);
        delete[] row_begins_copy;
        // ownership of copies and matrix is transfered to solver
        solver->assignProblem(matrix, col_lb_copy, col_ub_copy, obj_copy,
                              row_lb_copy, row_ub_copy);
        solver->setObjSense(sense);
        for(int i = 0; i < num_vars; ++i) {
            if(vtype[i] == traits::var_category::integer ||
               vtype[i] == traits::var_category::binary)
                solver->setInteger(i);
        }
        model.assignSolver(solver);
    }
    ~linked_cbc_solver() {}

    void set_loglevel(int loglevel) noexcept {
        if(!loglevel_index.has_value()) {
            loglevel_index.emplace(parameters.size());
            parameters.emplace_back();
        }
        parameters[loglevel_index.value()] = "-log=" + std::to_string(loglevel);
    }
    void set_timeout(int timeout_s) noexcept {
        if(!timeout_index.has_value()) {
            timeout_index.emplace(parameters.size());
            parameters.emplace_back();
        }
        parameters[timeout_index.value()] = "-sec=" + std::to_string(timeout_s);
    }
    void set_mip_gap(double precision) noexcept {
        parameters[timeout_index.value()] =
            "-mipgap=" + std::to_string(precision);
    }
    void add_param(const std::string & param) {
        parameters.emplace_back(param);
    }

    static int cbc_callback(CbcModel * currentSolver, int whereFrom) {
        return 0;
    }

    int optimize() noexcept {
        std::array<const char *, 64> argv;
        std::ranges::transform(parameters, argv.begin(), &std::string::c_str);
        argv[parameters.size()] = "-solve";

        CbcSolverUsefulData solverData;
        CbcMain0(model, solverData);
        CbcMain1(static_cast<int>(parameters.size() + 1), argv.data(), model,
                 &cbc_callback, solverData);

        return model.status();
    }

    [[nodiscard]] std::vector<double> get_solution() const noexcept {
        std::size_t num_vars = static_cast<std::size_t>(model.getNumCols());
        std::vector<double> solution(num_vars);
        const double * solution_arr = model.getColSolution();
        solution.assign(solution_arr, solution_arr + num_vars);
        return solution;
    }
    [[nodiscard]] double get_objective_value() const noexcept {
        return model.getObjValue();
    }
};

}  // namespace mippp
}  // namespace fhamonic

// assignMatrix (const bool colordered, const int minor, const int major, const
// CoinBigIndex numels, double *&elem, int *&ind, CoinBigIndex *&start, int
// *&len, const int maxmajor=-1, const CoinBigIndex maxsize=-1) assignProblem
// (CoinPackedMatrix *&matrix, double *&collb, double *&colub, double *&obj,
// char *&rowsen, double *&rowrhs, double *&rowrng)=0

#endif  // MIPPP_LINKED_CBC_TRAITS_HPP