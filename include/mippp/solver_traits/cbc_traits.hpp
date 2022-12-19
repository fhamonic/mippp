#ifndef MIPPP_CBC_TRAITS_HPP
#define MIPPP_CBC_TRAITS_HPP

#include <optional>
#include <string>
#include <vector>

#include <coin/CbcModel.hpp>  // deprecated interface, use CbcMain0 and CbcMain1
#include <coin/CbcSolver.hpp>
#include <coin/OsiClpSolverInterface.hpp>

namespace fhamonic {
namespace mippp {

struct cbc_solver_wrapper {
    CbcModel model;
    std::vector<std::string> parameters = {"cbc"};
    std::optional<std::size_t> loglevel_index;
    std::optional<std::size_t> timeout_index;

    [[nodiscard]] cbc_solver_wrapper() {}
    [[nodiscard]] explicit cbc_solver_wrapper(OsiSolverInterface * solver) {
        // ownership of solver is transfered to model
        model.assignSolver(solver);
    }
    ~cbc_solver_wrapper() {}

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
    void add_param(const std::string & param) {
        parameters.emplace_back(param);
    }

    int optimize() noexcept {
        std::array<const char *, 64> argv;
        std::ranges::transform(parameters, argv.begin(), &std::string::c_str);
        argv[parameters.size()] = "-solve";

        CbcSolverUsefulData solverData;
        CbcMain0(model, solverData);
        CbcMain1(static_cast<int>(parameters.size() + 1), argv.data(), model,
                 nullptr, solverData);
        // model.branchAndBound();
        return model.status();
    }

    [[nodiscard]] std::vector<double> get_solution() const noexcept {
        std::size_t nb_vars = static_cast<std::size_t>(model.getNumCols());
        std::vector<double> solution(nb_vars);
        const double * solution_arr = model.getColSolution();
        solution.assign(solution_arr, solution_arr + nb_vars);
        return solution;
    }
};

struct cbc_traits {
    enum opt_sense : int { min = 1, max = -1 };
    enum var_category : char { continuous = 0, integer = 1, binary = 2 };
    using model_wrapper = cbc_solver_wrapper;

    static cbc_solver_wrapper build(opt_sense opt_sense, int nb_vars,
                                    double const * obj, double const * col_lb,
                                    double const * col_ub,
                                    var_category const * vtype, int nb_rows,
                                    int nb_elems, int const * row_begins,
                                    int const * indices, double const * coefs,
                                    double const * row_lb,
                                    double const * row_ub) {
        double * col_lb_copy = new double[nb_vars];
        std::copy(col_lb, col_lb + nb_vars, col_lb_copy);
        double * col_ub_copy = new double[nb_vars];
        std::copy(col_ub, col_ub + nb_vars, col_ub_copy);
        double * obj_copy = new double[nb_vars];
        std::copy(obj, obj + nb_vars, obj_copy);

        double * row_lb_copy = new double[nb_rows];
        std::copy(row_lb, row_lb + nb_rows, row_lb_copy);
        double * row_ub_copy = new double[nb_rows];
        std::copy(row_ub, row_ub + nb_rows, row_ub_copy);

        OsiSolverInterface * solver = new OsiClpSolverInterface;

        int * row_begins_copy = new int[nb_rows + 1];
        std::copy(row_begins, row_begins + nb_rows, row_begins_copy);
        row_begins_copy[nb_rows] = nb_elems;  // thats dumb
        CoinPackedMatrix * matrix =
            new CoinPackedMatrix(false, nb_vars, nb_rows, nb_elems, coefs,
                                 indices, row_begins_copy, nullptr);
        delete[] row_begins_copy;
        // ownership of copies and matrix is transfered to solver
        solver->assignProblem(matrix, col_lb_copy, col_ub_copy, obj_copy,
                              row_lb_copy, row_ub_copy);
        solver->setObjSense(opt_sense);
        for(int i = 0; i < nb_vars; ++i) {
            if(vtype[i] == integer || vtype[i] == binary) solver->setInteger(i);
        }

        return cbc_solver_wrapper(solver);
    }
};

}  // namespace mippp
}  // namespace fhamonic

// assignMatrix (const bool colordered, const int minor, const int major, const
// CoinBigIndex numels, double *&elem, int *&ind, CoinBigIndex *&start, int
// *&len, const int maxmajor=-1, const CoinBigIndex maxsize=-1) assignProblem
// (CoinPackedMatrix *&matrix, double *&collb, double *&colub, double *&obj,
// char *&rowsen, double *&rowrhs, double *&rowrng)=0

#endif  // MIPPP_CBC_TRAITS_HPP