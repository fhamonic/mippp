#ifndef MIPPP_CBC_TRAITS_HPP
#define MIPPP_CBC_TRAITS_HPP

#include <coin/CbcModel.hpp>  // deprecated interface, use CbcMain0 and CbcMain1
#include <coin/CbcSolver.hpp>
#include <coin/OsiClpSolverInterface.hpp>

namespace fhamonic {
namespace mippp {

struct CbcTraits {
    enum OptSense : int { MINIMIZE = 1, MAXIMIZE = -1 };
    enum ColType : char { CONTINUOUS = 0, INTEGER = 1, BINARY = 2 };
    using ModelType = CbcModel;

    static CbcModel build(OptSense opt_sense, int nb_vars, double const * obj,
                          double const * col_lb, double const * col_ub,
                          ColType const * vtype, int nb_rows, int nb_elems,
                          int const * row_begins, int const * indices,
                          double const * coefs, double const * row_lb,
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
            if(vtype[i] == INTEGER || vtype[i] == BINARY) solver->setInteger(i);
        }
        CbcModel model;
        // ownership of solver is transfered to model
        model.assignSolver(solver);
        return model;
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