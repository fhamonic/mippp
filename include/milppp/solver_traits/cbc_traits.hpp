#ifndef MILPP_CBC_TRAITS_HPP
#define MILPP_CBC_TRAITS_HPP

#include "coin/CbcModel.h"
#include "coin/CbcSolver.h"

struct CbcTraits {
    enum OptSense : int { MINIMIZE=1, MAXIMIZE=-1 };
    enum ColType : char { CONTINUOUS = 0, INTEGER = 1, BINARY = 2 };

    static CbcModel build(OptSense opt_sense,
                              int nb_vars, 
                              double *obj,
                              double *col_lb,
                              double *col_ub,
                              ColType *vtype,
                              int nb_rows,
                              int nb_elems,
                              int *row_begins,
                              int *indices,
                              double *coefs,
                              double *row_lb,
                              double *row_ub) {
        CbcModel model;
        OsiSolverInterface * solver = new OsiSolverInterface;
        solver->loadProblem(nb_vars, nb_rows, row_begins, indices, coefs, col_lb, col_ub, obj, row_lb, row_ub);
        solver->setSense(opt_sense);
        for(int i=0; i<nb_vars; ++i) {
            if(vtype[i] == INTEGER || vtype[i] == BINARY)
                solver->setInteger(i);
        }
        model.assignSolver(solver);
        return model;
    }
};

// assignMatrix (const bool colordered, const int minor, const int major, const CoinBigIndex numels, double *&elem, int *&ind, CoinBigIndex *&start, int *&len, const int maxmajor=-1, const CoinBigIndex maxsize=-1)
// assignProblem (CoinPackedMatrix *&matrix, double *&collb, double *&colub, double *&obj, char *&rowsen, double *&rowrhs, double *&rowrng)=0

#endif //MILPP_CBC_TRAITS_HPP