#ifndef MIPPP_CPLEX_TRAITS_HPP
#define MIPPP_CPLEX_TRAITS_HPP

#include <scip/scipdefplugins.h>
#include "scip/cons_linear.h"
#include "scip/retcode.h"
#include "scip/scip.h"

namespace fhamonic {
namespace mippp {

struct scip_solver_wrapper {
    SCIP * model;
    std::vector<SCIP_VAR *> vars;
    std::vector<SCIP_CONS *> constrs;

    [[nodiscard]] scip_solver_wrapper() : model(nullptr) {
        SCIPcreate(&model);  // Creating the SCIP environment
        /* include default plugins */
        SCIPincludeDefaultPlugins(model);
        // Creating the SCIP Problem.
        SCIPcreateProbBasic(model, "scip_solver_wrapper");
    }
    ~scip_solver_wrapper() {
        for(auto & var : vars) {
            SCIPreleaseVar(model, &var);
        }
        for(auto & cons : constrs) {
            SCIPreleaseCons(model, &cons);
        }

        SCIPfree(&model);
    }

    void set_loglevel(int loglevel) noexcept {
        //
    }
    void set_timeout(int timeout_s) noexcept {
        SCIPsetRealParam(model, "limits/time", timeout_s);
    }
    void set_mip_gap(double precision) noexcept {
        SCIPsetRealParam(model, "limits/gap", precision);
    }

    int optimize() noexcept { return SCIPsolve(model); }

    [[nodiscard]] std::vector<double> get_solution() const noexcept {
        std::vector<double> solution(vars.size());
        SCIP_SOL * sol;
        sol = SCIPgetBestSol(model);

        for(std::size_t i = 0; i < vars.size(); ++i) {
            solution[i] = SCIPgetSolVal(model, sol, vars[i]);
        }

        return solution;
    }
    [[nodiscard]] double get_objective_value() const noexcept {
        return SCIPgetPrimalbound(model);
    }
};

struct scip_traits {
    enum opt_sense : std::underlying_type<SCIP_OBJSENSE>::type {
        min = SCIP_OBJSENSE_MINIMIZE,
        max = SCIP_OBJSENSE_MAXIMIZE
    };
    enum var_category : std::underlying_type<SCIP_VARTYPE>::type {
        continuous = SCIP_VARTYPE_CONTINUOUS,
        integer = SCIP_VARTYPE_INTEGER,
        binary = SCIP_VARTYPE_BINARY
    };
    enum ret_code : int { success = SCIP_OKAY, infeasible = 1, timeout = 2 };
    using model_wrapper = scip_solver_wrapper;

    [[nodiscard]] static scip_solver_wrapper build(
        opt_sense opt_sense, int nb_vars, double * obj, double * col_lb,
        double * col_ub, var_category * vtype, int nb_rows, int nb_elems,
        int * row_begins, int * indices, double * coefs, double * row_lb,
        double * row_ub) {
        scip_solver_wrapper scip;
        SCIPsetObjsense(scip.model, static_cast<SCIP_OBJSENSE>(opt_sense));

        scip.vars =
            std::vector<SCIP_VAR *>(static_cast<std::size_t>(nb_vars), nullptr);
        for(int i = 0; i < nb_vars; ++i) {
            SCIPcreateVarBasic(
                scip.model,     // SCIP environment
                &scip.vars[i],  // reference to the variable
                nullptr,        // name of the variable
                col_lb[i],      // Lower bound of the variable
                col_ub[i],      // upper bound of the variable
                obj[i],         // Obj. coefficient.
                static_cast<SCIP_VARTYPE>(vtype[i])  // Binary variable
            );
            if(SCIPaddVar(scip.model, scip.vars[i]) != SCIP_OKAY) {
                std::cerr << "SCIPaddVar not okay" << std::endl;
            }
        }

        std::string name;

        scip.constrs = std::vector<SCIP_CONS *>(
            static_cast<std::size_t>(nb_rows), nullptr);
        for(int i = 0; i < nb_rows; ++i) {
            int begin_elems = row_begins[i];
            int end_elems =
                (i == nb_rows - 1) ? (nb_elems) : (row_begins[i + 1]);

            name = "ROW_" + std::to_string(i);

            if(SCIP_RETCODE code = SCIPcreateConsBasicLinear(
                   scip.model, &scip.constrs[i], name.c_str(), 0, nullptr,
                   nullptr, row_lb[i], row_ub[i]);
               code != SCIP_OKAY) {
                std::cerr << "SCIPcreateConsBasicLinear not okay" << std::endl;
                SCIPerrorMessage("Error <%d> in function call\n", code);
            }

            for(int elem_num = begin_elems; elem_num != end_elems; ++elem_num) {
                SCIPaddCoefLinear(scip.model, scip.constrs[i],
                                  scip.vars[indices[elem_num]],
                                  coefs[elem_num]);
            }

            SCIPaddCons(scip.model, scip.constrs[i]);
        }

        return scip;
    }
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_CPLEX_TRAITS_HPP