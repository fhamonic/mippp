#ifndef MIPPP_SHARED_SCIP_TRAITS_HPP
#define MIPPP_SHARED_SCIP_TRAITS_HPP

#include <vector>

#include <scip/scipdefplugins.h>
#include "scip/cons_linear.h"
#include "scip/retcode.h"
#include "scip/scip.h"

#include "mippp/solver_traits/abstract_solver_wrapper.hpp"

namespace fhamonic {
namespace mippp {

struct shared_scip_solver_wrapper : public abstract_solver_wrapper {
    SCIP * model;
    std::vector<SCIP_VAR *> vars;
    std::vector<SCIP_CONS *> constrs;

    [[nodiscard]] shared_scip_solver_wrapper(const auto & m_model)
        : model(nullptr) {
        using traits = typename std::decay_t<decltype(m_model)>::solver_traits;
        SCIPcreate(&model);  // Creating the SCIP environment
        /* include default plugins */
        SCIPincludeDefaultPlugins(model);
        // Creating the SCIP Problem.
        SCIPcreateProbBasic(model, "shared_scip_solver_wrapper");

        typename traits::opt_sense sense = m_model.optimization_sense();
        int nb_vars = static_cast<int>(m_model.nb_variables());
        double const * obj = m_model.column_coefs();
        double const * col_lb = m_model.column_lower_bounds();
        double const * col_ub = m_model.column_upper_bounds();
        typename traits::var_category const * vtype = m_model.column_types();
        int nb_rows = static_cast<int>(m_model.nb_constraints());
        int nb_elems = static_cast<int>(m_model.nb_entries());
        int const * row_begins = m_model.row_begins();
        int const * indices = m_model.var_entries();
        double const * coefs = m_model.coef_entries();
        double const * row_lb = m_model.row_lower_bounds();
        double const * row_ub = m_model.row_upper_bounds();

        SCIPsetObjsense(model, static_cast<SCIP_OBJSENSE>(sense));

        vars =
            std::vector<SCIP_VAR *>(static_cast<std::size_t>(nb_vars), nullptr);
        for(int i = 0; i < nb_vars; ++i) {
            SCIPcreateVarBasic(
                model,  // SCIP environment
                &vars[static_cast<std::size_t>(
                    i)],  // reference to the variable
                nullptr,  // name of the variable
                col_lb[static_cast<std::size_t>(
                    i)],  // Lower bound of the variable
                col_ub[static_cast<std::size_t>(
                    i)],  // upper bound of the variable
                obj[static_cast<std::size_t>(i)],  // Obj. coefficient.
                static_cast<SCIP_VARTYPE>(
                    vtype[static_cast<std::size_t>(i)])  // Binary variable
            );
            if(SCIPaddVar(model, vars[static_cast<std::size_t>(i)]) !=
               SCIP_OKAY) {
                std::cerr << "SCIPaddVar not okay" << std::endl;
            }
        }

        std::string name;

        constrs = std::vector<SCIP_CONS *>(static_cast<std::size_t>(nb_rows),
                                           nullptr);
        for(int i = 0; i < nb_rows; ++i) {
            int begin_elems = row_begins[static_cast<std::size_t>(i)];
            int end_elems =
                (i == nb_rows - 1) ? (nb_elems) : (row_begins[i + 1]);

            name = "ROW_" + std::to_string(i);

            if(SCIP_RETCODE code = SCIPcreateConsBasicLinear(
                   model, &constrs[static_cast<std::size_t>(i)], name.c_str(),
                   0, nullptr, nullptr, row_lb[static_cast<std::size_t>(i)],
                   row_ub[static_cast<std::size_t>(i)]);
               code != SCIP_OKAY) {
                std::cerr << "SCIPcreateConsBasicLinear not okay" << std::endl;
                SCIPerrorMessage("Error <%d> in function call\n", code);
            }

            for(int elem_num = begin_elems; elem_num != end_elems; ++elem_num) {
                SCIPaddCoefLinear(
                    model, constrs[static_cast<std::size_t>(i)],
                    vars[static_cast<std::size_t>(indices[elem_num])],
                    coefs[elem_num]);
            }

            SCIPaddCons(model, constrs[static_cast<std::size_t>(i)]);
        }
    }
    ~shared_scip_solver_wrapper() {
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

struct shared_scip_traits {
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
    using solver_wrapper = shared_scip_solver_wrapper;

    [[nodiscard]] static shared_scip_solver_wrapper build(const auto & model) {
        return shared_scip_solver_wrapper(model);
    }
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_SHARED_SCIP_TRAITS_HPP