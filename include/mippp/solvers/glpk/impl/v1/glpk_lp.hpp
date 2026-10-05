#pragma once

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <variant>

#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/glpk/impl/v1/glpk_base.hpp"

namespace mippp {
namespace glpk::impl::v1 {

class glpk_lp : public glpk_base {
private:
    glp_smcp model_params;
    std::chrono::duration<double> _time_limit{
        std::numeric_limits<double>::infinity()};

public:
    [[nodiscard]] glpk_lp() : glpk_lp(glpk_api::load()) {}
    [[nodiscard]] explicit glpk_lp(const glpk_api & api)
        : glpk_base(api), model_params() {
        // Let GLPK fill in its own defaults before overriding: the fields left
        // untouched below must not stay zero. GLPK <= 4.62 rejects out_frq = 0
        // outright, and xerror() aborts the process rather than reporting it.
        glp->init_smcp(&model_params);
        model_params.msg_lev = GLP_MSG_OFF;
        model_params.meth = GLP_PRIMAL;
        model_params.pricing = GLP_PT_STD;
        model_params.r_test = GLP_RT_STD;
        model_params.tol_bnd = 1e-7;
        model_params.tol_dj = 1e-7;
        model_params.tol_piv = 1e-10;
        model_params.obj_ll = std::numeric_limits<double>::lowest();
        model_params.obj_ul = std::numeric_limits<double>::max();
        model_params.it_lim = std::numeric_limits<int>::max();
        model_params.tm_lim = std::numeric_limits<int>::max();
        model_params.presolve = 0;
        model_params.excl = 0;
        model_params.shift = 0;
        model_params.aorn = GLP_USE_AT;
    }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////// Tolerance parameters ///////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void set_feasibility_tolerance(double tol) {
        model_params.tol_bnd = model_params.tol_dj = tol;
        model_params.tol_piv = tol / 100;
    }
    double get_feasibility_tolerance() { return model_params.tol_bnd; }
    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////// Limits //////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    // tm_lim is rounded, so the limit read back is the copy kept here
    void set_time_limit(std::chrono::duration<double> t) {
        model_params.tm_lim = _tm_lim(t, "glpk_lp: negative or NaN time limit");
        _time_limit = t;
    }
    auto get_time_limit() { return _time_limit; }
    ///////////////////////////////////////////////////////////////////////////
    //////////////////////////////// Verbosity ////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void set_verbose(bool verbose) {
        model_params.msg_lev = verbose ? GLP_MSG_ALL : GLP_MSG_OFF;
    }
    bool is_verbose() { return model_params.msg_lev != GLP_MSG_OFF; }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////// Solve status ///////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    // clang-format off
private:
    using status_variant = std::variant<
            status::unknown,
            status::optimal,
            status::infeasible,
            status::unbounded,
            status::limit_reached,
            status::time_limit,
            status::failed,
            status::numerical_failure>;

    status_variant _status = status::unknown{};

protected:
    // Only a return code of 0 makes glp_get_status a verdict: after any other
    // code the basic solution is wherever the search stopped.
    static status_variant _simplex_status(int ret, int generic_status,
                                          int primal_status,
                                          double objective) noexcept {
        using namespace status;
        const bool has_sol = (primal_status == GLP_FEAS);
        switch(ret) {
            case 0:          break;
            case GLP_ENOPFS: return infeasible{};
            case GLP_ENODFS: return unbounded{};
            case GLP_EOBJLL:
            case GLP_EOBJUL:
            case GLP_EITLIM: return limit_reached{has_sol};
            case GLP_ETMLIM: return time_limit{has_sol};
            case GLP_ESING:
            case GLP_ECOND:  return numerical_failure{has_sol};
            case GLP_EBADB:
            case GLP_EBOUND:
            case GLP_EFAIL:  return failed{has_sol};
            default:         return unknown{has_sol};
        }
        switch(generic_status) {
            // DBL_MAX bounds a GLP_DB column, so an unbounded model can end
            // GLP_OPT on an objective that overflowed
            case GLP_OPT:    return std::isfinite(objective)
                                    ? status_variant(optimal{})
                                    : status_variant(unbounded{});
            case GLP_NOFEAS: return infeasible{};
            case GLP_UNBND:  return unbounded{};
            // GLP_INFEAS only says that this basic solution violates a bound
            default:         return unknown{has_sol};
        }
    }
    // clang-format on

public:
    const status_variant & get_status() const { return _status; }
    void reset_status() noexcept { _status = status::unknown{}; }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////////// Solve //////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void solve() {
        reset_status();  // a solve that throws reports unknown
        const int ret = glp->simplex(model, &model_params);
        if(ret == GLP_EBOUND && _has_crossed_bounds()) {
            _status = status::infeasible{};
            return;
        }
        _status =
            _simplex_status(ret, glp->get_status(model),
                            glp->get_prim_stat(model), get_solution_value());
    }
    double get_solution_value() {
        return objective_offset + glp->get_obj_val(model);
    }
    auto get_solution() {
        auto num_vars = num_variables();
        auto solution = std::make_unique_for_overwrite<double[]>(num_vars);
        for(std::size_t var = 0u; var < num_vars; ++var) {
            solution[var] = glp->get_col_prim(model, static_cast<int>(var) + 1);
        }
        return variable_mapping(std::move(solution));
    }
    auto get_dual_solution() {
        auto num_constrs = num_constraints();
        auto solution = std::make_unique_for_overwrite<double[]>(num_constrs);
        for(std::size_t constr = 0u; constr < num_constrs; ++constr) {
            solution[constr] =
                glp->get_row_dual(model, static_cast<int>(constr) + 1);
        }
        return constraint_mapping(std::move(solution));
    }
    auto get_reduced_costs() {
        auto num_vars = num_variables();
        auto reduced_costs = std::make_unique_for_overwrite<double[]>(num_vars);
        for(std::size_t var = 0u; var < num_vars; ++var) {
            reduced_costs[var] =
                glp->get_col_dual(model, static_cast<int>(var) + 1);
        }
        return variable_mapping(std::move(reduced_costs));
    }
};

}  // namespace glpk::impl::v1
}  // namespace mippp
