#pragma once

#include <chrono>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <limits>
#include <memory>
#include <utility>
#include <variant>
#include <vector>

#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"
#include "mippp/utility/basis_snapshot.hpp"
#include "mippp/utility/variant.hpp"

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
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////////// Basis //////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
private:
    // GLP_NF is given to free entities only: GLPK has no superbasic code
    using basis_variable_status =
        std::variant<basis_status::nonbasic, basis_status::basic,
                     basis_status::nonbasic_at_lower_bound,
                     basis_status::nonbasic_at_upper_bound,
                     basis_status::nonbasic_fixed, basis_status::nonbasic_free>;
    using basis_constraint_status =
        std::variant<basis_status::basic, basis_status::nonbasic,
                     basis_status::nonbasic_at_lower_bound,
                     basis_status::nonbasic_at_upper_bound,
                     basis_status::nonbasic_fixed, basis_status::nonbasic_free>;
    using basis_type =
        basis_snapshot<variable, constraint, basis_variable_status,
                       basis_constraint_status>;

    template <typename Status>
    static Status _basis_status(int code) noexcept {
        switch(code) {
            case GLP_BS:
                return Status(std::in_place_type<basis_status::basic>);
            case GLP_NL:
                return Status(
                    std::in_place_type<basis_status::nonbasic_at_lower_bound>);
            case GLP_NU:
                return Status(
                    std::in_place_type<basis_status::nonbasic_at_upper_bound>);
            case GLP_NS:
                return Status(std::in_place_type<basis_status::nonbasic_fixed>);
            case GLP_NF:
                return Status(std::in_place_type<basis_status::nonbasic_free>);
            default:
                return Status(std::in_place_type<basis_status::nonbasic>);
        }
    }

protected:
    // clang-format off
    // GLPK replaces a nonbasic code the entity's bounds do not allow: NL
    // stands for bare nonbasic, and NF on a bounded entity lands on a bound.
    template <typename Status>
    static constexpr int _basis_code(const Status & status) noexcept {
        using namespace basis_status;
        return std::visit<int>(detail::overloaded{
            [](basic)                   { return GLP_BS; },
            [](nonbasic_at_lower_bound) { return GLP_NL; },
            [](nonbasic_at_upper_bound) { return GLP_NU; },
            [](nonbasic_fixed)          { return GLP_NS; },
            [](nonbasic_free)           { return GLP_NF; },
            [](nonbasic)                { return GLP_NL; }},
            status);
    }
    // clang-format on

public:
    // GLPK always holds a basis, a new model's included.
    auto get_basis() {
        if(num_variables() == 0u) return basis_type{};
        const std::size_t num_col = num_variables();
        const std::size_t num_row = num_constraints();
        detail::handle_status_table<basis_variable_status> columns(num_col);
        for(std::size_t j = 0; j < num_col; ++j)
            columns.set(j,
                        _basis_status<basis_variable_status>(
                            glp->get_col_stat(model, static_cast<int>(j) + 1)));
        detail::handle_status_table<basis_constraint_status> rows(num_row);
        for(std::size_t i = 0; i < num_row; ++i)
            rows.set(i, _basis_status<basis_constraint_status>(
                            glp->get_row_stat(model, static_cast<int>(i) + 1)));
        return basis_type(std::move(columns), std::move(rows));
    }
    // A basis GLPK cannot use is replaced by its standard one, so the next
    // solve starts cold: a basic column with no matrix entry can make GLPK's
    // factorizer abort the process, and a basis glp_factorize refuses
    // would fail every later solve. glp_warm_up would overwrite the last
    // solution, and glp_adv_basis prints whatever msg_lev says.
    template <detail::lp_basis_over<variable, constraint> B>
    void set_basis(const B & basis) {
        const std::size_t num_col = num_variables();
        const std::size_t num_row = num_constraints();
        std::vector<int> col_stat(num_col);
        for(std::size_t j = 0; j < num_col; ++j) {
            const variable v(static_cast<int>(j));
            col_stat[j] =
                _basis_code(detail::basis_status_as<basis_variable_status>(
                    basis.get_status(v)));
            if(col_stat[j] == GLP_BS &&
               glp->get_mat_col(model, native_id(v), nullptr, nullptr) == 0) {
                glp->std_basis(model);
                return;
            }
        }
        for(std::size_t j = 0; j < num_col; ++j)
            glp->set_col_stat(model, static_cast<int>(j) + 1, col_stat[j]);
        for(std::size_t i = 0; i < num_row; ++i)
            glp->set_row_stat(
                model, static_cast<int>(i) + 1,
                _basis_code(detail::basis_status_as<basis_constraint_status>(
                    basis.get_status(constraint(static_cast<int>(i))))));
        if(glp->factorize(model) != 0) glp->std_basis(model);
    }
};

}  // namespace glpk::impl::v1
}  // namespace mippp
