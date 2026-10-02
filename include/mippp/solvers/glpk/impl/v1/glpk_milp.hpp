#pragma once

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <optional>
#include <ranges>
#include <utility>
#include <variant>
#include <vector>

#include "mippp/linear_constraint.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

#include "mippp/solvers/glpk/impl/v1/glpk_base.hpp"

namespace mippp {
namespace glpk::impl::v1 {

class glpk_milp : public glpk_base {
private:
    glp_iocp model_params;
    std::chrono::duration<double> _time_limit{
        std::numeric_limits<double>::infinity()};

public:
    [[nodiscard]] glpk_milp() : glpk_milp(glpk_api::load()) {}
    [[nodiscard]] explicit glpk_milp(const glpk_api & api)
        : glpk_base(api), model_params() {
        // See glpk_lp: the untouched fields must carry GLPK's defaults, not
        // zeros, or out_frq = 0 aborts the process on GLPK <= 4.62.
        glp->init_iocp(&model_params);
        model_params.msg_lev = GLP_MSG_OFF;
        model_params.br_tech = GLP_BR_PCH;
        model_params.bt_tech = GLP_BT_BLB;
        model_params.tol_int = 1e-6;
        model_params.tol_obj = 1e-7;
        model_params.tm_lim = std::numeric_limits<int>::max();
        model_params.pp_tech = GLP_PP_ROOT;
        model_params.mip_gap = 1e-4;
        model_params.mir_cuts = GLP_ON;
        model_params.gmi_cuts = GLP_ON;
        model_params.cov_cuts = GLP_ON;
        model_params.clq_cuts = GLP_ON;
        model_params.presolve = GLP_ON;
        model_params.binarize = GLP_OFF;
        model_params.fp_heur = GLP_ON;
        model_params.ps_heur = GLP_OFF;
        model_params.ps_tm_lim = 1000;
        model_params.sr_heur = GLP_ON;
    }

    using model_base<int, double>::add_integer_variable;
    using model_base<int, double>::add_integer_variables;
    using model_base<int, double>::add_binary_variable;
    using model_base<int, double>::add_binary_variables;

private:
    inline void _add_binary_variables(const std::size_t & offset,
                                      const std::size_t & count) {
        if(count == 0u) return;
        glp->add_cols(model, static_cast<int>(count));
        for(std::size_t i = offset + 1; i <= offset + count; ++i)
            glp->set_col_kind(model, static_cast<int>(i), GLP_BV);
    }

public:
    void set_continuous(variable v) noexcept {
        glp->set_col_kind(model, v.id() + 1, GLP_CV);
    }
    void set_integer(variable v) noexcept {
        glp->set_col_kind(model, v.id() + 1, GLP_IV);
    }
    void set_binary(variable v) noexcept {
        glp->set_col_kind(model, v.id() + 1, GLP_BV);
    }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////// Tolerance parameters ///////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    // glp_iocp::tol_int is how far from an integer a value may sit, i.e. the
    // integrality tolerance -- not the primal/dual feasibility tolerance
    // glpk_lp exposes through glp_smcp::tol_bnd/tol_dj.
    void set_integrality_tolerance(double tol) {
        model_params.tol_int = tol;
        model_params.tol_obj = tol / 10;
    }
    double get_integrality_tolerance() { return model_params.tol_int; }
    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////// Limits //////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    // tm_lim is rounded, so the limit read back is the copy kept here. The
    // search stops once tm_lim - 1 ms have passed, a millisecond early, so
    // one is added: a deadline forwarded as a limit then ends the solve at
    // or after the deadline, never just before it.
    void set_time_limit(std::chrono::duration<double> t) {
        const int milliseconds =
            _tm_lim(t, "glpk_milp: negative or NaN time limit");
        model_params.tm_lim = milliseconds < std::numeric_limits<int>::max()
                                  ? milliseconds + 1
                                  : milliseconds;
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
            status::infeasible_or_unbounded,
            status::infeasible,
            status::unbounded,
            status::time_limit,
            status::failed,
            status::interrupted>;

    status_variant _status = status::unknown{};

protected:
    // 0 only says the search completed: infeasibility is reported through
    // glp_mip_status, and a stop may still leave an incumbent
    static status_variant _intopt_status(int ret, int mip_status) noexcept {
        using namespace status;
        const bool has_sol = (mip_status == GLP_FEAS || mip_status == GLP_OPT);
        switch(ret) {
            case 0:
                if(mip_status == GLP_OPT)    return optimal{};
                if(mip_status == GLP_NOFEAS) return infeasible{};
                return unknown{has_sol};
            case GLP_EMIPGAP: return optimal{};
            case GLP_ENOPFS:  return infeasible{};
            // the LP relaxation has no dual feasible solution: the search
            // stops before knowing whether any integer point exists
            case GLP_ENODFS:  return infeasible_or_unbounded{};
            case GLP_ETMLIM:  return time_limit{has_sol};
            case GLP_ESTOP:   return interrupted{has_sol};
            case GLP_EBOUND:
            case GLP_EROOT:
            case GLP_EFAIL:   return failed{has_sol};
            default:          return unknown{has_sol};
        }
    }
    // clang-format on
public:
    const status_variant & get_status() const { return _status; }
    void reset_status() noexcept { _status = status::unknown{}; }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////////// Solve //////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
private:
    struct saved_sides {
        bool row;
        int index;
        int type;
        double lb;
        double ub;
    };
    // Within the integrality tolerance a side rounds to the nearest integer,
    // as GLPK's presolver rounds implied bounds, so that float noise on an
    // integral side does not cut its integer off: a side just past an integer
    // moves out to it. Empty when both sides already are integers.
    std::optional<std::pair<double, double>> _integral_sides(double lb,
                                                             double ub) const {
        const double tol = model_params.tol_int;
        const bool has_lb = lb > std::numeric_limits<double>::lowest();
        const bool has_ub = ub < std::numeric_limits<double>::max();
        const double rounded_lb = has_lb ? std::ceil(lb - tol) : lb;
        const double rounded_ub = has_ub ? std::floor(ub + tol) : ub;
        if((!has_lb || rounded_lb == lb) && (!has_ub || rounded_ub == ub))
            return std::nullopt;
        return std::pair{rounded_lb, rounded_ub};
    }
    // glp_intopt refuses a fractional bound on an integer column with
    // GLP_EBOUND, and its branch-and-bound can run without end on a row whose
    // sides hold no integer between them over unbounded integer columns,
    // x + y == 1.5 for one. An integer column, and a row whose columns are all
    // integer with integral coefficients, only take integer values, so
    // rounding their sides to integers loses no solution up to the
    // integrality tolerance, the slack glp_intopt already gives an integer
    // column. glp_intopt sees the rounded sides, and the caller's are put back
    // after the solve, which keeps the solution. No integer between two sides
    // leaves them crossed, which the solve then reports as infeasible.
    // Rounding does not reach a row with integral sides but no integer point,
    // 2x + 2y == 1, nor a row with a fractional coefficient: glp_intopt can
    // still branch on those until a time limit stops it.
    //
    // Everything is allocated before the first side changes, so that a
    // bad_alloc cannot leave the caller's sides rounded.
    std::vector<saved_sides> _round_integral_sides() {
        std::vector<saved_sides> saved;
        const int num_cols = glp->get_num_cols(model);
        const auto num_slots = static_cast<std::size_t>(num_cols) + 1u;
        std::vector<char> is_integer(num_slots, char{0});
        bool any_integer = false;
        for(int j = 1; j <= num_cols; ++j) {
            if(glp->get_col_kind(model, j) == GLP_CV) continue;
            is_integer[static_cast<std::size_t>(j)] = 1;
            any_integer = true;
        }
        if(!any_integer) return saved;
        const int num_rows = glp->get_num_rows(model);
        std::vector<int> ind(num_slots);
        std::vector<double> val(num_slots);
        saved.reserve(static_cast<std::size_t>(num_cols) +
                      static_cast<std::size_t>(num_rows));
        for(int j = 1; j <= num_cols; ++j) {
            if(!is_integer[static_cast<std::size_t>(j)]) continue;
            const double lb = glp->get_col_lb(model, j);
            const double ub = glp->get_col_ub(model, j);
            const auto rounded = _integral_sides(lb, ub);
            if(!rounded) continue;
            saved.push_back({false, j, glp->get_col_type(model, j), lb, ub});
            _set_col_bnds(j, rounded->first, rounded->second);
        }
        for(int i = 1; i <= num_rows; ++i) {
            const int len = glp->get_mat_row(model, i, ind.data(), val.data());
            bool integral = true;
            for(std::size_t k = 1; k <= static_cast<std::size_t>(len); ++k) {
                if(!is_integer[static_cast<std::size_t>(ind[k])] ||
                   val[k] != std::floor(val[k])) {
                    integral = false;
                    break;
                }
            }
            if(!integral) continue;
            const double lb = glp->get_row_lb(model, i);
            const double ub = glp->get_row_ub(model, i);
            const auto rounded = _integral_sides(lb, ub);
            if(!rounded) continue;
            saved.push_back({true, i, glp->get_row_type(model, i), lb, ub});
            _set_row_bnds(i, rounded->first, rounded->second);
        }
        return saved;
    }
    void _restore_sides(const std::vector<saved_sides> & saved) {
        for(const saved_sides & s : saved) {
            if(s.row)
                glp->set_row_bnds(model, s.index, s.type, s.lb, s.ub);
            else
                glp->set_col_bnds(model, s.index, s.type, s.lb, s.ub);
        }
    }

public:
    void solve() {
        // Rounding may throw, so it goes before the switch it would leave off.
        const std::vector<saved_sides> saved = _round_integral_sides();
        // GLPK prints its cover and clique cut setup whatever msg_lev says.
        // glp_term_out switches the calling thread's GLPK environment (the
        // whole process's, on a GLPK built without thread-local storage).
        const bool quiet = !is_verbose();
        const int term_out = quiet ? glp->term_out(GLP_OFF) : GLP_ON;
        const int ret = glp->intopt(model, &model_params);
        const bool crossed = (ret == GLP_EBOUND) && _has_crossed_bounds();
        _restore_sides(saved);
        if(quiet) glp->term_out(term_out);
        if(crossed) {
            _status = status::infeasible{};
            return;
        }
        _status = _intopt_status(ret, glp->mip_status(model));
    }
    double get_solution_value() {
        return objective_offset + glp->mip_obj_val(model);
    }
    auto get_solution() {
        auto num_vars = num_variables();
        auto solution = std::make_unique_for_overwrite<double[]>(num_vars);
        for(std::size_t var = 0u; var < num_vars; ++var) {
            solution[var] = glp->mip_col_val(model, static_cast<int>(var) + 1);
        }
        return variable_mapping(std::move(solution));
    }
};

}  // namespace glpk::impl::v1
}  // namespace mippp
