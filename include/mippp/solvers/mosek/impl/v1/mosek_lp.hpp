#pragma once

#include <concepts>
#include <cstddef>
#include <memory>
#include <optional>
#include <utility>
#include <variant>
#include <vector>

#include "mippp/detail/lp_algorithm_codec.hpp"
#include "mippp/detail/restore_guard.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"
#include "mippp/utility/basis_snapshot.hpp"
#include "mippp/utility/solver_exceptions.hpp"
#include "mippp/utility/variant.hpp"

#include "mippp/solvers/mosek/impl/v1/mosek_base.hpp"

namespace mippp {
namespace mosek::impl::v1 {

class mosek_lp : public mosek_base {
public:
    [[nodiscard]] mosek_lp() : mosek_lp(mosek_api::load()) {}
    [[nodiscard]] explicit mosek_lp(const mosek_api & api) : mosek_base(api) {}

    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////// LP algorithm ///////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
private:
    using lp_algorithm_variant =
        std::variant<lp_algorithm::automatic, lp_algorithm::simplex,
                     lp_algorithm::primal_simplex, lp_algorithm::dual_simplex,
                     lp_algorithm::interior_point,
                     lp_algorithm::interior_point_without_crossover,
                     lp_algorithm::other>;

protected:
    struct lp_algorithm_knobs {
        MSKint32t optimizer, basis_identification;
        bool operator==(const lp_algorithm_knobs &) const = default;
    };
    // MSK_BI_ALWAYS is MOSEK's default: its interior point then ends on a
    // basic solution, and MSK_BI_NEVER leaves the interior one alone.
    template <typename A>
    static constexpr lp_algorithm_knobs _lp_algorithm_knobs(A) noexcept {
        using namespace lp_algorithm;
        if constexpr(std::same_as<A, automatic>)
            return {MSK_OPTIMIZER_FREE, MSK_BI_ALWAYS};
        else if constexpr(std::same_as<A, simplex>)
            return {MSK_OPTIMIZER_FREE_SIMPLEX, MSK_BI_ALWAYS};
        else if constexpr(std::same_as<A, primal_simplex>)
            return {MSK_OPTIMIZER_PRIMAL_SIMPLEX, MSK_BI_ALWAYS};
        else if constexpr(std::same_as<A, dual_simplex>)
            return {MSK_OPTIMIZER_DUAL_SIMPLEX, MSK_BI_ALWAYS};
        else if constexpr(std::same_as<A, interior_point>)
            return {MSK_OPTIMIZER_INTPNT, MSK_BI_ALWAYS};
        else if constexpr(std::same_as<A, interior_point_without_crossover>)
            return {MSK_OPTIMIZER_INTPNT, MSK_BI_NEVER};
        else
            static_assert(std::same_as<A, void>, "a listed tag has no row");
    }

public:
    // Both parameters at every call: a basis identification left off would
    // make the next interior_point, and automatic, end without a basis too.
    template <detail::settable_lp_algorithm<lp_algorithm_variant> A>
    void set_lp_algorithm(A algorithm) {
        const lp_algorithm_knobs knobs = _lp_algorithm_knobs(algorithm);
        check(MSK->putintparam(task, MSK_IPAR_OPTIMIZER, knobs.optimizer));
        check(MSK->putintparam(task, MSK_IPAR_INTPNT_BASIS,
                               knobs.basis_identification));
    }
    lp_algorithm_variant get_lp_algorithm() {
        lp_algorithm_knobs knobs;
        check(MSK->getintparam(task, MSK_IPAR_OPTIMIZER, &knobs.optimizer));
        check(MSK->getintparam(task, MSK_IPAR_INTPNT_BASIS,
                               &knobs.basis_identification));
        return detail::decode_lp_algorithm(
            lp_algorithm_variant{lp_algorithm::other{}}, knobs,
            [](auto algorithm) { return _lp_algorithm_knobs(algorithm); });
    }
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
            status::primal_and_dual_infeasible,
            status::unbounded,
            status::limit_reached,
            status::time_limit,
            status::iteration_limit,
            status::failed,
            status::numerical_failure,
            status::interrupted>;

    status_variant _status = status::unknown{};
    std::optional<MSKsoltypee> _solution;
    // set by set_basis() for the next solve() only, and left alone by the
    // reset_status() that solve() calls first
    bool _starts_from_basis = false;

    // whether a stopped solve left the solution get_solution() reads
    bool _has_solution() {
        if(!_solution) return false;
        MSKsolstae solsta;
        check(MSK->getsolsta(task, *_solution, &solsta));
        return _has_primal_solution(solsta);
    }

    status_variant _get_status(MSKrescodee trm) {
        using namespace status;
        switch(trm) {
            case MSK_RES_OK: {
                if(!_solution) return unknown{};
                MSKprostae prosta;
                check(MSK->getprosta(task, *_solution, &prosta));
                switch(prosta) {
                    case MSK_PRO_STA_PRIM_AND_DUAL_FEAS: {
                        MSKsolstae solsta;
                        check(MSK->getsolsta(task, *_solution, &solsta));
                        switch (solsta) {
                            case MSK_SOL_STA_OPTIMAL:
                            case MSK_SOL_STA_INTEGER_OPTIMAL:  return optimal{};
                            default: 
                                return unknown{};
                        }
                    }
                    case MSK_PRO_STA_PRIM_INFEAS_OR_UNBOUNDED:
                                                  return infeasible_or_unbounded{};
                    case MSK_PRO_STA_PRIM_INFEAS: return infeasible{};
                    case MSK_PRO_STA_PRIM_AND_DUAL_INFEAS:
                                                  return primal_and_dual_infeasible{};
                    case MSK_PRO_STA_DUAL_INFEAS: return unbounded{};
                    case MSK_PRO_STA_PRIM_FEAS:
                    case MSK_PRO_STA_DUAL_FEAS:
                    case MSK_PRO_STA_ILL_POSED:
                    case MSK_PRO_STA_UNKNOWN:
                    default: 
                        return unknown{};
                }
            }
            case MSK_RES_TRM_MAX_TIME:          return time_limit{_has_solution()};
            case MSK_RES_TRM_MAX_ITERATIONS:    return iteration_limit{_has_solution()};
            case MSK_RES_TRM_OBJECTIVE_RANGE:   return limit_reached{_has_solution()};
            case MSK_RES_TRM_USER_CALLBACK:     return interrupted{_has_solution()};
            case MSK_RES_TRM_NUMERICAL_PROBLEM:
            case MSK_RES_TRM_MAX_NUM_SETBACKS:
            case MSK_RES_TRM_STALL:             return numerical_failure{_has_solution()};
            case MSK_RES_TRM_LOST_RACE:
            case MSK_RES_TRM_INTERNAL:
            case MSK_RES_TRM_INTERNAL_STOP:     return failed{_has_solution()};
            default: 
                return unknown{};
        }
    }
    MSKsoltypee _require_solution() const {
        if(!_solution) throw solver_error("MOSEK holds no solution");
        return *_solution;
    }
    // clang-format on
public:
    const status_variant & get_status() const {
        return _results_valid ? _status
                              : detail::unknown_status<status_variant>;
    }
    void reset_status() noexcept {
        _status = status::unknown{};
        _solution.reset();
        _invalidate_results();
    }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////////// Solve //////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
private:
    // MOSEK's automatic choice runs the interior point, which ignores a
    // basis: the solve after set_basis() runs a simplex instead, and an
    // optimizer chosen natively is kept.
    bool _switches_to_simplex() {
        if(!std::exchange(_starts_from_basis, false)) return false;
        MSKint32t optimizer;
        check(MSK->getintparam(task, MSK_IPAR_OPTIMIZER, &optimizer));
        return optimizer == MSK_OPTIMIZER_FREE;
    }
    // An interior point writes the basic solution through its basis
    // identification alone. Without one, or stopped before it, a solve
    // leaves the basic solution of an earlier solve in place, which
    // get_basis() would read as this solve's. A simplex starts from it.
    void _drop_the_basis_no_simplex_starts_from() {
        MSKint32t optimizer;
        check(MSK->getintparam(task, MSK_IPAR_OPTIMIZER, &optimizer));
        switch(optimizer) {
            case MSK_OPTIMIZER_FREE_SIMPLEX:
            case MSK_OPTIMIZER_PRIMAL_SIMPLEX:
            case MSK_OPTIMIZER_DUAL_SIMPLEX:
            case MSK_OPTIMIZER_NEW_PRIMAL_SIMPLEX:
            case MSK_OPTIMIZER_NEW_DUAL_SIMPLEX:
                return;
        }
        MSKbooleant defined = 0;
        check(MSK->solutiondef(task, MSK_SOL_BAS, &defined));
        if(defined) check(MSK->deletesolution(task, MSK_SOL_BAS));
    }

public:
    void solve() {
        reset_status();  // a solve that throws reports unknown
        // MSK_optimize reports a limit as an error code; MSK_optimizetrm
        // returns it as the termination code, and is the solve itself
        MSKrescodee trm = MSK_RES_OK;
        if(_switches_to_simplex()) {
            check(MSK->putintparam(task, MSK_IPAR_OPTIMIZER,
                                   MSK_OPTIMIZER_FREE_SIMPLEX));
            detail::restore_guard guard([&] {
                check(MSK->putintparam(task, MSK_IPAR_OPTIMIZER,
                                       MSK_OPTIMIZER_FREE));
            });
            check(MSK->optimizetrm(task, &trm));
            guard.restore();
        } else {
            _drop_the_basis_no_simplex_starts_from();
            check(MSK->optimizetrm(task, &trm));
        }
        _solution = _pick_continuous_solution();
        _status = _get_status(trm);
        _mark_results_valid();
    }
    double get_solution_value() {
        _require_valid_results();
        double val;
        check(MSK->getprimalobj(task, _require_solution(), &val));
        return val;
    }
    auto get_solution() {
        _require_valid_results();
        auto solution =
            std::make_unique_for_overwrite<double[]>(num_variables());
        check(MSK->getxx(task, _require_solution(), solution.get()));
        return variable_mapping(std::move(solution));
    }
    auto get_dual_solution() {
        _require_valid_results();
        auto dual_solution =
            std::make_unique_for_overwrite<double[]>(num_constraints());
        check(MSK->getsolution(task, _require_solution(), nullptr, nullptr,
                               nullptr, nullptr, nullptr, nullptr, nullptr,
                               dual_solution.get(), nullptr, nullptr, nullptr,
                               nullptr, nullptr));
        return constraint_mapping(std::move(dual_solution));
    }
    auto get_reduced_costs() {
        _require_valid_results();
        const auto num_vars = num_variables();
        auto reduced_costs = std::make_unique_for_overwrite<double[]>(num_vars);
        check(MSK->getreducedcosts(task, _require_solution(), 0,
                                   static_cast<int>(num_vars),
                                   reduced_costs.get()));
        return variable_mapping(std::move(reduced_costs));
    }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////////// Basis //////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
private:
    using basis_variable_status =
        std::variant<basis_status::nonbasic, basis_status::basic,
                     basis_status::nonbasic_at_lower_bound,
                     basis_status::nonbasic_at_upper_bound,
                     basis_status::nonbasic_fixed, basis_status::superbasic>;
    using basis_constraint_status =
        std::variant<basis_status::basic, basis_status::nonbasic,
                     basis_status::nonbasic_at_lower_bound,
                     basis_status::nonbasic_at_upper_bound,
                     basis_status::nonbasic_fixed, basis_status::superbasic>;
    using basis_type =
        basis_snapshot<variable, constraint, basis_variable_status,
                       basis_constraint_status>;

    // clang-format off
    // SUPBAS also marks a bounded entity strictly inside its bounds, as a stop
    // of the dual simplex or the certificate of an unbounded LP leaves some. A
    // defined slot holds UNK keys after a stop before the first pivot, and INF
    // on crossed bounds.
    template <typename Status>
    static Status _basis_status(MSKstakeye key) {
        using namespace basis_status;
        switch(key) {
            case MSK_SK_BAS:    return basic{};
            case MSK_SK_LOW:    return nonbasic_at_lower_bound{};
            case MSK_SK_UPR:    return nonbasic_at_upper_bound{};
            case MSK_SK_FIX:    return nonbasic_fixed{};
            case MSK_SK_SUPBAS: return superbasic{};
            default:
                throw solver_error(
                    "mippp: get_basis() found no basis, MOSEK's basic "
                    "solution holds an unknown or infeasible status key");
        }
    }

protected:
    // MOSEK snaps a side the bounds do not have at the next solve.
    template <typename Status>
    static constexpr MSKstakeye _basis_key(const Status & status) noexcept {
        using namespace basis_status;
        return std::visit<MSKstakeye>(detail::overloaded{
            [](basic)                   { return MSK_SK_BAS; },
            [](nonbasic_at_lower_bound) { return MSK_SK_LOW; },
            [](nonbasic_at_upper_bound) { return MSK_SK_UPR; },
            [](nonbasic_fixed)          { return MSK_SK_FIX; },
            [](superbasic)              { return MSK_SK_SUPBAS; },
            [](nonbasic)                { return MSK_SK_LOW; }},
            status);
    }
    // clang-format on

public:
    // Reads the basic slot whichever slot solve() picked: the interior one
    // holds keys that are no basis. MOSEK keeps its keys through changes of
    // the model, and a column appended since reads FIX.
    auto get_basis() {
        _require_valid_results();
        if(num_variables() == 0u) return basis_type{};
        MSKbooleant defined = 0;
        check(MSK->solutiondef(task, MSK_SOL_BAS, &defined));
        if(!defined)
            throw solver_error(
                "mippp: get_basis() found no basis, MOSEK holds no basic "
                "solution");
        const std::size_t num_col = num_variables();
        const std::size_t num_row = num_constraints();
        std::vector<MSKstakeye> skx(num_col), skc(num_row);
        check(MSK->getsolution(task, MSK_SOL_BAS, nullptr, nullptr, skc.data(),
                               skx.data(), nullptr, nullptr, nullptr, nullptr,
                               nullptr, nullptr, nullptr, nullptr, nullptr));
        detail::handle_status_table<basis_variable_status> columns(num_col);
        for(std::size_t j = 0; j < num_col; ++j)
            columns.set(j, _basis_status<basis_variable_status>(skx[j]));
        detail::handle_status_table<basis_constraint_status> rows(num_row);
        for(std::size_t i = 0; i < num_row; ++i)
            rows.set(i, _basis_status<basis_constraint_status>(skc[i]));
        return basis_type(std::move(columns), std::move(rows));
    }
    template <detail::lp_basis_over<variable, constraint> B>
    void set_basis(const B & basis) {
        _invalidate_results();
        const std::size_t num_col = num_variables();
        const std::size_t num_row = num_constraints();
        std::vector<MSKstakeye> skx(num_col), skc(num_row);
        for(std::size_t j = 0; j < num_col; ++j)
            skx[j] = _basis_key(detail::basis_status_as<basis_variable_status>(
                basis.get_status(variable(static_cast<int>(j)))));
        for(std::size_t i = 0; i < num_row; ++i)
            skc[i] =
                _basis_key(detail::basis_status_as<basis_constraint_status>(
                    basis.get_status(constraint(static_cast<int>(i)))));
        check(MSK->putskx(task, MSK_SOL_BAS, skx.data()));
        check(MSK->putskc(task, MSK_SOL_BAS, skc.data()));
        _starts_from_basis = true;
    }
};

}  // namespace mosek::impl::v1
}  // namespace mippp
