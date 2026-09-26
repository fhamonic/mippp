#pragma once

#include <algorithm>
#include <cstddef>
#include <limits>
#include <memory>
#include <ranges>
#include <tuple>
#include <utility>
#include <variant>
#include <vector>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"
#include "mippp/quadratic_expression.hpp"

#include "mippp/solvers/highs/impl/v1/highs_base.hpp"

namespace mippp {
namespace highs::impl::v1 {

class highs_qp : public highs_base {
public:
    [[nodiscard]] highs_qp() : highs_qp(highs_api::load()) {}
    [[nodiscard]] explicit highs_qp(const highs_api & api) : highs_base(api) {}

private:
    bool _has_hessian = false;
    std::vector<std::tuple<HighsInt, HighsInt, double>> tmp_quadratic_entries;

    // HiGHS rejects dim 0 on a non-empty model: the empty Hessian is passed
    // with the full dimension and no entries
    void _clear_hessian() {
        if(!_has_hessian) return;
        const auto num_vars = _num_var_native_ids();
        tmp_begins.assign(num_vars, 0);
        check(Highs->passHessian(model, static_cast<HighsInt>(num_vars), 0,
                                 kHighsHessianFormatTriangular,
                                 tmp_begins.data(), nullptr, nullptr));
        _has_hessian = false;
    }

public:
    template <linear_expression LE>
    void set_objective(LE && le) {
        _clear_hessian();
        highs_base::set_objective(std::forward<LE>(le));
    }
    template <linear_expression LE>
    void set_objective(distinct_variables_t, LE && le) {
        set_objective(std::forward<LE>(le));
    }

    template <quadratic_expression QE>
    void set_quadratic_objective(QE && qe) {
        const auto num_vars = _num_var_native_ids();
        highs_base::set_objective(qe.linear_part());
        tmp_quadratic_entries.resize(0);
        // HiGHS minimizes ½·xᵀQx over the lower triangle of Q: a diagonal
        // coefficient is passed doubled, an off-diagonal one as is because
        // its mirror image supplies the other half; get_quadratic_objective
        // undoes this.
        for(auto && [var1, var2, coef] : qe.quadratic_terms()) {
            HighsInt i = _native_id(var1);
            HighsInt j = _native_id(var2);
            if(j < i) std::swap(i, j);
            tmp_quadratic_entries.emplace_back(
                i, j, (i == j ? 2.0 : 1.0) * static_cast<double>(coef));
        }
        std::ranges::sort(tmp_quadratic_entries, {}, [](const auto & e) {
            return std::make_pair(std::get<0>(e), std::get<1>(e));
        });
        tmp_begins.resize(num_vars);
        tmp_indices.resize(0);
        tmp_scalars.resize(0);
        HighsInt next_col = 0;
        HighsInt last_i = -1, last_j = -1;
        for(auto && [i, j, value] : tmp_quadratic_entries) {
            if(i == last_i && j == last_j) {
                tmp_scalars.back() += value;
                continue;
            }
            while(next_col <= i)
                tmp_begins[static_cast<std::size_t>(next_col++)] =
                    static_cast<HighsInt>(tmp_scalars.size());
            tmp_indices.emplace_back(j);
            tmp_scalars.emplace_back(value);
            last_i = i;
            last_j = j;
        }
        while(next_col < static_cast<HighsInt>(num_vars))
            tmp_begins[static_cast<std::size_t>(next_col++)] =
                static_cast<HighsInt>(tmp_scalars.size());
        check(Highs->passHessian(model, static_cast<HighsInt>(num_vars),
                                 static_cast<HighsInt>(tmp_scalars.size()),
                                 kHighsHessianFormatTriangular,
                                 tmp_begins.data(), tmp_indices.data(),
                                 tmp_scalars.data()));
        _has_hessian = !tmp_scalars.empty();
    }
    template <quadratic_expression QE>
    void set_quadratic_objective(distinct_variables_t, QE && qe) {
        set_quadratic_objective(std::forward<QE>(qe));
    }

private:
    struct hessian_data {
        std::vector<HighsInt> start;
        std::vector<HighsInt> index;
        std::vector<double> value;
    };

public:
    // Highs_getModel copies the whole model: everything but the Hessian
    // goes to scratch, the Hessian into arrays the view keeps alive. HiGHS
    // stores every diagonal entry, zero or not, hence the filter.
    auto get_quadratic_objective() {
        const auto num_vars = _num_var_native_ids();
        auto q = std::make_shared<hessian_data>();
        q->start.assign(num_vars + 1, 0);
        if(_has_hessian) {
            const auto num_rows =
                static_cast<std::size_t>(Highs->getNumRow(model));
            const auto num_nz =
                static_cast<std::size_t>(Highs->getNumNz(model));
            const auto hessian_nz =
                static_cast<std::size_t>(Highs->getHessianNumNz(model));
            q->index.resize(hessian_nz);
            q->value.resize(hessian_nz);
            tmp_scalars.resize(3 * num_vars + 2 * num_rows + num_nz);
            tmp_indices.resize(2 * num_vars + num_nz + 2);
            HighsInt n_col, n_row, n_nz, h_nz, sense;
            double offset;
            double * const scalars = tmp_scalars.data();
            HighsInt * const indices = tmp_indices.data();
            check(Highs->getModel(
                model, kHighsMatrixFormatColwise, kHighsHessianFormatTriangular,
                &n_col, &n_row, &n_nz, &h_nz, &sense, &offset, scalars,
                scalars + num_vars, scalars + 2 * num_vars,
                scalars + 3 * num_vars, scalars + 3 * num_vars + num_rows,
                indices, indices + num_vars + 1,
                scalars + 3 * num_vars + 2 * num_rows, q->start.data(),
                q->index.data(), q->value.data(),
                indices + num_vars + 1 + num_nz));
            q->start[num_vars] = h_nz;
        }
        return quadratic_expression_view(
            std::views::join(std::views::transform(
                std::views::iota(index{0}, static_cast<index>(num_vars)),
                [this, q](index i) {
                    return std::views::transform(
                        std::views::filter(
                            std::views::iota(
                                q->start[static_cast<std::size_t>(i)],
                                q->start[static_cast<std::size_t>(i) + 1]),
                            [q](HighsInt k) {
                                return q->value[static_cast<std::size_t>(k)] !=
                                       0.0;
                            }),
                        [this, q, i](HighsInt k) {
                            const HighsInt j =
                                q->index[static_cast<std::size_t>(k)];
                            return std::make_tuple(
                                _var_handle(i), _var_handle(j),
                                (i == j ? 0.5 : 1.0) *
                                    q->value[static_cast<std::size_t>(k)]);
                        });
                })),
            get_objective());
    }
    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////// Limits //////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    // A linear objective is solved by the simplex method and a quadratic one
    // by the QP solver, each with its own limit: both are set.
    void set_iteration_limit(std::size_t n) {
        const int limit = static_cast<int>(
            std::min<std::size_t>(n, std::numeric_limits<int>::max()));
        check(
            Highs->setIntOptionValue(model, "simplex_iteration_limit", limit));
        check(Highs->setIntOptionValue(model, "qp_iteration_limit", limit));
    }
    std::size_t get_iteration_limit() {
        int simplex_limit, qp_limit;
        check(Highs->getIntOptionValue(model, "simplex_iteration_limit",
                                       &simplex_limit));
        check(Highs->getIntOptionValue(model, "qp_iteration_limit", &qp_limit));
        return static_cast<std::size_t>(std::min(simplex_limit, qp_limit));
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
            status::unbounded,
            status::limit_reached,
            status::time_limit,
            status::iteration_limit,
            status::failed,
            status::interrupted>;

    status_variant _status = status::unknown{};

    status_variant _get_status() {
        using namespace status;
        const int status_ = Highs->getModelStatus(model);
        switch (status_) {
            case kHighsModelStatusOptimal:        return optimal{};
            case kHighsModelStatusUnboundedOrInfeasible: 
                                                  return infeasible_or_unbounded{};
            case kHighsModelStatusInfeasible:     return infeasible{};
            case kHighsModelStatusUnbounded:      return unbounded{};
        }
        int psolstatus;
        check(Highs->getIntInfoValue(model, "primal_solution_status", &psolstatus));
        const bool has_sol = (psolstatus == kHighsSolutionStatusFeasible);
        switch (status_) {
            case kHighsModelStatusInterrupt:      return interrupted{has_sol};
            case kHighsModelStatusLoadError:
            case kHighsModelStatusModelError:
            case kHighsModelStatusPresolveError:
            case kHighsModelStatusSolveError:
            case kHighsModelStatusPostsolveError: return failed{has_sol};
            case kHighsModelStatusObjectiveBound:
            case kHighsModelStatusObjectiveTarget: return limit_reached{has_sol};
            case kHighsModelStatusTimeLimit:      return time_limit{has_sol};
            case kHighsModelStatusIterationLimit: return iteration_limit{has_sol};
            case kHighsModelStatusModelEmpty:
            case kHighsModelStatusNotset:
            case kHighsModelStatusUnknown:        return unknown{has_sol};
            default:
                return unknown{has_sol};
        }
    }
    // clang-format on
public:
    const status_variant & get_status() const { return _status; }
    ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////////// Solve //////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    void solve() {
        if(num_variables() == 0u) {
            _status = status::unknown{};
            return;
        }
        check(Highs->run(model));
        _status = _get_status();
    }
    double get_solution_value() { return Highs->getObjectiveValue(model); }
    auto get_solution() {
        auto num_vars = num_variables();
        auto solution = std::make_unique_for_overwrite<double[]>(num_vars);
        check(Highs->getSolution(model, solution.get(), nullptr, nullptr,
                                 nullptr));
        return variable_mapping(
            [this, solution = std::move(solution)](const variable & v) {
                return *(solution.get() + _native_id(v));
            });
    }
    auto get_dual_solution() {
        auto num_constrs = num_constraints();
        auto solution = std::make_unique_for_overwrite<double[]>(num_constrs);
        check(Highs->getSolution(model, nullptr, nullptr, nullptr,
                                 solution.get()));
        return constraint_mapping(std::move(solution));
    }
};

}  // namespace highs::impl::v1
}  // namespace mippp
