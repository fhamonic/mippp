#pragma once

#include <algorithm>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <optional>
#include <vector>

#include "mippp/linear_constraint.hpp"

namespace mippp::detail {

// Removes the entries whose coefficient is an exact zero, of either sign.
// COPT answers a callback row holding one with a wrong optimum or a false
// infeasible, GLPK 4.59 to 4.63 abort on it, and `x - x` folds to one: this
// runs after the folding, never before.
template <std::integral Index, std::floating_point Scalar>
void strip_zero_entries(std::vector<Index> & indices,
                        std::vector<Scalar> & coefficients) {
    std::size_t kept = 0;
    for(std::size_t k = 0; k < indices.size(); ++k) {
        if(coefficients[k] == Scalar{0}) continue;
        indices[kept] = indices[k];
        coefficients[kept] = coefficients[k];
        ++kept;
    }
    indices.resize(kept);
    coefficients.resize(kept);
}

// How far the row `coefficients . x <sense> rhs` is from holding at `point`,
// which holds one value per index: positive when the point violates it, and
// for an equality the distance on either side. NaN when the row holds one.
template <std::integral Index, std::floating_point Scalar>
[[nodiscard]] Scalar row_violation(const std::vector<Index> & indices,
                                   const std::vector<Scalar> & coefficients,
                                   const constraint_sense sense,
                                   const Scalar rhs,
                                   const Scalar * const point) noexcept {
    Scalar activity{0};
    for(std::size_t k = 0; k < indices.size(); ++k)
        activity += coefficients[k] * point[indices[k]];
    switch(sense) {
        case constraint_sense::less_equal:
            return activity - rhs;
        case constraint_sense::greater_equal:
            return rhs - activity;
        default:
            return std::abs(activity - rhs);
    }
}

// A user cut is kept only if, once its largest coefficient is 1, the point
// violates it by more than this. Lower, a cut held at the point stalls GLPK
// to its time limit and keeps CPLEX at the root, re-separating forever.
template <std::floating_point Scalar>
inline constexpr Scalar user_cut_violation_tolerance = Scalar(1e-6);

template <std::floating_point Scalar>
struct prepared_cut {
    constraint_sense sense;
    Scalar rhs;
};

// Prepares the user cut `coefficients . x <sense> rhs` for a solver. The
// entries name each index once: fold them first. Nothing comes back when the
// cut is to be dropped; otherwise the entries and the result hold one
// inequality that the point violates.
//
// Each step answers a fault measured on a solver handed a valid cut as
// written, none of them reported by an error code:
// - zeros stripped: see strip_zero_entries;
// - scaled to a largest coefficient of 1: past 1e9 COPT, SCIP and GLPK lose
//   the optimum at status::optimal, and Gurobi ignores the cut;
// - an equality replaced by its violated side: Xpress 46 to 48 crash on a
//   valid equality over columns presolve merged, GLPK's pool aborts;
// - dropped unless violated: see user_cut_violation_tolerance.
// The price: `w.x - K y <= r` with y = 0 at the point is dropped once K
// exceeds a million times its violation. A user cut may be applied nowhere,
// and on GLPK the other choices are a wrong optimum, unscaled, or a stall,
// under a lower tolerance.
template <std::integral Index, std::floating_point Scalar>
[[nodiscard]] std::optional<prepared_cut<Scalar>> prepare_user_cut(
    std::vector<Index> & indices, std::vector<Scalar> & coefficients,
    const constraint_sense sense, Scalar rhs, const Scalar * const point) {
    strip_zero_entries(indices, coefficients);
    if(indices.empty()) return std::nullopt;
    Scalar largest{0};
    for(const Scalar a : coefficients) largest = std::max(largest, std::abs(a));
    if(largest != Scalar{1}) {
        for(Scalar & a : coefficients) a /= largest;
        rhs /= largest;
        // a coefficient 1e308 times smaller than the largest is now a zero
        strip_zero_entries(indices, coefficients);
    }
    // the comparisons below would keep `x >= inf` as a violated cut
    if(!std::isfinite(rhs)) return std::nullopt;
    Scalar activity{0};
    for(std::size_t k = 0; k < indices.size(); ++k)
        activity += coefficients[k] * point[indices[k]];
    const Scalar tolerance = user_cut_violation_tolerance<Scalar>;
    // Not `violation <= tolerance`: a NaN in the cut, or one the scaling
    // made, compares false both ways and the cut must then be dropped.
    if(sense != constraint_sense::greater_equal && activity - rhs > tolerance)
        return prepared_cut<Scalar>{constraint_sense::less_equal, rhs};
    if(sense != constraint_sense::less_equal && rhs - activity > tolerance)
        return prepared_cut<Scalar>{constraint_sense::greater_equal, rhs};
    return std::nullopt;
}

}  // namespace mippp::detail
