#pragma once
#include <cmath>
#include <stdexcept>
#include "mippp/utility/linear_iis_types.hpp"
#include "mippp/utility/linear_iis_model.hpp"

namespace mippp::iis::detail {

// Validate input once and assign stable IDs to its bound and constraint sides.
// The input is borrowed: callers must keep it unchanged until extraction ends.
// Preparing this representation does not load a solver.
template <std::floating_point Scalar>
struct prepared_linear_system {
    const linear_system<Scalar> & system;
    const domain mode;
    bool has_integers = false;
    std::vector<member> candidates;

    prepared_linear_system(const linear_system<Scalar> & input, domain analyzed,
                           bool supports_integer)
        : system(input), mode(analyzed) {
        // Validate and assign stable candidate IDs once, before any solver calls.
        // Each finite side is independent: for example, only the lower side of
        // 2 <= x <= 5 may conflict with a separate bound x <= 1.
        std::size_t candidate_count = 0;
        for(const auto & v : system.variables) {
            candidate_count += v.lower.has_value();
            candidate_count += v.upper.has_value();
        }
        for(const auto & row : system.rows) {
            candidate_count += row.lower.has_value();
            candidate_count += row.upper.has_value();
        }
        candidates.reserve(candidate_count);
        auto add_bound = [&](std::optional<Scalar> bound, member_kind kind,
                             std::size_t index) {
            if(!bound) return;
            if(!std::isfinite(*bound))
                throw std::invalid_argument("Invalid bound at " +
                    std::string(kind == member_kind::row_lower || kind == member_kind::row_upper
                        ? "row " : "variable ") + std::to_string(index) + " (zero-based index). " +
                    setting_help(kind == member_kind::row_lower || kind == member_kind::variable_lower
                        ? "lower" : "upper", setting_value(*bound),
                        "a finite number, or std::nullopt for no bound"));
            candidates.push_back({kind, index});
        };
        for(std::size_t i = 0; i < system.variables.size(); ++i) {
            const auto & v = system.variables[i];
            has_integers = has_integers || v.integer;
            if(v.integer && mode == domain::original && !supports_integer)
                throw std::invalid_argument("Variable " + std::to_string(i) +
                    " requires whole-number values, but the selected solver model does not support them. "
                    "Choose an integer-capable model to explain the original problem. " +
                    setting_help("policy.analyzed_domain", "original", "original, lp_relaxation") +
                    " lp_relaxation allows fractional values and cannot explain conflicts caused only by integer requirements.");
            add_bound(v.lower, member_kind::variable_lower, i);
            add_bound(v.upper, member_kind::variable_upper, i);
        }
        for(std::size_t i = 0; i < system.rows.size(); ++i) {
            const auto & row = system.rows[i];
            for(const auto & [column, coefficient] : row.terms) {
                if(column >= system.variables.size())
                    throw std::invalid_argument("Row " + std::to_string(i) + " refers to variable " +
                        std::to_string(column) + " (zero-based indices), but the problem has " +
                        std::to_string(system.variables.size()) + " variables. " +
                        (system.variables.empty() ? "No variable references are valid; add the variable first."
                        : "Use indices 0 through " + std::to_string(system.variables.size() - 1) + "."));
                if(!std::isfinite(coefficient))
                    throw std::invalid_argument("Row " + std::to_string(i) + ", variable " +
                        std::to_string(column) + " (zero-based indices): " +
                        setting_help("coefficient", setting_value(coefficient), "any finite number"));
            }
            add_bound(row.lower, member_kind::row_lower, i);
            add_bound(row.upper, member_kind::row_upper, i);
        }
    }

    auto inequalities() const {
        std::vector<detail::linear_inequality<Scalar>> inequalities;
        inequalities.reserve(candidates.size());
        for(const auto & [kind, index] : candidates) {
            const bool lower = kind == member_kind::row_lower ||
                               kind == member_kind::variable_lower;
            if(kind == member_kind::row_lower || kind == member_kind::row_upper) {
                const auto & row = system.rows[index];
                inequalities.push_back({row.terms, lower ? *row.lower : *row.upper, lower});
            } else {
                const auto & var = system.variables[index];
                inequalities.push_back({{{index, Scalar{1}}},
                                       lower ? *var.lower : *var.upper, lower});
            }
        }
        return inequalities;
    }
};
} // namespace mippp::iis::detail
