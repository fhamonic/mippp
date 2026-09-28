#pragma once

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <optional>
#include <ranges>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "mippp/model_concepts.hpp"

namespace mippp::iis_oracle {

// A side is present when its std::optional holds a value, never an infinity.
struct linear_system {
    struct variable {
        std::optional<double> lower;
        std::optional<double> upper;
    };
    struct row {
        std::vector<std::pair<std::size_t, double>> terms;
        std::optional<double> lower;
        std::optional<double> upper;
    };
    std::vector<variable> variables;
    std::vector<row> rows;
};

// Each eliminated variable can square the number of inequalities.
inline constexpr std::size_t max_variables = 6;

enum class side_kind { variable_lower, variable_upper, row_lower, row_upper };

struct side {
    side_kind kind;
    std::size_t index;
    friend bool operator==(const side &, const side &) = default;
};

inline std::vector<side> all_sides(const linear_system & system) {
    std::vector<side> sides;
    for(std::size_t i = 0; i < system.variables.size(); ++i) {
        if(system.variables[i].lower)
            sides.push_back({side_kind::variable_lower, i});
        if(system.variables[i].upper)
            sides.push_back({side_kind::variable_upper, i});
    }
    for(std::size_t i = 0; i < system.rows.size(); ++i) {
        if(system.rows[i].lower) sides.push_back({side_kind::row_lower, i});
        if(system.rows[i].upper) sides.push_back({side_kind::row_upper, i});
    }
    return sides;
}

inline double finite_data(double value) {
    if(!std::isfinite(value))
        throw std::invalid_argument(
            "iis_oracle: data must be finite, an absent side is std::nullopt");
    return value;
}
inline double exact_product(double a, double b) {
    const double product = a * b;
    if(!std::isfinite(product) || std::fma(a, b, -product) != 0.)
        throw std::domain_error(
            "iis_oracle: a product rounds, keep the data small dyadic numbers");
    return product;
}
// Knuth's TwoSum: its error term is exact, unlike (a + b) - a - b.
inline double exact_sum(double a, double b) {
    const double sum = a + b;
    const double b_part = sum - a;
    if(!std::isfinite(sum) || (a - (sum - b_part)) + (b - b_part) != 0.)
        throw std::domain_error(
            "iis_oracle: a sum rounds, keep the data small dyadic numbers");
    return sum;
}

// Fourier-Motzkin elimination over the given sides alone. It ignores
// integrality, so an integer case needs its own witnesses, and it throws
// rather than decide on a rounded value.
inline bool is_feasible(const linear_system & system,
                        std::span<const side> sides) {
    const std::size_t n = system.variables.size();
    if(n > max_variables)
        throw std::length_error(
            "iis_oracle: more variables than max_variables");
    // a x <= b, stored as {a, b}
    std::vector<std::vector<double>> rows;
    for(const auto [kind, index] : sides) {
        std::vector<double> row(n + 1, 0.);
        const bool lower =
            kind == side_kind::row_lower || kind == side_kind::variable_lower;
        const double sign = lower ? -1. : 1.;
        if(kind == side_kind::row_lower || kind == side_kind::row_upper) {
            const auto & source = system.rows.at(index);
            for(const auto & [column, value] : source.terms) {
                // row[n] holds the right-hand side, so at() would not catch n
                if(column >= n)
                    throw std::out_of_range(
                        "iis_oracle: a term names a variable the case does "
                        "not have");
                row[column] = exact_sum(row[column], sign * finite_data(value));
            }
            row[n] = sign * finite_data(lower ? source.lower.value()
                                              : source.upper.value());
        } else {
            const auto & source = system.variables.at(index);
            row.at(index) = sign;
            row[n] = sign * finite_data(lower ? source.lower.value()
                                              : source.upper.value());
        }
        rows.push_back(std::move(row));
    }
    for(std::size_t col = 0; col < n; ++col) {
        std::vector<std::vector<double>> next;
        for(const auto & row : rows)
            if(row[col] == 0.) next.push_back(row);
        for(const auto & positive : rows) {
            if(positive[col] <= 0.) continue;
            for(const auto & negative : rows) {
                if(negative[col] >= 0.) continue;
                std::vector<double> row(n + 1, 0.);
                for(std::size_t j = col + 1; j <= n; ++j)
                    row[j] =
                        exact_sum(exact_product(positive[j], -negative[col]),
                                  exact_product(negative[j], positive[col]));
                next.push_back(std::move(row));
            }
        }
        rows = std::move(next);
    }
    return std::ranges::all_of(rows,
                               [n](const auto & row) { return row[n] >= 0.; });
}

enum class membership { absent, whole, lower, upper, both };

template <typename Status>
    requires lp_iis_status<Status>
membership membership_of(const Status & status) {
    return std::visit(
        []<typename Tag>(const Tag &) {
            if constexpr(std::derived_from<Tag, iis_status::member_both>)
                return membership::both;
            else if constexpr(std::derived_from<Tag, iis_status::member_lower>)
                return membership::lower;
            else if constexpr(std::derived_from<Tag, iis_status::member_upper>)
                return membership::upper;
            else if constexpr(std::derived_from<Tag, iis_status::member>)
                return membership::whole;
            else {
                static_assert(std::same_as<Tag, iis_status::absent>);
                return membership::absent;
            }
        },
        status);
}

struct case_answer {
    std::vector<membership> variables;
    std::vector<membership> rows;
    friend bool operator==(const case_answer &, const case_answer &) = default;
};

// The i-th handle must be the case's i-th variable or row, whatever order the
// model was built in.
template <typename Iis, std::ranges::input_range Variables,
          std::ranges::input_range Constraints>
case_answer read_answer(const Iis & iis, Variables && variables,
                        Constraints && constraints) {
    case_answer answer;
    for(auto && v : variables)
        answer.variables.push_back(membership_of(iis.get_status(v)));
    for(auto && c : constraints)
        answer.rows.push_back(membership_of(iis.get_status(c)));
    return answer;
}

// The reported sides must be infeasible, and feasible once any one unit is
// dropped. member_both makes each side a unit, while a bare member keeps all
// the sides of its variable or row in one unit: an answer holding a crossed
// row whole and any other member is reducible.
inline ::testing::AssertionResult is_iis(const linear_system & system,
                                         const case_answer & answer) {
    if(answer.variables.size() != system.variables.size() ||
       answer.rows.size() != system.rows.size())
        return ::testing::AssertionFailure()
               << "the answer covers " << answer.variables.size()
               << " variables and " << answer.rows.size()
               << " rows, the case has " << system.variables.size() << " and "
               << system.rows.size();
    struct unit {
        std::string name;
        std::vector<side> sides;
    };
    std::vector<unit> units;
    std::string missing;
    const auto add_units = [&](std::string name, membership reported,
                               bool has_lower, bool has_upper, side lower_side,
                               side upper_side) {
        const bool needs_lower =
            reported == membership::lower || reported == membership::both;
        const bool needs_upper =
            reported == membership::upper || reported == membership::both;
        if((needs_lower && !has_lower) || (needs_upper && !has_upper) ||
           (reported == membership::whole && !has_lower && !has_upper)) {
            missing += " " + name;
            return;
        }
        if(reported == membership::whole) {
            unit whole{name + " whole", {}};
            if(has_lower) whole.sides.push_back(lower_side);
            if(has_upper) whole.sides.push_back(upper_side);
            units.push_back(std::move(whole));
            return;
        }
        if(needs_lower) units.push_back({name + " lower", {lower_side}});
        if(needs_upper) units.push_back({name + " upper", {upper_side}});
    };
    for(std::size_t i = 0; i < system.variables.size(); ++i)
        add_units("variable " + std::to_string(i), answer.variables[i],
                  system.variables[i].lower.has_value(),
                  system.variables[i].upper.has_value(),
                  {side_kind::variable_lower, i},
                  {side_kind::variable_upper, i});
    for(std::size_t i = 0; i < system.rows.size(); ++i)
        add_units("row " + std::to_string(i), answer.rows[i],
                  system.rows[i].lower.has_value(),
                  system.rows[i].upper.has_value(), {side_kind::row_lower, i},
                  {side_kind::row_upper, i});
    if(!missing.empty())
        return ::testing::AssertionFailure()
               << "reported sides the case does not have:" << missing;

    std::vector<side> kept;
    for(const auto & u : units)
        kept.insert(kept.end(), u.sides.begin(), u.sides.end());
    if(is_feasible(system, kept))
        return ::testing::AssertionFailure()
               << "the reported sides are feasible";
    for(const auto & u : units) {
        std::vector<side> rest;
        for(const side s : kept)
            if(std::ranges::find(u.sides, s) == u.sides.end())
                rest.push_back(s);
        if(!is_feasible(system, rest))
            return ::testing::AssertionFailure()
                   << "still infeasible without " << u.name;
    }
    return ::testing::AssertionSuccess();
}

}  // namespace mippp::iis_oracle
