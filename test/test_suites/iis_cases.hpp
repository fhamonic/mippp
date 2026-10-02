#pragma once

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <concepts>
#include <cstddef>
#include <optional>
#include <ostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/utility/iis_outcome.hpp"
#include "mippp/utility/variant.hpp"

#include "iis_oracle.hpp"
#include "iis_vectors.hpp"

namespace mippp {

// GoogleTest would otherwise print these scoped enums as raw bytes
inline void PrintTo(iis_outcome outcome, std::ostream * os) {
    switch(outcome) {
        case iis_outcome::irreducible:
            *os << "irreducible";
            return;
        case iis_outcome::not_proven_minimal:
            *os << "not_proven_minimal";
            return;
        case iis_outcome::feasible:
            *os << "feasible";
            return;
        case iis_outcome::undetermined:
            *os << "undetermined";
            return;
    }
    *os << "iis_outcome(" << static_cast<int>(outcome) << ')';
}
inline void PrintTo(iis_reason reason, std::ostream * os) {
    switch(reason) {
        case iis_reason::solve_limit:
            *os << "solve_limit";
            return;
        case iis_reason::time_limit:
            *os << "time_limit";
            return;
        case iis_reason::cancelled:
            *os << "cancelled";
            return;
        case iis_reason::inconclusive_trial:
            *os << "inconclusive_trial";
            return;
    }
    *os << "iis_reason(" << static_cast<int>(reason) << ')';
}

namespace iis_oracle {
inline void PrintTo(membership m, std::ostream * os) {
    switch(m) {
        case membership::absent:
            *os << "absent";
            return;
        case membership::whole:
            *os << "whole";
            return;
        case membership::lower:
            *os << "lower";
            return;
        case membership::upper:
            *os << "upper";
            return;
        case membership::both:
            *os << "both";
            return;
    }
    *os << "membership(" << static_cast<int>(m) << ')';
}
inline void PrintTo(const case_answer & answer, std::ostream * os) {
    *os << "{variables [";
    for(std::size_t i = 0; i < answer.variables.size(); ++i) {
        if(i != 0) *os << ", ";
        PrintTo(answer.variables[i], os);
    }
    *os << "], rows [";
    for(std::size_t i = 0; i < answer.rows.size(); ++i) {
        if(i != 0) *os << ", ";
        PrintTo(answer.rows[i], os);
    }
    *os << "]}";
}
}  // namespace iis_oracle

namespace iis_cases {

using iis_oracle::case_answer;
using iis_oracle::linear_system;
using iis_oracle::membership;

inline constexpr auto none = std::nullopt;

// A case is the oracle's linear system plus what the oracle cannot see: which
// columns are integer, and, for those cases, the answers that are correct
// once integrality is background.
struct iis_case {
    const char * name;
    linear_system system;
    // indices into system.variables; non-empty means the case needs milp_model
    std::vector<std::size_t> integer_columns;
    // exact answers, any one of which is accepted. Empty on an LP case, whose
    // answer the oracle validates instead; an integer case must list them.
    std::vector<case_answer> accepted_answers;
    iis_outcome expected_outcome;
};

///////////////////////////////////////////////////////////////////////////////
////////////////////////////// Building a case ////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// The handles in case order: the oracle's read_answer assumes the i-th handle
// is the case's i-th variable or row.
template <lp_model M>
struct built_case {
    std::vector<model_variable_t<M>> variables;
    std::vector<model_constraint_t<M>> constraints;
};

inline bool is_ranged(const linear_system::row & row) noexcept {
    return row.lower && row.upper && *row.lower != *row.upper;
}

// A model that can widen a one-sided row is as good as one with ranged rows
// here, so the published vectors with a ranged row run on it unchanged.
template <lp_model M>
[[nodiscard]] constexpr bool can_build(const iis_case & c) noexcept {
    if(!c.integer_columns.empty() && !milp_model<M>) return false;
    if constexpr(!has_ranged_constraints<M> &&
                 !has_modifiable_constraint_bounds<M>) {
        if(std::ranges::any_of(c.system.rows, is_ranged)) return false;
    }
    return true;
}

// For a backend that stores a ranged row as one side and a non-negative
// width, whose setters then refuse sides that cross: a case holding such a
// row is skipped from its data, before anything is built.
inline std::optional<std::string> crossed_row_skip_reason(const iis_case & c,
                                                          const char * solver) {
    for(std::size_t i = 0; i < c.system.rows.size(); ++i) {
        const auto & row = c.system.rows[i];
        if(row.lower && row.upper && *row.lower > *row.upper)
            return std::string(solver) + " cannot store row " +
                   std::to_string(i) +
                   ", whose sides cross: a ranged row is one side and a "
                   "width";
    }
    return std::nullopt;
}

// A function object rather than a lambda: xsum stores it, and the suites are
// instantiated where an enclosing using-directive would not reach a lambda.
template <lp_model M>
struct scaled_variable_of {
    const std::vector<model_variable_t<M>> * variables;
    auto operator()(const std::pair<std::size_t, double> & t) const {
        using namespace operators;
        return t.second * (*variables)[t.first];
    }
};

template <lp_model M, linear_expression LE>
model_constraint_t<M> add_row(M & model, LE && lhs,
                              const linear_system::row & row) {
    using namespace operators;
    if(!row.lower && !row.upper)
        throw std::invalid_argument("iis_cases: a row needs a side");
    if(row.lower && row.upper) {
        if(*row.lower == *row.upper)
            return model.add_constraint(std::forward<LE>(lhs) == *row.lower);
        if constexpr(has_ranged_constraints<M>) {
            return model.add_ranged_constraint(std::forward<LE>(lhs),
                                               *row.lower, *row.upper);
        } else if constexpr(has_modifiable_constraint_bounds<M>) {
            auto c = model.add_constraint(std::forward<LE>(lhs) >= *row.lower);
            model.set_constraint_upper_bound(c, *row.upper);
            return c;
        } else {
            throw std::logic_error(
                "iis_cases: a ranged row on a model that cannot build one, "
                "check can_build first");
        }
    }
    if(row.lower)
        return model.add_constraint(std::forward<LE>(lhs) >= *row.lower);
    return model.add_constraint(std::forward<LE>(lhs) <= *row.upper);
}

// On a fresh model: ids then follow the call order on every backend, which
// is what read_answer relies on.
template <lp_model M>
built_case<M> build(M & model, const iis_case & c) {
    using namespace operators;
    using variable = model_variable_t<M>;
    using scalar = model_scalar_t<M>;
    built_case<M> built;
    for(std::size_t i = 0; i < c.system.variables.size(); ++i) {
        const auto & v = c.system.variables[i];
        // an absent optional is no bound here, unlike default_variable_params
        const model_variable_params_t<M> params{.lower_bound = v.lower,
                                                .upper_bound = v.upper};
        const bool integer =
            std::ranges::find(c.integer_columns, i) != c.integer_columns.end();
        if constexpr(milp_model<M>) {
            built.variables.push_back(integer
                                          ? model.add_integer_variable(params)
                                          : model.add_variable(params));
        } else {
            if(integer)
                throw std::logic_error(
                    "iis_cases: an integer column on an lp_model, check "
                    "can_build first");
            built.variables.push_back(model.add_variable(params));
        }
    }
    const scaled_variable_of<M> term_of{&built.variables};
    for(const auto & row : c.system.rows) {
        if(row.terms.empty()) {
            built.constraints.push_back(
                add_row(model, empty_linear_expression<variable, scalar>, row));
        } else {
            built.constraints.push_back(
                add_row(model, xsum(row.terms, term_of), row));
        }
    }
    return built;
}

///////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// Transforms ////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

inline linear_system identity(const linear_system & system) { return system; }

// variable bounds are untouched, so the IIS structure is the same
inline linear_system scaled(const linear_system & system, double factor = 2.) {
    if(!(factor > 0.))
        throw std::invalid_argument(
            "iis_cases: a non-positive factor would swap the sides");
    linear_system scaled_system = system;
    for(auto & row : scaled_system.rows) {
        for(auto & term : row.terms) term.second *= factor;
        if(row.lower) *row.lower *= factor;
        if(row.upper) *row.upper *= factor;
    }
    return scaled_system;
}
inline linear_system scaled_by_two(const linear_system & system) {
    return scaled(system);
}

inline linear_system reversed(const linear_system & system) {
    const std::size_t n = system.variables.size();
    linear_system reversed_system;
    reversed_system.variables.assign(system.variables.rbegin(),
                                     system.variables.rend());
    for(auto it = system.rows.rbegin(); it != system.rows.rend(); ++it) {
        linear_system::row row = *it;
        for(auto & term : row.terms) term.first = n - 1 - term.first;
        reversed_system.rows.push_back(std::move(row));
    }
    return reversed_system;
}

// the substitution x = -x': a lower bound becomes an upper one
inline linear_system sign_flipped(const linear_system & system) {
    linear_system flipped = system;
    for(auto & v : flipped.variables) {
        const std::optional<double> lower = v.lower, upper = v.upper;
        v.lower = upper ? std::optional(-*upper) : none;
        v.upper = lower ? std::optional(-*lower) : none;
    }
    for(auto & row : flipped.rows)
        for(auto & term : row.terms) term.second = -term.second;
    return flipped;
}

using transform_fn = linear_system (*)(const linear_system &);
inline constexpr std::array<std::pair<const char *, transform_fn>, 4>
    transforms{{{"identity", identity},
                {"scaled", scaled_by_two},
                {"reversed", reversed},
                {"sign_flipped", sign_flipped}}};

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////// The cases //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// Suffixed apart from the tests that call them: a test's own name would
// shadow an unqualified call inside its body.
inline iis_case bounds_against_a_row_case() {
    using enum membership;
    return {"bounds_against_a_row",
            {{{0., 1.}, {0., 1.}}, {{{{0, 1.}, {1, 1.}}, 3., none}}},
            {},
            {{{upper, upper}, {lower}}},
            iis_outcome::irreducible};
}
inline iis_case one_side_of_an_equality_row_case() {
    using enum membership;
    return {"one_side_of_an_equality_row",
            {{{0., none}}, {{{{0, 1.}}, -1., -1.}}},
            {},
            {{{lower}, {upper}}, {{lower}, {whole}}},
            iis_outcome::irreducible};
}
inline iis_case integer_equal_to_one_half_case() {
    using enum membership;
    return {"integer_equal_to_one_half",
            {{{none, none}}, {{{{0, 1.}}, 0.5, 0.5}}},
            {0},
            {{{absent}, {both}}, {{absent}, {whole}}},
            iis_outcome::irreducible};
}
// integrality makes the first row redundant: r1 alone has no integer point
inline iis_case integers_summing_to_one_half_case() {
    using enum membership;
    return {"integers_summing_to_one_half",
            {{{none, none}, {none, none}},
             {{{{1, 1.}}, 0., 0.}, {{{0, 1.}, {1, 1.}}, 1.5, 1.5}}},
            {0, 1},
            {{{absent, absent}, {absent, both}},
             {{absent, absent}, {absent, whole}}},
            iis_outcome::irreducible};
}
// the row keeps the model from being row-less; only integrality conflicts
inline iis_case integer_in_a_fractional_interval_case() {
    using enum membership;
    return {"integer_in_a_fractional_interval",
            {{{0.25, 0.75}}, {{{{0, 1.}}, none, 5.}}},
            {0},
            {{{both}, {absent}}},
            iis_outcome::irreducible};
}
// a routine that cannot side a ranged row reports it whole, like an equality
inline iis_case ranged_row_lower_side_case() {
    using enum membership;
    return {"ranged_row_lower_side",
            {{{0., 1.}, {0., 1.}}, {{{{0, 1.}, {1, 1.}}, 3., 4.}}},
            {},
            {{{upper, upper}, {lower}}, {{upper, upper}, {whole}}},
            iis_outcome::irreducible};
}
inline iis_case ranged_row_upper_side_case() {
    using enum membership;
    return {"ranged_row_upper_side",
            {{{2., none}}, {{{{0, 1.}}, -1., 1.}}},
            {},
            {{{lower}, {upper}}, {{lower}, {whole}}},
            iis_outcome::irreducible};
}
// integrality needs both sides: x <= 1.75 alone admits 1 and x >= 1.25 alone
// admits 2, so a routine that names one side claims a feasible subsystem
inline iis_case ranged_row_holding_no_integer_case() {
    using enum membership;
    return {"ranged_row_holding_no_integer",
            {{{-10., 10.}}, {{{{0, 1.}}, 1.25, 1.75}}},
            {0},
            {{{absent}, {both}}, {{absent}, {whole}}},
            iis_outcome::irreducible};
}
// two IISs, {x0 lower, r0 upper} and {x0 lower, r1 upper}: the oracle decides
inline iis_case redundant_rows_case() {
    return {"redundant_rows",
            {{{0., none}},
             {{{{0, 1.}}, none, -1.},
              {{{0, 1.}}, none, -2.},
              {{{0, 1.}}, none, 10.},
              {{{0, 1.}}, -5., none}}},
            {},
            {},
            iis_outcome::irreducible};
}
inline iis_case two_disjoint_conflicts_case() {
    return {
        "two_disjoint_conflicts",
        {{{0., 1.}, {0., 1.}}, {{{{0, 1.}}, 2., none}, {{{1, 1.}}, 2., none}}},
        {},
        {},
        iis_outcome::irreducible};
}
inline iis_case chain_where_every_row_is_needed_case() {
    using enum membership;
    return {"chain_where_every_row_is_needed",
            {{{none, none}, {none, none}, {none, none}, {none, none}},
             {{{{0, 1.}, {1, -1.}}, 1., none},
              {{{1, 1.}, {2, -1.}}, 1., none},
              {{{2, 1.}, {3, -1.}}, 1., none},
              {{{3, 1.}, {0, -1.}}, 1., none}}},
            {},
            {{{absent, absent, absent, absent}, {lower, lower, lower, lower}}},
            iis_outcome::irreducible};
}
// x1 and r0 make the crossed pair a strict subset of the candidates
inline iis_case crossed_variable_bounds_case() {
    using enum membership;
    return {"crossed_variable_bounds",
            {{{1., 0.}, {0., 5.}}, {{{{1, 1.}}, none, 3.}}},
            {},
            {{{both, absent}, {absent}}},
            iis_outcome::irreducible};
}
inline iis_case crossed_row_sides_case() {
    using enum membership;
    return {"crossed_row_sides",
            {{{none, none}, {0., 1.}},
             {{{{0, 1.}}, 1., 0.}, {{{1, 1.}}, none, 3.}}},
            {},
            {{{absent, absent}, {both, absent}}},
            iis_outcome::irreducible};
}
// 0 >= 1 is infeasible on its own, so the upper side of the crossed row is
// not needed: both sides would be reducible
inline iis_case crossed_term_less_row_case() {
    using enum membership;
    return {"crossed_term_less_row",
            {{{0., 1.}}, {{{}, 1., 0.}, {{{0, 1.}}, none, 3.}}},
            {},
            {{{absent}, {lower, absent}}},
            iis_outcome::irreducible};
}
inline iis_case feasible_model_case() {
    return {"feasible_model",
            {{{0., 1.}, {0., 1.}}, {{{{0, 1.}, {1, 1.}}, none, 3.}}},
            {},
            {},
            iis_outcome::feasible};
}
inline iis_case feasible_integer_model_case() {
    using enum membership;
    return {"feasible_integer_model",
            {{{0., 4.}, {0., 4.}}, {{{{0, 1.}, {1, 1.}}, none, 6.5}}},
            {0, 1},
            {{{absent, absent}, {absent}}},
            iis_outcome::feasible};
}
// x + y >= 3 against x <= 1 and y <= 1, beside a row the conflict does not
// need: Gurobi, HiGHS and COPT answer a singleton conflict before they read
// the clock, and this one needs a solve.
inline iis_case four_row_conflict_case() {
    using enum membership;
    return {"four_row_conflict",
            {{{none, none}, {none, none}},
             {{{{0, 1.}, {1, 1.}}, 3., none},
              {{{0, 1.}}, none, 1.},
              {{{1, 1.}}, none, 1.},
              {{{0, 1.}, {1, -1.}}, none, 10.}}},
            {},
            {{{absent, absent}, {lower, upper, upper, absent}}},
            iis_outcome::irreducible};
}
// the same conflict on a MIP, which some routines answer on another path
inline iis_case integer_four_row_conflict_case() {
    iis_case c = four_row_conflict_case();
    c.name = "integer_four_row_conflict";
    c.integer_columns = {0};
    return c;
}

///////////////////////////////////////////////////////////////////////////////
/////////////////////////////// The shared fixture ////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// Plain doubles read through the capabilities the model has; a vector left
// empty means the model cannot read that item.
struct saved_model_data {
    std::vector<std::pair<double, double>> variable_bounds;
    std::vector<double> objective_coefficients;
    double objective_offset = 0.;
    std::vector<std::pair<double, double>> row_bounds;
};

inline std::size_t count_members(const std::vector<membership> & entries) {
    return static_cast<std::size_t>(std::ranges::count_if(
        entries, [](membership m) { return m != membership::absent; }));
}

// The case bodies both suites run, as members: the path (the free function or
// the model's own routine) is a policy with a static compute(model).
template <typename T, typename Path>
struct fixture : public T {
    using T::new_model;
    using typename T::model_type;

    template <typename M>
    saved_model_data save(M & model, const built_case<M> & built) {
        saved_model_data data;
        if constexpr(has_readable_variable_bounds<M>) {
            for(auto v : built.variables)
                data.variable_bounds.emplace_back(
                    model.get_variable_lower_bound(v),
                    model.get_variable_upper_bound(v));
        }
        if constexpr(has_readable_objective<M>) {
            for(auto v : built.variables)
                data.objective_coefficients.push_back(
                    model.get_objective_coefficient(v));
            data.objective_offset = model.get_objective_offset();
        }
        if constexpr(has_readable_constraint_bounds<M>) {
            for(auto c : built.constraints)
                data.row_bounds.emplace_back(
                    model.get_constraint_lower_bound(c),
                    model.get_constraint_upper_bound(c));
        }
        return data;
    }

    // Exact comparison on purpose: an infinite side reads the backend's own
    // infinity(), which a side left at a different infinity would not match.
    template <typename M>
    void expect_unchanged(M & model, const built_case<M> & built,
                          const saved_model_data & before) {
        const saved_model_data after = save(model, built);
        EXPECT_EQ(after.variable_bounds, before.variable_bounds);
        EXPECT_EQ(after.objective_coefficients, before.objective_coefficients);
        EXPECT_EQ(after.objective_offset, before.objective_offset);
        EXPECT_EQ(after.row_bounds, before.row_bounds);
    }

    template <typename Iis, typename M>
    void validate(const iis_case & c, const Iis & iis,
                  const built_case<M> & built) {
        const case_answer answer =
            iis_oracle::read_answer(iis, built.variables, built.constraints);
        ASSERT_EQ(iis.get_outcome(), c.expected_outcome) << answer_text(answer);
        if(c.expected_outcome == iis_outcome::irreducible ||
           c.expected_outcome == iis_outcome::feasible) {
            EXPECT_EQ(iis.get_reason(), std::nullopt);
        }
        // The side of a one-sided entity is known from the model, so it is
        // always named: whole is left to an entity whose two sides a routine
        // does not tell apart.
        for(std::size_t i = 0; i < answer.variables.size(); ++i) {
            const auto & v = c.system.variables[i];
            EXPECT_TRUE(answer.variables[i] != membership::whole ||
                        (v.lower && v.upper))
                << "variable " << i << " is whole with one bound";
        }
        for(std::size_t i = 0; i < answer.rows.size(); ++i) {
            const auto & row = c.system.rows[i];
            EXPECT_TRUE(answer.rows[i] != membership::whole ||
                        (row.lower && row.upper))
                << "row " << i << " is whole with one side";
        }
        if(c.expected_outcome == iis_outcome::feasible) {
            for(const membership m : answer.variables)
                EXPECT_EQ(m, membership::absent);
            for(const membership m : answer.rows)
                EXPECT_EQ(m, membership::absent);
        } else if(c.integer_columns.empty()) {
            EXPECT_TRUE(iis_oracle::is_iis(c.system, answer))
                << answer_text(answer);
        }
        if(!c.accepted_answers.empty()) {
            EXPECT_TRUE(std::ranges::find(c.accepted_answers, answer) !=
                        c.accepted_answers.end())
                << "not an accepted answer: " << answer_text(answer);
        }
        EXPECT_EQ(iis.num_variable_members(), count_members(answer.variables));
        EXPECT_EQ(iis.num_constraint_members(), count_members(answer.rows));
    }

    static std::string answer_text(const case_answer & answer) {
        return ::testing::PrintToString(answer);
    }

    // A backend states, from the case's data alone, an input it documents as
    // unsupported, rather than the error it raises on it. Every builder of a
    // case asks, since the rejection comes from the setters build() calls.
    static std::optional<std::string> skip_reason(const iis_case & c) {
        if constexpr(requires(const iis_case & x) {
                         {
                             T::iis_case_skip_reason(x)
                         } -> std::same_as<std::optional<std::string>>;
                     }) {
            return T::iis_case_skip_reason(c);
        } else {
            return std::nullopt;
        }
    }

    void check_case(const iis_case & c) {
        using M = model_type;
        if(!c.integer_columns.empty() && !milp_model<M>)
            GTEST_SKIP() << "needs milp_model";
        if(!can_build<M>(c)) GTEST_SKIP() << "needs ranged rows";
        if(const auto reason = skip_reason(c)) GTEST_SKIP() << *reason;
        auto model = this->new_model();
        const built_case<M> built = build(model, c);
        const saved_model_data before = save(model, built);
        auto iis = Path::compute(model);
        ASSERT_NO_FATAL_FAILURE(validate(c, iis, built));
        expect_unchanged(model, built, before);
        model.solve();
        if(c.expected_outcome == iis_outcome::irreducible) {
            EXPECT_TRUE(is_a<status::infeasible>(model.get_status()));
        } else if(c.expected_outcome == iis_outcome::feasible) {
            EXPECT_TRUE(is_a<status::optimal>(model.get_status()));
        }
    }

    // A skip ends the whole test, so a vector the model cannot build is only
    // counted, and the test skips when nothing at all could run.
    void check_published_vectors() {
        using M = model_type;
        std::size_t runs = 0;
        for(const auto & v : iis_oracle::published_vectors()) {
            for(const auto & [name, transform] : transforms) {
                SCOPED_TRACE(std::string(v.name) + " / " + name);
                const iis_case c{v.name,
                                 transform(v.system),
                                 {},
                                 {},
                                 iis_outcome::irreducible};
                if(!can_build<M>(c) || skip_reason(c)) continue;
                ++runs;
                auto model = this->new_model();
                const built_case<M> built = build(model, c);
                const saved_model_data before = save(model, built);
                auto iis = Path::compute(model);
                ASSERT_NO_FATAL_FAILURE(validate(c, iis, built));
                expect_unchanged(model, built, before);
            }
        }
        if(runs == 0)
            GTEST_SKIP() << "every published vector needs ranged rows";
    }

    void check_removed_variable_is_skipped() {
        using namespace operators;
        using M = model_type;
        if constexpr(!has_remove_variable<M>) {
            GTEST_SKIP() << "no remove_variable";
        } else {
            using enum membership;
            auto model = this->new_model();
            auto x = model.add_variable({.lower_bound = 0., .upper_bound = 1.});
            auto y = model.add_variable({.lower_bound = 0., .upper_bound = 1.});
            auto z = model.add_variable({.lower_bound = 0., .upper_bound = 1.});
            auto r0 = model.add_constraint(x + z >= 3.);
            model.remove_variable(y);
            auto iis = Path::compute(model);
            const linear_system live{{{0., 1.}, {0., 1.}},
                                     {{{{0, 1.}, {1, 1.}}, 3., none}}};
            const case_answer answer = iis_oracle::read_answer(
                iis, std::vector{x, z}, std::vector{r0});
            EXPECT_EQ(iis.get_outcome(), iis_outcome::irreducible);
            EXPECT_TRUE(iis_oracle::is_iis(live, answer));
            EXPECT_EQ(answer, (case_answer{{upper, upper}, {lower}}));
            EXPECT_TRUE(is<iis_status::absent>(iis.get_status(y)));
            EXPECT_EQ(iis.num_variable_members(), 2u);
            EXPECT_EQ(iis.num_constraint_members(), 1u);
        }
    }

    // keyed by id and never re-read from the model: the answer describes the
    // model as it was, and a handle that reuses the id reads the old entry
    void check_answer_survives_a_later_removal() {
        using namespace operators;
        using M = model_type;
        if constexpr(!has_remove_variable<M>) {
            GTEST_SKIP() << "no remove_variable";
        } else {
            using enum membership;
            auto model = this->new_model();
            auto x = model.add_variable({.lower_bound = 0., .upper_bound = 1.});
            auto y = model.add_variable({.lower_bound = 0., .upper_bound = 1.});
            auto r0 = model.add_constraint(x + y >= 3.);
            auto iis = Path::compute(model);
            ASSERT_EQ(iis.get_outcome(), iis_outcome::irreducible);
            model.remove_variable(x);
            EXPECT_EQ(iis_oracle::membership_of(iis.get_status(x)), upper);
            EXPECT_EQ(iis_oracle::membership_of(iis.get_status(y)), upper);
            EXPECT_EQ(iis_oracle::membership_of(iis.get_status(r0)), lower);
            EXPECT_EQ(iis.num_variable_members(), 2u);
            auto w = model.add_variable();
            ASSERT_EQ(w.uid(), x.uid());
            EXPECT_EQ(iis_oracle::membership_of(iis.get_status(w)), upper);
        }
    }

    // the model as it is, never the stale status of the earlier solve
    void check_model_modified_after_an_infeasible_solve() {
        using namespace operators;
        using M = model_type;
        if constexpr(!has_modifiable_variable_bounds<M>) {
            GTEST_SKIP() << "no modifiable variable bounds";
        } else {
            using enum membership;
            auto model = this->new_model();
            auto x = model.add_variable({.lower_bound = 0., .upper_bound = 1.});
            auto r0 = model.add_constraint(x >= 2.);
            model.solve();
            ASSERT_TRUE(is_a<status::infeasible>(model.get_status()));
            model.set_variable_upper_bound(x, 3.);
            auto iis = Path::compute(model);
            EXPECT_EQ(iis.get_outcome(), iis_outcome::feasible);
            EXPECT_EQ(iis.num_variable_members(), 0u);
            EXPECT_EQ(iis.num_constraint_members(), 0u);
            model.set_variable_upper_bound(x, 1.);
            auto iis2 = Path::compute(model);
            EXPECT_EQ(iis2.get_outcome(), iis_outcome::irreducible);
            const linear_system system{{{0., 1.}}, {{{{0, 1.}}, 2., none}}};
            const case_answer answer =
                iis_oracle::read_answer(iis2, std::vector{x}, std::vector{r0});
            EXPECT_TRUE(iis_oracle::is_iis(system, answer));
            EXPECT_EQ(answer, (case_answer{{upper}, {lower}}));
        }
    }

    // The sense is not readable through a concept: the re-solve to the same
    // maximum is what shows it was not touched.
    void check_model_data_and_result_survive_the_call() {
        using namespace operators;
        using M = model_type;
        auto model = this->new_model();
        auto x = model.add_variable({.lower_bound = 0., .upper_bound = 4.});
        auto y = model.add_variable({.lower_bound = 0., .upper_bound = 4.});
        model.set_maximization();
        model.set_objective(x + 2 * y + 1);
        auto r0 = model.add_constraint(x + y <= 6.);
        auto r1 = model.add_constraint(x - y >= -2.);
        model.solve();
        ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
        ASSERT_NEAR(model.get_solution_value(), 11., TEST_EPSILON);
        built_case<M> built{{x, y}, {r0, r1}};
        saved_model_data before = save(model, built);
        auto iis = Path::compute(model);
        EXPECT_EQ(iis.get_outcome(), iis_outcome::feasible);
        expect_unchanged(model, built, before);
        model.solve();
        ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
        EXPECT_NEAR(model.get_solution_value(), 11., TEST_EPSILON);
        {
            const auto solution = model.get_solution();
            EXPECT_NEAR(solution[x], 2., TEST_EPSILON);
            EXPECT_NEAR(solution[y], 4., TEST_EPSILON);
        }
        auto r2 = model.add_constraint(x + y >= 20.);
        built.constraints.push_back(r2);
        model.solve();
        ASSERT_TRUE(is_a<status::infeasible>(model.get_status()));
        before = save(model, built);
        auto iis2 = Path::compute(model);
        EXPECT_EQ(iis2.get_outcome(), iis_outcome::irreducible);
        const linear_system system{{{0., 4.}, {0., 4.}},
                                   {{{{0, 1.}, {1, 1.}}, none, 6.},
                                    {{{0, 1.}, {1, -1.}}, -2., none},
                                    {{{0, 1.}, {1, 1.}}, 20., none}}};
        const case_answer answer =
            iis_oracle::read_answer(iis2, built.variables, built.constraints);
        EXPECT_TRUE(iis_oracle::is_iis(system, answer)) << answer_text(answer);
        expect_unchanged(model, built, before);
        model.solve();
        EXPECT_TRUE(is_a<status::infeasible>(model.get_status()));
    }

    // no re-solve: solvers report unknown on a model without columns
    void check_column_less_model() {
        using namespace operators;
        using M = model_type;
        auto model = this->new_model();
        constexpr auto & no_terms =
            empty_linear_expression<model_variable_t<M>, model_scalar_t<M>>;
        auto r0 = model.add_constraint(no_terms >= 1.);
        auto r1 = model.add_constraint(no_terms <= 2.);
        auto iis = Path::compute(model);
        EXPECT_EQ(iis.get_outcome(), iis_outcome::irreducible);
        EXPECT_EQ(iis.get_reason(), std::nullopt);
        EXPECT_EQ(iis_oracle::membership_of(iis.get_status(r0)),
                  membership::lower);
        EXPECT_TRUE(is<iis_status::absent>(iis.get_status(r1)));
        EXPECT_EQ(iis.num_constraint_members(), 1u);
        EXPECT_EQ(iis.num_variable_members(), 0u);
    }

    // The status of an earlier solve no longer describes the solver, which
    // may now hold the routine's or a trial's own solution, while the data
    // are those of the model: a re-solve finds the same status again.
    void check_status_is_unknown_after_the_call() {
        {
            auto model = this->new_model();
            build(model, bounds_against_a_row_case());
            model.solve();
            ASSERT_TRUE(is_a<status::infeasible>(model.get_status()));
            const auto iis = Path::compute(model);
            EXPECT_EQ(iis.get_outcome(), iis_outcome::irreducible);
            EXPECT_TRUE(is<status::unknown>(model.get_status()));
            model.solve();
            EXPECT_TRUE(is_a<status::infeasible>(model.get_status()));
        }
        {
            auto model = this->new_model();
            build(model, feasible_model_case());
            model.solve();
            ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
            const auto iis = Path::compute(model);
            EXPECT_EQ(iis.get_outcome(), iis_outcome::feasible);
            EXPECT_TRUE(is<status::unknown>(model.get_status()));
            model.solve();
            EXPECT_TRUE(is_a<status::optimal>(model.get_status()));
        }
    }

    // Some wrappers write the remaining budget into the model's limit for
    // part of the call, such as the routine after the solve that a MIP path
    // runs first, or the solve that confirms a feasible model: the cases
    // reach each of those paths.
    void check_time_limit_reads_back_unchanged() {
        using M = model_type;
        if constexpr(!has_time_limit<M>) {
            GTEST_SKIP() << "no time limit";
        } else {
            std::vector<iis_case> cases{bounds_against_a_row_case(),
                                        feasible_model_case()};
            if constexpr(milp_model<M>) {
                cases.push_back(integer_equal_to_one_half_case());
                cases.push_back(feasible_integer_model_case());
            }
            for(const iis_case & c : cases) {
                SCOPED_TRACE(c.name);
                if(!can_build<M>(c) || skip_reason(c)) continue;
                auto model = this->new_model();
                build(model, c);
                model.set_time_limit(std::chrono::seconds(3));
                const auto before = model.get_time_limit();
                const auto iis = Path::compute(model);
                EXPECT_EQ(iis.get_outcome(), c.expected_outcome);
                EXPECT_EQ(model.get_time_limit(), before);
                EXPECT_EQ(model.get_time_limit(), std::chrono::seconds(3));
            }
        }
    }

    // An indicator has no handle to report, so it stays in every subsystem:
    // the members are an IIS relative to the indicators, and a conflict among
    // indicators alone is irreducible with no member.
    void check_indicator_constraints_are_background() {
        using namespace operators;
        using M = model_type;
        if constexpr(!has_indicator_constraints<M>) {
            GTEST_SKIP() << "no indicator constraints";
        } else {
            using enum iis_oracle::membership;
            {
                SCOPED_TRACE("a needed indicator");
                // z is held at 1 by a row, and the indicator then asks
                // x >= 5 of x in [0, 1]: without it the rest is feasible
                auto model = this->new_model();
                auto z = model.add_binary_variable();
                auto x =
                    model.add_variable({.lower_bound = 0., .upper_bound = 1.});
                auto r0 = model.add_constraint(z >= 1.);
                model.add_indicator_constraint(z, true, x >= 5.);
                const auto iis = Path::compute(model);
                expect_accepted_answer(iis, iis_outcome::irreducible,
                                       std::vector{z, x}, std::vector{r0},
                                       {{{absent, upper}, {lower}}});
            }
            {
                SCOPED_TRACE("an indicator the conflict does not need");
                auto model = this->new_model();
                auto z = model.add_binary_variable();
                auto x =
                    model.add_variable({.lower_bound = 0., .upper_bound = 1.});
                auto y =
                    model.add_variable({.lower_bound = 0., .upper_bound = 10.});
                auto r0 = model.add_constraint(x >= 2.);
                model.add_indicator_constraint(z, true, x + y <= 5.);
                const auto iis = Path::compute(model);
                expect_accepted_answer(iis, iis_outcome::irreducible,
                                       std::vector{z, x, y}, std::vector{r0},
                                       {{{absent, upper, absent}, {lower}}});
            }
            {
                SCOPED_TRACE("indicators alone");
                // whatever z is, one indicator asks x >= 5, another x <= 1
                auto model = this->new_model();
                auto z = model.add_binary_variable();
                auto x = model.add_variable();
                auto y =
                    model.add_variable({.lower_bound = 0., .upper_bound = 1.});
                model.add_indicator_constraint(z, true, x >= 5.);
                model.add_indicator_constraint(z, false, x >= 5.);
                model.add_indicator_constraint(z, true, x <= 1.);
                model.add_indicator_constraint(z, false, x <= 1.);
                auto r0 = model.add_constraint(y <= 3.);
                const auto iis = Path::compute(model);
                expect_accepted_answer(iis, iis_outcome::irreducible,
                                       std::vector{z, x, y}, std::vector{r0},
                                       {{{absent, absent, absent}, {absent}}});
                model.solve();
                EXPECT_TRUE(is_a<status::infeasible>(model.get_status()));
            }
        }
    }

private:
    template <typename Iis, typename Variables, typename Constraints>
    void expect_accepted_answer(const Iis & iis, iis_outcome outcome,
                                const Variables & variables,
                                const Constraints & constraints,
                                const std::vector<case_answer> & accepted) {
        ASSERT_EQ(iis.get_outcome(), outcome);
        EXPECT_EQ(iis.get_reason(), std::nullopt);
        const case_answer answer =
            iis_oracle::read_answer(iis, variables, constraints);
        EXPECT_TRUE(std::ranges::find(accepted, answer) != accepted.end())
            << "not an accepted answer: " << answer_text(answer);
        EXPECT_EQ(iis.num_variable_members(), count_members(answer.variables));
        EXPECT_EQ(iis.num_constraint_members(), count_members(answer.rows));
    }
};

}  // namespace iis_cases
}  // namespace mippp
