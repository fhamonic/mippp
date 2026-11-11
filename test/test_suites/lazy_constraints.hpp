#pragma once

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <limits>
#include <numeric>
#include <ranges>
#include <utility>
#include <vector>

#include "mippp/linear_constraint.hpp"
#include "mippp/model_concepts.hpp"

namespace mippp {

template <typename T>
struct LazyConstraintsTest : public T {
    using typename T::model_type;
    static_assert(milp_model<model_type>);
    static_assert(has_candidate_solution_callback<model_type>);
    static_assert(
        has_lazy_constraints<candidate_solution_callback_handle_t<model_type>,
                             model_type>);
};
TYPED_TEST_SUITE_P(LazyConstraintsTest);
GTEST_ALLOW_UNINSTANTIATED_PARAMETERIZED_TEST(LazyConstraintsTest);

namespace detail {

// An asymmetric travelling salesman over eight cities, as an assignment
// model whose subtours a callback eliminates. Not any instance will do for
// the tests that use it: on distances drawn from a formula, COPT 8.0 returned
// the right tour where on these it answers infeasible.
struct eight_cities {
    static constexpr std::size_t n = 8;
    // clang-format off
    static constexpr std::array<std::array<int, n>, n> distances = {{
        {  0,  99, 105,  85,  13,  97, 111,  48},
        {101,   0,  77,  52, 103,  55,  85,  50},
        {110,  74,   0,  35,  88,  16,  28,  74},
        { 75,  42,  37,   0,  69,  18,  50,  30},
        { 11,  94,  86,  73,   0,  73,  91,  42},
        {102,  65,  16,  19,  82,   0,  38,  58},
        { 96,  89,  31,  57,  91,  37,   0,  85},
        { 47,  43,  74,  29,  51,  52,  78,   0}}};
    // clang-format on
    static constexpr auto cities() {
        return std::views::iota(std::size_t{0}, n);
    }
    // arc a goes from city a / n to city a % n
    static constexpr auto arcs() {
        return std::views::iota(std::size_t{0}, n * n);
    }
    static constexpr double distance(const std::size_t a) {
        return static_cast<double>(distances[a / n][a % n]);
    }
    static double shortest_tour() {
        static const double shortest = [] {
            std::array<std::size_t, n> order;
            std::iota(order.begin(), order.end(), std::size_t{0});
            double best = std::numeric_limits<double>::infinity();
            do {
                double length = distance(order[n - 1] * n + order[0]);
                for(std::size_t k = 0; k + 1 < n; ++k)
                    length += distance(order[k] * n + order[k + 1]);
                best = std::min(best, length);
            } while(std::next_permutation(order.begin() + 1, order.end()));
            return best;
        }();
        return shortest;
    }
    // one arc out of each city and one into it, none from a city to itself
    template <typename Model>
    static auto build(Model & model) {
        using namespace operators;
        auto x = model.add_binary_variables(n * n);
        model.set_minimization();
        model.set_objective(xsum(
            arcs(), [&](const std::size_t a) { return distance(a) * x(a); }));
        for(const std::size_t i : cities()) {
            model.add_constraint(x(i * n + i) <= 0);
            model.add_constraint(xsum(cities(), [&, i](const std::size_t j) {
                                     return x(i * n + j);
                                 }) == 1);
            model.add_constraint(xsum(cities(), [&, i](const std::size_t j) {
                                     return x(j * n + i);
                                 }) == 1);
        }
        return x;
    }
    // The sets of cities that the arcs a point uses leave apart, none when
    // they hold together. value(a) is the value of arc a: at an integer point
    // the sets are its subtours, and at any point no arc leaves a set, so the
    // arcs inside one sum to its size.
    template <typename Value>
    static std::vector<std::vector<std::size_t>> pieces(Value && value) {
        std::array<std::size_t, n> piece_of;
        std::iota(piece_of.begin(), piece_of.end(), std::size_t{0});
        for(bool merged = true; merged;) {
            merged = false;
            for(const std::size_t a : arcs()) {
                const std::size_t i = a / n, j = a % n;
                if(i == j || piece_of[i] == piece_of[j] || !(value(a) > 1e-6))
                    continue;
                const std::size_t from = std::max(piece_of[i], piece_of[j]);
                const std::size_t to = std::min(piece_of[i], piece_of[j]);
                for(auto & p : piece_of)
                    if(p == from) p = to;
                merged = true;
            }
        }
        std::vector<std::vector<std::size_t>> pieces;
        for(const std::size_t p : cities()) {
            std::vector<std::size_t> piece;
            for(const std::size_t i : cities())
                if(piece_of[i] == p) piece.push_back(i);
            if(!piece.empty() && piece.size() < n)
                pieces.push_back(std::move(piece));
        }
        return pieces;
    }
    static std::vector<std::size_t> arcs_inside(
        const std::vector<std::size_t> & piece) {
        std::vector<std::size_t> inside;
        for(const std::size_t i : piece)
            for(const std::size_t j : piece)
                if(i != j) inside.push_back(i * n + j);
        return inside;
    }
};

template <typename Test, typename Separate>
void lazy_constraints_cut_off_candidates(Test & test, Separate && separate) {
    using namespace operators;
    auto model = test.new_model();
    auto a = model.add_binary_variable();
    auto b = model.add_binary_variable();
    auto c = model.add_binary_variable();
    model.set_maximization();
    model.set_objective(3 * a + 2 * b + c);

    int num_cuts = 0;
    model.set_candidate_solution_callback([&](auto & handle) {
        auto solution = handle.get_solution();
        if(solution[a] + solution[b] + solution[c] <= 2 + TEST_EPSILON) return;
        ++num_cuts;
        separate(handle, a + b + c <= 2);
    });
    model.solve();

    ASSERT_GE(num_cuts, 1);
    ASSERT_NEAR(model.get_solution_value(), 5, TEST_EPSILON);
    auto solution = model.get_solution();
    ASSERT_NEAR(solution[a], 1, TEST_EPSILON);
    ASSERT_NEAR(solution[b], 1, TEST_EPSILON);
    ASSERT_NEAR(solution[c], 0, TEST_EPSILON);
}
}  // namespace detail

TYPED_TEST_P(LazyConstraintsTest, cuts_off_candidates) {
    this->SkipOnLicenseError([this]() {
        detail::lazy_constraints_cut_off_candidates(
            *this,
            [](auto & handle, auto && lc) { handle.add_lazy_constraint(lc); });
    });
}

TYPED_TEST_P(LazyConstraintsTest, cuts_off_candidates_distinct_variables) {
    this->SkipOnLicenseError([this]() {
        detail::lazy_constraints_cut_off_candidates(
            *this, [](auto & handle, auto && lc) {
                handle.add_lazy_constraint(distinct_variables, lc);
            });
    });
}

// A separator may offer every constraint of its family, violated or not.
// One the candidate satisfies must not cost that candidate: natively CPLEX
// and COPT reject it whatever the row says, and this model ends infeasible.
TYPED_TEST_P(LazyConstraintsTest, a_satisfied_constraint_rejects_nothing) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x =
            model.add_integer_variable({.lower_bound = 0, .upper_bound = 5});
        auto y =
            model.add_integer_variable({.lower_bound = 0, .upper_bound = 5});
        model.set_maximization();
        model.set_objective(x + y);
        model.add_constraint(x + y <= 7);

        int num_candidates = 0;
        model.set_candidate_solution_callback([&](auto & handle) {
            ++num_candidates;
            handle.add_lazy_constraint(x + y <= 100);
            handle.add_lazy_constraint(distinct_variables, x + y >= -100);
            // held at the optimum, to the last bit
            handle.add_lazy_constraint(x + y <= 7);
        });
        model.solve();

        ASSERT_GE(num_candidates, 1);
        ASSERT_TRUE(is<status::optimal>(model.get_status()));
        ASSERT_NEAR(model.get_solution_value(), 7, TEST_EPSILON);
    });
}

TYPED_TEST_P(LazyConstraintsTest, a_violated_constraint_cuts_among_satisfied) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto a = model.add_binary_variable();
        auto b = model.add_binary_variable();
        auto c = model.add_binary_variable();
        model.set_maximization();
        model.set_objective(3 * a + 2 * b + c);

        int num_cuts = 0;
        model.set_candidate_solution_callback([&](auto & handle) {
            auto point = handle.get_solution();
            handle.add_lazy_constraint(a + b + c <= 100);
            if(point[a] + point[b] + point[c] > 2 + TEST_EPSILON) {
                ++num_cuts;
                handle.add_lazy_constraint(a + b + c <= 2);
            }
            handle.add_lazy_constraint(a + b + c >= -100);
        });
        model.solve();

        ASSERT_GE(num_cuts, 1);
        ASSERT_TRUE(is<status::optimal>(model.get_status()));
        ASSERT_NEAR(model.get_solution_value(), 5, TEST_EPSILON);
    });
}

// Subtour elimination written as one row over every arc, with a coefficient
// that is 1 inside the subtour and 0 outside: what a separator writes when
// its coefficients are data. COPT answers a callback row holding an exact
// zero with a false infeasible, or a wrong optimum.
TYPED_TEST_P(LazyConstraintsTest, zero_coefficients_change_nothing) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        using cities = detail::eight_cities;
        for(const bool distinct : {false, true}) {
            auto model = this->new_model();
            auto x = cities::build(model);
            int num_cuts = 0;
            model.set_candidate_solution_callback([&](auto & handle) {
                auto point = handle.get_solution();
                for(const auto & piece : cities::pieces(
                        [&](const std::size_t a) { return point[x(a)]; })) {
                    std::array<double, cities::n * cities::n> inside{};
                    for(const std::size_t a : cities::arcs_inside(piece))
                        inside[a] = 1;
                    ++num_cuts;
                    const auto row =
                        xsum(cities::arcs(), [&](const std::size_t a) {
                            return inside[a] * x(a);
                        }) <= static_cast<double>(piece.size()) - 1;
                    if(distinct)
                        handle.add_lazy_constraint(distinct_variables, row);
                    else
                        handle.add_lazy_constraint(row);
                }
            });
            model.solve();

            ASSERT_GE(num_cuts, 1);
            ASSERT_TRUE(is<status::optimal>(model.get_status()));
            ASSERT_NEAR(model.get_solution_value(), cities::shortest_tour(),
                        TEST_EPSILON);
        }
    });
}

// A solver that held an optimum before the callback existed must not hand it
// back unasked: to Gurobi and CPLEX a callback is no change to the model.
TYPED_TEST_P(LazyConstraintsTest, a_callback_set_after_a_solve_is_asked) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto a = model.add_binary_variable();
        auto b = model.add_binary_variable();
        auto c = model.add_binary_variable();
        model.set_maximization();
        model.set_objective(a + 2 * b + 3 * c);
        model.add_constraint(a + b + c <= 2);
        model.solve();
        ASSERT_NEAR(model.get_solution_value(), 5, TEST_EPSILON);

        model.set_candidate_solution_callback([&](auto & handle) {
            auto point = handle.get_solution();
            if(point[b] + point[c] > 1 + TEST_EPSILON)
                handle.add_lazy_constraint(b + c <= 1);
        });
        model.solve();

        ASSERT_TRUE(is<status::optimal>(model.get_status()));
        ASSERT_NEAR(model.get_solution_value(), 4, TEST_EPSILON);
        auto solution = model.get_solution();
        ASSERT_LE(solution[b] + solution[c], 1 + TEST_EPSILON);
    });
}

REGISTER_TYPED_TEST_SUITE_P(LazyConstraintsTest, cuts_off_candidates,
                            cuts_off_candidates_distinct_variables,
                            a_satisfied_constraint_rejects_nothing,
                            a_violated_constraint_cuts_among_satisfied,
                            zero_coefficients_change_nothing,
                            a_callback_set_after_a_solve_is_asked);

}  // namespace mippp
