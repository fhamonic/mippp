#pragma once

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <random>
#include <ranges>
#include <vector>

#include "mippp/linear_constraint.hpp"
#include "mippp/model_concepts.hpp"

namespace mippp {

template <typename T>
struct DualBoundTest : public T {
    using typename T::model_type;
    static_assert(milp_model<model_type>);
    static_assert(has_dual_bound<model_type>);

    // Multidimensional 0/1 knapsack over 5 dense rows, each half as large as
    // the sum of its weights: a packing when maximizing, a covering when
    // minimizing. The offset pins the backends that add it themselves.
    static auto build_knapsack(model_type & model, std::size_t n,
                               bool maximize) {
        using namespace operators;
        std::mt19937 rng(42);
        std::uniform_int_distribution<int> coef(1, 1000);
        auto items = std::views::iota(std::size_t{0}, n);
        std::vector<int> row(n);
        for(int & p : row) p = coef(rng);
        auto x = model.add_binary_variables(n);
        if(maximize)
            model.set_maximization();
        else
            model.set_minimization();
        model.set_objective(xsum(items, [&](auto j) { return row[j] * x(j); }) +
                            1000);
        for(int i = 0; i < 5; ++i) {
            int total = 0;
            for(int & w : row) total += (w = coef(rng));
            auto lhs = xsum(items, [&](auto j) { return row[j] * x(j); });
            if(maximize)
                model.add_constraint(lhs <= total / 2);
            else
                model.add_constraint(lhs >= total / 2);
        }
        return x;
    }

    // the default optimality tolerance, a relative gap of 1e-4
    static double default_gap(double value, double bound) {
        return 1e-4 * std::max(std::abs(value), std::abs(bound)) + TEST_EPSILON;
    }
};
TYPED_TEST_SUITE_P(DualBoundTest);
GTEST_ALLOW_UNINSTANTIATED_PARAMETERIZED_TEST(DualBoundTest);

TYPED_TEST_P(DualBoundTest, bounds_a_minimization_from_below) {
    this->SkipOnLicenseError([this]() {
        auto model = this->new_model();
        TestFixture::build_knapsack(model, 25, false);
        model.solve();
        ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
        const double value = model.get_solution_value();
        const double bound = model.get_dual_bound();
        EXPECT_LE(bound, value + TEST_EPSILON);
        EXPECT_GE(bound, value - TestFixture::default_gap(value, bound));
    });
}

TYPED_TEST_P(DualBoundTest, bounds_a_maximization_from_above) {
    this->SkipOnLicenseError([this]() {
        auto model = this->new_model();
        TestFixture::build_knapsack(model, 25, true);
        model.solve();
        ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
        const double value = model.get_solution_value();
        const double bound = model.get_dual_bound();
        EXPECT_GE(bound, value - TEST_EPSILON);
        EXPECT_LE(bound, value + TestFixture::default_gap(value, bound));
    });
}

// A solve stopped at a loose gap leaves the bound on its side of the
// incumbent and no further than the gap. Every backend's gap divides by the
// incumbent, the bound or the smaller of the two, all at most the larger.
TYPED_TEST_P(DualBoundTest, loose_gap_keeps_the_bound_on_its_side) {
    using M = typename TestFixture::model_type;
    if constexpr(!has_optimality_tolerance<M>) {
        GTEST_SKIP() << "no optimality tolerance";
    } else {
        this->SkipOnLicenseError([this]() {
            constexpr double gap = 0.45;
            auto model = this->new_model();
            TestFixture::build_knapsack(model, 60, true);
            model.set_optimality_tolerance(gap);
            model.solve();
            ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
            const double value = model.get_solution_value();
            const double bound = model.get_dual_bound();
            EXPECT_GE(bound, value - TEST_EPSILON);
            EXPECT_LE(bound - value,
                      gap * std::max(std::abs(value), std::abs(bound)) +
                          TEST_EPSILON);
        });
    }
}

// Relaxed after a search, the model is solved as an LP, whose optimum is its
// own bound: no bound of the earlier search may remain.
TYPED_TEST_P(DualBoundTest, integer_free_model_reports_its_lp_optimum) {
    this->SkipOnLicenseError([this]() {
        constexpr std::size_t size = 25;
        auto model = this->new_model();
        auto x = TestFixture::build_knapsack(model, size, true);
        model.solve();
        ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
        const double mip_value = model.get_solution_value();
        for(std::size_t j = 0; j < size; ++j) model.set_continuous(x(j));
        model.solve();
        ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
        const double value = model.get_solution_value();
        EXPECT_GE(value, mip_value - TEST_EPSILON);
        EXPECT_NEAR(model.get_dual_bound(), value,
                    TEST_EPSILON * std::max(1.0, std::abs(value)));
    });
}

// Market split with a planted point: feasible, which proves the bound must not
// pass that point's value, yet out of reach of heuristics. COPT drops the bound
// of a search stopped without an incumbent and reads +1e30 in either sense.
TYPED_TEST_P(DualBoundTest, a_stop_without_an_incumbent_keeps_a_valid_bound) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        for(const bool maximize : {false, true}) {
            const std::size_t rows = 7, cols = 60;
            std::mt19937 rng(42);
            std::uniform_int_distribution<int> coef(0, 99), coin(0, 1);
            std::vector<int> planted(cols);
            int planted_size = 0;
            for(int & p : planted) planted_size += (p = coin(rng));
            auto model = this->new_model();
            auto x = model.add_binary_variables(cols);
            const auto col_ids = std::views::iota(std::size_t{0}, cols);
            if(maximize)
                model.set_maximization();
            else
                model.set_minimization();
            model.set_objective(
                xsum(col_ids, [&](auto j) { return 1.0 * x(j); }) + 1000);
            std::vector<int> row(cols);
            for(std::size_t i = 0; i < rows; ++i) {
                int rhs = 0;
                for(std::size_t j = 0; j < cols; ++j) {
                    row[j] = coef(rng);
                    rhs += row[j] * planted[j];
                }
                model.add_constraint(xsum(col_ids, [&](auto j) {
                                         return row[j] * x(j);
                                     }) == rhs);
            }
            model.set_time_limit(std::chrono::seconds(1));
            model.solve();
            const auto status = model.get_status();
            if(!is_a<status::stopped>(status) ||
               status::solution_available(status))
                GTEST_SKIP() << "found the planted point or closed the search";
            const double value_at_planted = 1000 + planted_size;
            const double bound = model.get_dual_bound();
            // Past the root any bound lies within the objective's range, which
            // tells a bound the search reached from a mere "no bound" infinity.
            if(maximize) {
                EXPECT_GE(bound, value_at_planted - TEST_EPSILON);
                EXPECT_LE(bound, 1000 + cols + TEST_EPSILON);
            } else {
                EXPECT_LE(bound, value_at_planted + TEST_EPSILON);
                EXPECT_GE(bound, 1000 - TEST_EPSILON);
            }
        }
    });
}

REGISTER_TYPED_TEST_SUITE_P(DualBoundTest, bounds_a_minimization_from_below,
                            a_stop_without_an_incumbent_keeps_a_valid_bound,
                            bounds_a_maximization_from_above,
                            loose_gap_keeps_the_bound_on_its_side,
                            integer_free_model_reports_its_lp_optimum);

}  // namespace mippp
