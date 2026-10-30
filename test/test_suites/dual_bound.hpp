#pragma once

#include <gtest/gtest.h>

#include <algorithm>
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

REGISTER_TYPED_TEST_SUITE_P(DualBoundTest, bounds_a_minimization_from_below,
                            bounds_a_maximization_from_above,
                            loose_gap_keeps_the_bound_on_its_side,
                            integer_free_model_reports_its_lp_optimum);

}  // namespace mippp
