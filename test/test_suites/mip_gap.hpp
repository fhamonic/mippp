#pragma once

#include <gtest/gtest.h>

#include <cstddef>
#include <random>
#include <ranges>
#include <vector>

#include "mippp/linear_constraint.hpp"
#include "mippp/model_concepts.hpp"

namespace mippp {

// On a MILP model the optimality tolerance is the relative gap. On an LP
// model it is the solver's own optimality tolerance, such as cplex_lp's
// reduced-cost tolerance.
template <typename T>
struct MipGapTest : public T {
    using typename T::model_type;
    static_assert(milp_model<model_type>);
    static_assert(has_optimality_tolerance<model_type>);

    // Multidimensional 0/1 knapsack, 5 dense rows each half as large as the
    // sum of its weights: presolve leaves its relaxation fractional.
    static auto build_knapsack(model_type & model, std::size_t n) {
        using namespace operators;
        std::mt19937 rng(42);
        std::uniform_int_distribution<int> coef(1, 1000);
        auto items = std::views::iota(std::size_t{0}, n);
        std::vector<int> row(n);
        for(int & p : row) p = coef(rng);
        auto x = model.add_binary_variables(n);
        model.set_maximization();
        model.set_objective(xsum(items, [&](auto j) { return row[j] * x(j); }));
        for(int i = 0; i < 5; ++i) {
            int total = 0;
            for(int & w : row) total += (w = coef(rng));
            model.add_constraint(xsum(items, [&](auto j) {
                                     return row[j] * x(j);
                                 }) <= total / 2);
        }
        return x;
    }
};
TYPED_TEST_SUITE_P(MipGapTest);
GTEST_ALLOW_UNINSTANTIATED_PARAMETERIZED_TEST(MipGapTest);

// Stopping at the gap is optimal, and a loose gap keeps the values integral.
// Written to an integrality tolerance instead, it would let the relaxation's
// fractional values pass for integers.
TYPED_TEST_P(MipGapTest, loose_gap_is_optimal_and_integral) {
    this->SkipOnLicenseError([this]() {
        constexpr std::size_t size = 60;
        auto model = this->new_model();
        auto x = TestFixture::build_knapsack(model, size);
        model.set_optimality_tolerance(0.45);
        model.solve();
        ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
        auto solution = model.get_solution();
        for(std::size_t j = 0; j < size; ++j) {
            const double v = solution[x(j)];
            ASSERT_NEAR(v, v < 0.5 ? 0.0 : 1.0, 1e-4) << "x" << j;
        }
    });
}

REGISTER_TYPED_TEST_SUITE_P(MipGapTest, loose_gap_is_optimal_and_integral);

}  // namespace mippp
