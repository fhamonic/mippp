#pragma once

#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <random>
#include <ranges>
#include <utility>
#include <vector>

#include "mippp/linear_constraint.hpp"
#include "mippp/model_concepts.hpp"

namespace mippp {

// MILP models only: an LP solve stopped by its time limit leaves a basis
// rather than an incumbent.
template <typename T>
struct TimeLimitIncumbentTest : public T {
    using typename T::model_type;
    static_assert(milp_model<model_type>);
    static_assert(has_time_limit<model_type>);

    // Multidimensional 0/1 knapsack, 10 dense rows each half as large as the
    // sum of its weights, for which x = 0 is feasible. Some solvers close it
    // within the limit at one size and not at the next.
    struct knapsack {
        std::vector<int> profits;
        std::vector<std::vector<int>> rows;
        std::vector<int> capacities;
    };
    static auto build_knapsack(model_type & model, std::size_t n) {
        using namespace operators;
        std::mt19937 rng(42);
        std::uniform_int_distribution<int> coef(1, 1000);
        auto items = std::views::iota(std::size_t{0}, n);
        knapsack data{std::vector<int>(n), {}, {}};
        for(int & p : data.profits) p = coef(rng);
        auto x = model.add_binary_variables(n);
        model.set_maximization();
        model.set_objective(
            xsum(items, [&](auto j) { return data.profits[j] * x(j); }));
        for(std::size_t i = 0; i < 10; ++i) {
            auto & row = data.rows.emplace_back(n);
            int total = 0;
            for(int & w : row) total += (w = coef(rng));
            data.capacities.push_back(total / 2);
            model.add_constraint(xsum(items, [&](auto j) {
                                     return row[j] * x(j);
                                 }) <= data.capacities.back());
        }
        return std::make_pair(x, std::move(data));
    }
};
TYPED_TEST_SUITE_P(TimeLimitIncumbentTest);
GTEST_ALLOW_UNINSTANTIATED_PARAMETERIZED_TEST(TimeLimitIncumbentTest);

// The stopped solve keeps its incumbent: the status says one is available,
// and get_solution() returns it for the model as built.
//
// The solver must hold an incumbent when stopped, which heuristics alone do
// not ensure: COPT 8 searches the size-100 instance for seconds without
// finding x = 0. So x = 0 is handed as a MIP start to the models that take
// one; SCIP, which takes none, finds it with its trivial heuristic.
TYPED_TEST_P(TimeLimitIncumbentTest, keeps_the_incumbent) {
    using model_type = typename TestFixture::model_type;
    this->SkipOnLicenseError([this]() {
        constexpr std::chrono::duration<double> limit{0.5};
        for(const std::size_t size : {100u, 200u, 400u, 800u}) {
            auto model = this->new_model();
            auto [x, data] = TestFixture::build_knapsack(model, size);
            if constexpr(has_mip_start<model_type>) {
                std::vector<std::pair<model_variable_t<model_type>,
                                      model_scalar_t<model_type>>>
                    start;
                for(std::size_t j = 0; j < size; ++j)
                    start.emplace_back(x(j), 0.0);
                model.add_mip_start(start);
            }
            model.set_time_limit(limit);
            model.solve();
            const auto & outcome = model.get_status();
            if(is_a<status::completed>(outcome)) continue;

            ASSERT_TRUE(is_a<status::time_limit>(outcome))
                << "at size " << size;
            ASSERT_TRUE(status::solution_available(outcome))
                << "at size " << size;
            ASSERT_EQ(model.num_variables(), size);
            ASSERT_EQ(model.num_constraints(), data.rows.size());
            auto solution = model.get_solution();
            double objective = 0.0;
            std::vector<double> loads(data.rows.size(), 0.0);
            for(std::size_t j = 0; j < size; ++j) {
                const double v = solution[x(j)];
                ASSERT_NEAR(v, v < 0.5 ? 0.0 : 1.0, 1e-4)
                    << "x" << j << " at size " << size;
                objective += data.profits[j] * v;
                for(std::size_t i = 0; i < data.rows.size(); ++i)
                    loads[i] += data.rows[i][j] * v;
            }
            for(std::size_t i = 0; i < data.rows.size(); ++i)
                ASSERT_LE(loads[i], data.capacities[i] +
                                        TEST_EPSILON * data.capacities[i])
                    << "row " << i << " at size " << size;
            ASSERT_NEAR(objective, model.get_solution_value(),
                        TEST_EPSILON * (1.0 + objective))
                << "at size " << size;
            return;
        }
        GTEST_SKIP() << "the solver closed even the largest instance before "
                        "the "
                     << limit.count() << "s limit could interrupt it";
    });
}

REGISTER_TYPED_TEST_SUITE_P(TimeLimitIncumbentTest, keeps_the_incumbent);

}  // namespace mippp
