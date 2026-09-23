#pragma once

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "assert_helper.hpp"

#include <chrono>
#include <random>
#include <ranges>
#include <vector>

#include "mippp/linear_constraint.hpp"
#include "mippp/model_concepts.hpp"

namespace mippp {

template <typename T>
struct TimeLimitTest : public T {
    using typename T::model_type;
    static_assert(lp_model<model_type>);
    static_assert(has_time_limit<model_type>);
};
TYPED_TEST_SUITE_P(TimeLimitTest);
GTEST_ALLOW_UNINSTANTIATED_PARAMETERIZED_TEST(TimeLimitTest);

// The getter/setter contract is fully deterministic, so we test it on its own,
// free of any wall-clock measurement.
TYPED_TEST_P(TimeLimitTest, set_get_time_limit) {
    this->SkipOnLicenseError([this]() {
        auto model = this->new_model();
        using seconds = std::chrono::duration<double>;
        for(double s : {0.5, 1.0, 7.0, 42.0, 123.5}) {
            model.set_time_limit(seconds(s));
            ASSERT_NEAR(
                std::chrono::duration_cast<seconds>(model.get_time_limit())
                    .count(),
                s, 1e-3);
        }
    });
}

// Behavioural test: the time limit must stop a solve that would otherwise run
// well past it, at the right time and with the right status.
//
// How long an instance keeps a solver busy depends on the solver and on the
// machine, so the instance grows until a solve is stopped. Every solve either
// completes or reports time_limit, and none outlasts the limit by more than
// the overshoot; the first time_limit, which must not come well before the
// limit, ends the test. A solver that closes even the largest instance cannot
// be exercised here and skips.
//
// MILP models solve Cornuejols-Dawande market split instances: m dense
// equality rows over 10 (m - 1) binaries, which branch-and-bound cannot close
// within the limit from m = 4 or 5 on, far below the size caps of community
// licences. LP models solve dense random LPs under a 0.2 s limit: one that
// outlasts 1 s takes SoPlex longer to build than to solve, and a shorter limit
// meets clock offsets (SoPlex stops at a 50 ms limit after some 15 ms).
TYPED_TEST_P(TimeLimitTest, interrupts_long_solve) {
    using model_type = typename TestFixture::model_type;
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        using seconds = std::chrono::duration<double>;

        // generic: its body uses MILP members, so must not be instantiated
        // for LP models. The slacks keep it feasible, their sum is minimized.
        auto build_market_split = [](auto & model, std::size_t m) {
            const std::size_t n = 10 * (m - 1);
            std::mt19937 rng(42);
            std::uniform_int_distribution<int> coef(0, 99);
            auto x = model.add_binary_variables(n);
            auto plus = model.add_variables(m);
            auto minus = model.add_variables(m);
            model.set_minimization();
            model.set_objective(
                xsum(std::views::iota(std::size_t{0}, m),
                     [&](auto i) { return plus(i) + minus(i); }));
            std::vector<int> row(n);
            for(std::size_t i = 0; i < m; ++i) {
                int total = 0;
                for(int & a : row) total += (a = coef(rng));
                model.add_constraint(
                    xsum(std::views::iota(std::size_t{0}, n),
                         [&](auto j) { return row[j] * x(j); }) +
                        plus(i) - minus(i) ==
                    total / 2);
            }
        };
        // positive coefficients over x >= 0 keep it bounded
        auto build_dense_lp = [](model_type & model, std::size_t size) {
            std::mt19937 rng(42);
            std::uniform_int_distribution<int> coef(1, 1000);
            auto columns = std::views::iota(std::size_t{0}, size);
            auto x = model.add_variables(size);
            std::vector<int> row(size);
            for(int & a : row) a = coef(rng);
            model.set_maximization();
            model.set_objective(
                xsum(columns, [&](auto j) { return row[j] * x(j); }));
            for(std::size_t i = 0; i < size; ++i) {
                for(int & a : row) a = coef(rng);
                model.add_constraint(xsum(columns, [&](auto j) {
                                         return row[j] * x(j);
                                     }) <= 1000.0 * static_cast<double>(size));
            }
        };

        auto solve_within = [&, this](std::size_t size, seconds limit) {
            auto model = this->new_model();
            if constexpr(milp_model<model_type>)
                build_market_split(model, size);
            else
                build_dense_lp(model, size);
            model.set_time_limit(limit);

            // steady_clock: monotonic, unaffected by wall-clock adjustments.
            auto start = std::chrono::steady_clock::now();
            model.solve();
            auto end = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<seconds>(end - start);

            return std::make_pair(duration, model.get_status());
        };

        constexpr bool milp = milp_model<model_type>;
        constexpr seconds limit{milp ? 1.0 : 0.2};
        constexpr seconds overshoot{1.0};
        const std::vector<std::size_t> sizes =
            milp
                ? std::vector<std::size_t>{2, 3, 4, 5, 6, 7}
                : std::vector<std::size_t>{100, 200, 400, 566, 800, 1131, 1600};

        for(const std::size_t size : sizes) {
            const auto [solve_time, outcome] = solve_within(size, limit);
            std::cout << "size " << size << ": " << solve_time.count() << "s"
                      << std::endl;

            ASSERT_LE(solve_time.count(), (limit + overshoot).count())
                << "at size " << size;
            if(is_a<status::completed>(outcome)) continue;
            ASSERT_TRUE(is_a<status::time_limit>(outcome))
                << "at size " << size
                << ": a solve under a time limit either completes or "
                   "reports time_limit";
            ASSERT_GE(solve_time.count(), 0.5 * limit.count())
                << "time_limit reported well before the limit, at size "
                << size;
            return;
        }
        GTEST_SKIP()
            << "the solver closed even the largest instance before the "
            << limit.count() << "s limit could interrupt it";
    });
}

REGISTER_TYPED_TEST_SUITE_P(TimeLimitTest, set_get_time_limit,
                            interrupts_long_solve);

}  // namespace mippp