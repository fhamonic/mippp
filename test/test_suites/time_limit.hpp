#pragma once

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "assert_helper.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <limits>
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

    // Cornuejols-Dawande market split: m dense equality rows over 10 (m - 1)
    // binaries. The slacks keep it feasible, their sum is minimized. Called
    // for MILP models only, and a member function of a class template is only
    // instantiated where called, so LP models never compile its MILP calls.
    static void build_market_split(model_type & model, std::size_t m) {
        using namespace operators;
        const std::size_t n = 10 * (m - 1);
        std::mt19937 rng(42);
        std::uniform_int_distribution<int> coef(0, 99);
        auto x = model.add_binary_variables(n);
        auto plus = model.add_variables(m);
        auto minus = model.add_variables(m);
        model.set_minimization();
        model.set_objective(xsum(std::views::iota(std::size_t{0}, m),
                                 [&](auto i) { return plus(i) + minus(i); }));
        std::vector<int> row(n);
        for(std::size_t i = 0; i < m; ++i) {
            int total = 0;
            for(int & a : row) total += (a = coef(rng));
            model.add_constraint(xsum(std::views::iota(std::size_t{0}, n),
                                      [&](auto j) { return row[j] * x(j); }) +
                                     plus(i) - minus(i) ==
                                 total / 2);
        }
    }
    // positive coefficients over x >= 0 keep it bounded
    static void build_dense_lp(model_type & model, std::size_t size) {
        using namespace operators;
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
    }
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

// Each backend spells "no limit" its own way, from 1e20 to +inf, but never as
// a negative value, which would turn min(remaining, get_time_limit()) negative.
TYPED_TEST_P(TimeLimitTest, fresh_time_limit_is_unlimited) {
    this->SkipOnLicenseError([this]() {
        using seconds = std::chrono::duration<double>;
        auto model = this->new_model();
        const seconds fresh = model.get_time_limit();
        ASSERT_FALSE(std::isnan(fresh.count()));
        ASSERT_GE(fresh.count(), 1e9);
        model.set_time_limit(fresh);
        EXPECT_EQ(seconds(model.get_time_limit()).count(), fresh.count());
    });
}

TYPED_TEST_P(TimeLimitTest, unlimited_time_limit_round_trips) {
    this->SkipOnLicenseError([this]() {
        using seconds = std::chrono::duration<double>;
        for(const seconds unlimited :
            {seconds(std::numeric_limits<double>::infinity()),
             seconds::max()}) {
            auto model = this->new_model();
            model.set_time_limit(seconds(5.0));
            ASSERT_NO_THROW(model.set_time_limit(unlimited))
                << unlimited.count();
            const seconds read = model.get_time_limit();
            ASSERT_GE(read.count(), 1e9) << unlimited.count();
            model.set_time_limit(read);
            EXPECT_EQ(seconds(model.get_time_limit()).count(), read.count())
                << unlimited.count();
        }
    });
}

// A wrapper that keeps its own copy of a value the solver silently refused
// passes the round trips above: only a solve shows the solver's limit.
TYPED_TEST_P(TimeLimitTest, lifted_time_limit_takes_effect) {
    this->SkipOnLicenseError([this]() {
        using seconds = std::chrono::duration<double>;
        for(const seconds unlimited :
            {seconds(std::numeric_limits<double>::infinity()),
             seconds::max()}) {
            auto model = this->new_model();
            TestFixture::build_dense_lp(model, 3);
            model.set_time_limit(seconds(0.0));
            model.set_time_limit(unlimited);
            model.solve();
            EXPECT_TRUE(is_a<status::completed>(model.get_status()))
                << unlimited.count();
        }
    });
}

TYPED_TEST_P(TimeLimitTest, negative_time_limit_is_never_read_back) {
    this->SkipOnLicenseError([this]() {
        using seconds = std::chrono::duration<double>;
        auto model = this->new_model();
        try {
            model.set_time_limit(seconds(-1.0));
        } catch(const license_error &) {
            throw;
        } catch(const std::exception &) {
            // a setter may refuse a negative limit
        }
        const double read = seconds(model.get_time_limit()).count();
        EXPECT_FALSE(std::isnan(read));
        EXPECT_GE(read, 0.0);
    });
}

TYPED_TEST_P(TimeLimitTest, forwarded_time_limit_restores_exactly) {
    this->SkipOnLicenseError([this]() {
        using seconds = std::chrono::duration<double>;
        for(const bool caller_limit : {false, true}) {
            auto model = this->new_model();
            if(caller_limit) model.set_time_limit(seconds(2.0));
            const seconds saved = model.get_time_limit();
            for(const seconds remaining :
                {seconds(1e-3), seconds(0.25), seconds(3.0)}) {
                const seconds forwarded = std::min(remaining, saved);
                ASSERT_GT(forwarded.count(), 0.0);
                ASSERT_LE(forwarded.count(), remaining.count());
                model.set_time_limit(forwarded);
                EXPECT_NEAR(seconds(model.get_time_limit()).count(),
                            forwarded.count(), 1e-9);
                model.set_time_limit(saved);
                EXPECT_EQ(seconds(model.get_time_limit()).count(),
                          saved.count());
            }
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
// MILP models solve market split instances, which branch-and-bound cannot
// close within the limit from m = 4 or 5 on, far below the size caps of
// community licences. LP models solve dense random LPs under a 0.2 s limit:
// one that outlasts 1 s takes SoPlex longer to build than to solve, and a
// shorter limit meets clock offsets (SoPlex stops at a 50 ms limit after some
// 15 ms).
TYPED_TEST_P(TimeLimitTest, interrupts_long_solve) {
    using model_type = typename TestFixture::model_type;
    this->SkipOnLicenseError([this]() {
        using seconds = std::chrono::duration<double>;

        auto solve_within = [&, this](std::size_t size, seconds limit) {
            auto model = this->new_model();
            if constexpr(milp_model<model_type>)
                TestFixture::build_market_split(model, size);
            else
                TestFixture::build_dense_lp(model, size);
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
                            fresh_time_limit_is_unlimited,
                            unlimited_time_limit_round_trips,
                            lifted_time_limit_takes_effect,
                            negative_time_limit_is_never_read_back,
                            forwarded_time_limit_restores_exactly,
                            interrupts_long_solve);

}  // namespace mippp