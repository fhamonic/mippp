#pragma once

#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <ctime>
#include <limits>

#include "mippp/model_concepts.hpp"

#include "time_limit.hpp"

namespace mippp {

template <typename T>
struct ThreadLimitTest : public T {
    using typename T::model_type;
    static_assert(lp_model<model_type>);
    static_assert(has_thread_limit<model_type>);

    // The core-counting tests can only fail where the default solve is
    // parallel: a smaller instance or a shorter limit lets them pass whatever
    // set_thread_limit does. At these sizes, on 16 hardware threads, the
    // defaults kept 5 to 13 cores busy on the MILP, and 2.4 to 4.1 on the LP
    // of Gurobi, COPT and MOSEK; CPLEX and Xpress solved the LP on one core.
    // Within the community editions' sizes.
    static void build(model_type & model) {
        if constexpr(milp_model<model_type>) {
            TimeLimitTest<T>::build_market_split(model, 6);
            model.set_time_limit(std::chrono::milliseconds(500));
        } else {
            TimeLimitTest<T>::build_dense_lp(model, 900);
        }
    }
    struct solve_usage {
        double cores;  // CPU time over wall time
        std::chrono::duration<double> wall;
    };
    // std::clock() counts every thread of the process on POSIX systems; on
    // Windows it measures wall time, so the ratio is 1 there and the two
    // core-counting tests pass vacuously.
    static solve_usage measured_solve(model_type & model) {
        const std::clock_t cpu_start = std::clock();
        const auto wall_start = std::chrono::steady_clock::now();
        model.solve();
        const double cpu =
            static_cast<double>(std::clock() - cpu_start) / CLOCKS_PER_SEC;
        const std::chrono::duration<double> wall =
            std::chrono::steady_clock::now() - wall_start;
        return {cpu / wall.count(), wall};
    }
    // One thread uses one core, give or take the solver's helper threads.
    // Load from other processes only lowers the ratio, so this needs no
    // serial run.
    static constexpr double one_core = 1.5;
    // A shorter solve is mostly set-up, and its ratio is noise. Every solve
    // these tests mean to measure took 0.2 s or more.
    static constexpr std::chrono::milliseconds measurable_solve{100};
};
TYPED_TEST_SUITE_P(ThreadLimitTest);
GTEST_ALLOW_UNINSTANTIATED_PARAMETERIZED_TEST(ThreadLimitTest);

TYPED_TEST_P(ThreadLimitTest, set_get_thread_limit) {
    this->SkipOnLicenseError([this]() {
        auto model = this->new_model();
        ASSERT_EQ(model.get_thread_limit(), 0u);
        model.set_thread_limit(1);
        ASSERT_EQ(model.get_thread_limit(), 1u);
        model.set_thread_limit(3);
        ASSERT_EQ(model.get_thread_limit(), 3u);
        model.set_thread_limit(0);
        ASSERT_EQ(model.get_thread_limit(), 0u);
    });
}
// Gurobi and CPLEX reject a native value above their maximum, and an int
// cannot hold every std::size_t.
TYPED_TEST_P(ThreadLimitTest, a_limit_above_the_maximum_is_capped) {
    this->SkipOnLicenseError([this]() {
        auto model = this->new_model();
        model.set_thread_limit(std::numeric_limits<std::size_t>::max());
        const std::size_t cap = model.get_thread_limit();
        ASSERT_GE(cap, 128u);
        model.set_thread_limit(cap);
        ASSERT_EQ(model.get_thread_limit(), cap);
    });
}
TYPED_TEST_P(ThreadLimitTest, one_thread_uses_one_core) {
    this->SkipOnLicenseError([this]() {
        auto model = this->new_model();
        model.set_thread_limit(1);
        TestFixture::build(model);
        EXPECT_LT(TestFixture::measured_solve(model).cores,
                  TestFixture::one_core);
    });
}
// Skipped on the duration of the second solve, not on the model kind: a
// simplex LP solved again restarts from its optimal basis and ends at once,
// while MOSEK's interior point, and every MILP, runs again.
TYPED_TEST_P(ThreadLimitTest, a_solved_model_takes_a_new_limit) {
    this->SkipOnLicenseError(
        [this]() {
            auto model = this->new_model();
            TestFixture::build(model);
            model.solve();
            model.set_thread_limit(1);
            const auto usage = TestFixture::measured_solve(model);
            if(usage.wall < TestFixture::measurable_solve)
                GTEST_SKIP()
                    << "solved again in "
                    << std::chrono::duration_cast<std::chrono::milliseconds>(
                           usage.wall)
                           .count()
                    << " ms, too soon to measure";
            EXPECT_LT(usage.cores, TestFixture::one_core);
        });
}

REGISTER_TYPED_TEST_SUITE_P(ThreadLimitTest, set_get_thread_limit,
                            a_limit_above_the_maximum_is_capped,
                            one_thread_uses_one_core,
                            a_solved_model_takes_a_new_limit);

}  // namespace mippp
