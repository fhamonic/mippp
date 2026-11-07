#pragma once

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <random>
#include <ranges>
#include <utility>
#include <vector>

#include "mippp/linear_constraint.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/utility/status.hpp"

namespace mippp {

// What the solvers share, for the suites below.
template <typename T>
struct ProgressCallbackFixture : public T {
    using typename T::model_type;
    using handle_type = progress_callback_handle_t<model_type>;

    static constexpr double objective_offset = 100;
    static constexpr double infinity = std::numeric_limits<double>::infinity();

    struct sample {
        double solution_value;
        double dual_bound;
    };
    // At 1e5 calls per second on Gurobi, unbounded it weighs on the search.
    static constexpr std::size_t kept_samples = 1u << 16;

    // A multidimensional knapsack, where incumbents and the bound both move.
    // Minimized as -c.x + offset, the optimum is far below 0, so that a "no
    // value yet" read as 0 or as a large finite number breaks the assertions.
    // Maximized as c.x + offset, it has the opposite sign, for the same check
    // of the other infinity.
    static auto build_knapsack(model_type & model, const std::size_t rows,
                               const std::size_t cols, const bool maximize) {
        using namespace operators;
        std::mt19937 rng(7);
        std::uniform_int_distribution<int> coef(1, 1000), noise(0, 500);
        std::vector<std::vector<double>> a(rows, std::vector<double>(cols));
        for(auto & row : a)
            for(auto & v : row) v = coef(rng);
        std::vector<double> c(cols);
        for(std::size_t j = 0; j < cols; ++j) {
            double sum = 0;
            for(const auto & row : a) sum += row[j];
            c[j] = std::floor(sum / static_cast<double>(rows) + noise(rng));
        }
        auto x = model.add_binary_variables(cols);
        const auto col_ids = std::views::iota(std::size_t{0}, cols);
        if(maximize) {
            model.set_maximization();
            model.set_objective(
                xsum(col_ids, [&](auto j) { return c[j] * x(j); }) +
                objective_offset);
        } else {
            model.set_minimization();
            model.set_objective(
                xsum(col_ids, [&](auto j) { return (-c[j]) * x(j); }) +
                objective_offset);
        }
        for(const auto & row : a) {
            double sum = 0;
            for(double v : row) sum += v;
            model.add_constraint(xsum(col_ids, [&](auto j) {
                                     return row[j] * x(j);
                                 }) <= std::floor(sum / 2));
        }
        return x;
    }
    // Solved in well under a second on every solver. At their default thread
    // counts, CPLEX and Xpress close it at the root.
    static auto build_small(model_type & model, const bool maximize = false) {
        return build_knapsack(model, 5, 25, maximize);
    }
    // No solver closes it within the limits of these tests.
    static auto build_large(model_type & model) {
        return build_knapsack(model, 30, 100, false);
    }
    // Market split without its slacks: no heuristic finds a point, so the
    // incumbent stays absent for as long as the limit lets the search run.
    static void build_no_incumbent(model_type & model, const bool maximize) {
        using namespace operators;
        const std::size_t rows = 7, cols = 10 * (rows - 1);
        std::mt19937 rng(42);
        std::uniform_int_distribution<int> coef(0, 99);
        auto x = model.add_binary_variables(cols);
        const auto col_ids = std::views::iota(std::size_t{0}, cols);
        if(maximize)
            model.set_maximization();
        else
            model.set_minimization();
        model.set_objective(xsum(col_ids, [&](auto j) { return 1.0 * x(j); }));
        std::vector<int> row(cols);
        for(std::size_t i = 0; i < rows; ++i) {
            int total = 0;
            for(int & a : row) total += (a = coef(rng));
            model.add_constraint(xsum(col_ids, [&](auto j) {
                                     return row[j] * x(j);
                                 }) == total / 2);
        }
    }
    double optimum_of_small(const bool maximize = false) {
        auto model = this->new_model();
        build_small(model, maximize);
        model.solve();
        return model.get_solution_value();
    }
    static void use_one_thread(model_type & model) {
        if constexpr(has_thread_limit<model_type>) model.set_thread_limit(1);
    }
    template <typename Model>
    static std::optional<std::size_t> node_count_of(Model & model) {
        if constexpr(has_node_count<Model>)
            return model.get_node_count();
        else
            return std::nullopt;
    }
};

template <typename T>
struct ProgressCallbackTest : public ProgressCallbackFixture<T> {
    using typename T::model_type;
    using base = ProgressCallbackFixture<T>;
    using typename base::handle_type;
    using typename base::sample;
    static_assert(milp_model<model_type>);
    static_assert(has_progress_callback<model_type>);
    static_assert(has_time_limit<model_type>);
    static_assert(!has_enumerable_variables<handle_type, model_type>);
    static_assert(!has_enumerable_constraints<handle_type, model_type>);

    // Runs the small knapsack and checks every sample against the final answer.
    void reports_the_search(const bool maximize) {
        auto model = this->new_model();
        base::build_small(model, maximize);
        std::size_t num_calls = 0;
        std::vector<sample> samples;
        model.set_progress_callback([&](auto & handle) {
            ++num_calls;
            if(samples.size() < base::kept_samples)
                samples.push_back(
                    {handle.get_solution_value(), handle.get_dual_bound()});
        });
        model.solve();
        ASSERT_TRUE(is<status::optimal>(model.get_status()));
        const double value = model.get_solution_value();
        const double sign = maximize ? -1 : 1;

        ASSERT_GE(num_calls, 1u);
        bool saw_incumbent = false, saw_bound = false;
        double worst_kept_value = sign * base::infinity;
        double best_kept_bound = -sign * base::infinity;
        for(const sample & s : samples) {
            // a native sentinel left as is fails here
            ASSERT_TRUE(std::isinf(s.solution_value) ||
                        std::abs(s.solution_value) < 1e9)
                << s.solution_value;
            ASSERT_TRUE(std::isinf(s.dual_bound) ||
                        std::abs(s.dual_bound) < 1e9)
                << s.dual_bound;
            // the right infinity on either side, and no value past the optimum
            ASSERT_GE(sign * s.solution_value, sign * value - TEST_EPSILON);
            ASSERT_LE(sign * s.dual_bound, sign * value + TEST_EPSILON);
            // never worse than an earlier call: a worker's own view would be
            ASSERT_LE(sign * s.solution_value,
                      sign * worst_kept_value + TEST_EPSILON);
            ASSERT_GE(sign * s.dual_bound,
                      sign * best_kept_bound - TEST_EPSILON);
            worst_kept_value = s.solution_value;
            best_kept_bound = s.dual_bound;
            saw_incumbent = saw_incumbent || !std::isinf(s.solution_value);
            saw_bound = saw_bound || !std::isinf(s.dual_bound);
        }
        ASSERT_TRUE(saw_incumbent);
        ASSERT_TRUE(saw_bound);
    }
    void reads_no_incumbent_as_infinite(const bool maximize) {
        auto model = this->new_model();
        base::build_no_incumbent(model, maximize);
        model.set_time_limit(std::chrono::seconds(30));
        std::size_t num_calls = 0, num_wrong = 0;
        model.set_progress_callback([&](auto & handle) {
            if(handle.get_solution_value() !=
               (maximize ? -base::infinity : base::infinity))
                ++num_wrong;
            if(++num_calls == 20) handle.interrupt();
        });
        model.solve();
        ASSERT_GE(num_calls, 1u);
        ASSERT_EQ(num_wrong, 0u);
    }
};
TYPED_TEST_SUITE_P(ProgressCallbackTest);
GTEST_ALLOW_UNINSTANTIATED_PARAMETERIZED_TEST(ProgressCallbackTest);

TYPED_TEST_P(ProgressCallbackTest, reports_the_search_when_minimizing) {
    this->SkipOnLicenseError([this]() { this->reports_the_search(false); });
}
TYPED_TEST_P(ProgressCallbackTest, reports_the_search_when_maximizing) {
    this->SkipOnLicenseError([this]() { this->reports_the_search(true); });
}
TYPED_TEST_P(ProgressCallbackTest,
             no_incumbent_reads_infinite_when_minimizing) {
    this->SkipOnLicenseError(
        [this]() { this->reads_no_incumbent_as_infinite(false); });
}
TYPED_TEST_P(ProgressCallbackTest,
             no_incumbent_reads_infinite_when_maximizing) {
    this->SkipOnLicenseError(
        [this]() { this->reads_no_incumbent_as_infinite(true); });
}
TYPED_TEST_P(ProgressCallbackTest, interrupt_ends_the_solve) {
    this->SkipOnLicenseError([this]() {
        auto model = this->new_model();
        TestFixture::build_large(model);
        model.set_time_limit(std::chrono::seconds(30));
        std::size_t num_calls = 0;
        model.set_progress_callback([&](auto & handle) {
            ++num_calls;
            handle.interrupt();
        });
        model.solve();
        ASSERT_GE(num_calls, 1u);
        ASSERT_TRUE(is_a<status::interrupted>(model.get_status()));
    });
}
// HiGHS keeps the interrupt flag between its runs until Highs_setCallback
// clears it: a callback registered only once stops the second solve at its
// first check.
TYPED_TEST_P(ProgressCallbackTest, an_interrupt_is_for_one_solve) {
    this->SkipOnLicenseError([this]() {
        const double optimum = this->optimum_of_small();
        auto model = this->new_model();
        TestFixture::build_small(model);
        bool stop = true;
        model.set_progress_callback([&](auto & handle) {
            if(std::exchange(stop, false)) handle.interrupt();
        });
        model.solve();
        ASSERT_TRUE(is_a<status::interrupted>(model.get_status()));
        model.solve();
        ASSERT_TRUE(is<status::optimal>(model.get_status()));
        ASSERT_NEAR(model.get_solution_value(), optimum, TEST_EPSILON);
    });
}
// The callback must not reach the search: on COPT and CPLEX a candidate
// callback does, through the presolve reductions it turns off.
TYPED_TEST_P(ProgressCallbackTest, leaves_the_search_unchanged) {
    this->SkipOnLicenseError([this]() {
        auto without = this->new_model();
        TestFixture::use_one_thread(without);
        TestFixture::build_small(without);
        without.solve();

        auto with = this->new_model();
        TestFixture::use_one_thread(with);
        TestFixture::build_small(with);
        std::size_t num_calls = 0;
        with.set_progress_callback([&](auto &) { ++num_calls; });
        with.solve();

        ASSERT_GE(num_calls, 1u);
        ASSERT_NEAR(with.get_solution_value(), without.get_solution_value(),
                    TEST_EPSILON);
        ASSERT_EQ(TestFixture::node_count_of(with),
                  TestFixture::node_count_of(without));
    });
}
// Registered by the setter, the native user data would point at the moved-from
// model.
TYPED_TEST_P(ProgressCallbackTest, a_moved_model_keeps_its_callback) {
    this->SkipOnLicenseError([this]() {
        auto model = this->new_model();
        TestFixture::build_small(model);
        std::size_t num_calls = 0;
        model.set_progress_callback([&](auto &) { ++num_calls; });
        auto moved = std::move(model);
        moved.solve();
        ASSERT_GE(num_calls, 1u);
        ASSERT_TRUE(is<status::optimal>(moved.get_status()));
    });
}
TYPED_TEST_P(ProgressCallbackTest, a_second_callback_replaces_the_first) {
    this->SkipOnLicenseError([this]() {
        auto model = this->new_model();
        TestFixture::build_small(model);
        std::size_t first = 0, second = 0;
        model.set_progress_callback([&](auto &) { ++first; });
        model.set_progress_callback([&](auto &) { ++second; });
        model.solve();
        ASSERT_EQ(first, 0u);
        ASSERT_GE(second, 1u);
    });
}
// SCIP subscribes once per transformed problem, so a solve that resumes one has
// to subscribe a callback set since. A setter that freed the transformed
// problem instead would lose the results it must keep.
TYPED_TEST_P(ProgressCallbackTest, a_callback_set_after_a_solve_runs) {
    this->SkipOnLicenseError([this]() {
        auto model = this->new_model();
        TestFixture::build_large(model);
        model.set_time_limit(std::chrono::milliseconds(50));
        model.solve();
        std::size_t num_calls = 0;
        model.set_progress_callback([&](auto & handle) {
            ++num_calls;
            handle.interrupt();
        });
        model.set_time_limit(std::chrono::seconds(30));
        model.solve();
        ASSERT_GE(num_calls, 1u);
        ASSERT_TRUE(is_a<status::interrupted>(model.get_status()));
    });
}
// A change frees SCIP's transformed problem, whose event filter goes with it.
TYPED_TEST_P(ProgressCallbackTest, a_changed_model_keeps_its_callback) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x = TestFixture::build_small(model);
        std::size_t num_calls = 0;
        model.set_progress_callback([&](auto &) { ++num_calls; });
        model.solve();
        ASSERT_GE(num_calls, 1u);
        const double first_value = model.get_solution_value();
        num_calls = 0;
        model.add_constraint(x(0) + x(1) + x(2) + x(3) + x(4) <= 1);
        model.solve();
        ASSERT_GE(num_calls, 1u);
        ASSERT_GE(model.get_solution_value(), first_value - TEST_EPSILON);
    });
}
// CPLEX refuses CPXlpopt while a generic callback is registered, whatever the
// callback does. Whether it fires on such a model is the solver's business.
TYPED_TEST_P(ProgressCallbackTest, solves_a_model_without_integer_variables) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x = model.add_variables(2);
        model.set_maximization();
        model.set_objective(x(0) + 2 * x(1) + 3);
        model.add_constraint(x(0) + x(1) <= 4);
        model.set_progress_callback([&](auto &) {});
        model.solve();
        ASSERT_TRUE(is<status::optimal>(model.get_status()));
        ASSERT_NEAR(model.get_solution_value(), 11, TEST_EPSILON);
    });
}

REGISTER_TYPED_TEST_SUITE_P(
    ProgressCallbackTest, reports_the_search_when_minimizing,
    reports_the_search_when_maximizing,
    no_incumbent_reads_infinite_when_minimizing,
    no_incumbent_reads_infinite_when_maximizing, interrupt_ends_the_solve,
    an_interrupt_is_for_one_solve, leaves_the_search_unchanged,
    a_moved_model_keeps_its_callback, a_second_callback_replaces_the_first,
    a_callback_set_after_a_solve_runs, a_changed_model_keeps_its_callback,
    solves_a_model_without_integer_variables);

// Apart from the suite above: COPT's callback reads a node count of 0.
template <typename T>
struct ProgressCallbackNodeCountTest : public ProgressCallbackFixture<T> {
    using typename T::model_type;
    static_assert(has_progress_callback<model_type>);
    static_assert(has_node_count<progress_callback_handle_t<model_type>>);
};
TYPED_TEST_SUITE_P(ProgressCallbackNodeCountTest);
GTEST_ALLOW_UNINSTANTIATED_PARAMETERIZED_TEST(ProgressCallbackNodeCountTest);

TYPED_TEST_P(ProgressCallbackNodeCountTest, counts_grow_up_to_the_final_one) {
    this->SkipOnLicenseError([this]() {
        auto model = this->new_model();
        TestFixture::build_large(model);
        model.set_time_limit(std::chrono::seconds(30));
        std::size_t last = 0, num_decreases = 0;
        model.set_progress_callback([&](auto & handle) {
            const std::size_t count = handle.get_node_count();
            if(count < last) ++num_decreases;
            last = count;
            if(count >= 100) handle.interrupt();
        });
        model.solve();
        ASSERT_EQ(num_decreases, 0u);
        ASSERT_GE(last, 100u);
        if(const auto final_count = TestFixture::node_count_of(model)) {
            ASSERT_LE(last, *final_count);
        }
    });
}

REGISTER_TYPED_TEST_SUITE_P(ProgressCallbackNodeCountTest,
                            counts_grow_up_to_the_final_one);

// Gurobi, CPLEX and COPT take one callback function per model, shared by the
// two callbacks; on all four that also have a native IIS routine, neither must
// reach it.
template <typename T>
struct ProgressCallbackSharingTest : public ProgressCallbackFixture<T> {
    using typename T::model_type;
    static_assert(has_progress_callback<model_type>);
    static_assert(has_candidate_solution_callback<model_type>);
    static_assert(has_iis<model_type>);

    template <typename Model, typename Candidate, typename Progress>
    static void set_both(Model & model, const bool candidate_first,
                         Candidate && candidate, Progress && progress) {
        if(candidate_first) {
            model.set_candidate_solution_callback(candidate);
            model.set_progress_callback(progress);
        } else {
            model.set_progress_callback(progress);
            model.set_candidate_solution_callback(candidate);
        }
    }
};
TYPED_TEST_SUITE_P(ProgressCallbackSharingTest);
GTEST_ALLOW_UNINSTANTIATED_PARAMETERIZED_TEST(ProgressCallbackSharingTest);

TYPED_TEST_P(ProgressCallbackSharingTest, both_callbacks_run_in_one_solve) {
    this->SkipOnLicenseError([this]() {
        for(const bool candidate_first : {true, false}) {
            auto model = this->new_model();
            TestFixture::build_small(model);
            std::size_t num_candidates = 0, num_progress = 0;
            TestFixture::set_both(
                model, candidate_first, [&](auto &) { ++num_candidates; },
                [&](auto &) { ++num_progress; });
            model.solve();
            ASSERT_TRUE(is<status::optimal>(model.get_status()));
            ASSERT_GE(num_candidates, 1u);
            ASSERT_GE(num_progress, 1u);
        }
    });
}
// A candidate's refusal the progress callback must not get in the way of, by
// lazy constraint where the handle has them, by rejection on Xpress.
TYPED_TEST_P(ProgressCallbackSharingTest, refused_candidates_stay_refused) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        using model_type = typename TestFixture::model_type;
        using candidate_handle =
            candidate_solution_callback_handle_t<model_type>;
        for(const bool candidate_first : {true, false}) {
            auto model = this->new_model();
            auto x = TestFixture::build_small(model);
            std::size_t num_progress = 0;
            TestFixture::set_both(
                model, candidate_first,
                [&, x](auto & handle) {
                    const auto solution = handle.get_solution();
                    if(solution[x(0)] + solution[x(1)] <= 1.5) return;
                    if constexpr(has_lazy_constraints<candidate_handle,
                                                      model_type>)
                        handle.add_lazy_constraint(x(0) + x(1) <= 1);
                    else
                        handle.reject_solution();
                },
                [&](auto &) { ++num_progress; });
            model.solve();
            ASSERT_TRUE(status::solution_available(model.get_status()));
            ASSERT_GE(num_progress, 1u);
            const auto solution = model.get_solution();
            ASSERT_LE(solution[x(0)] + solution[x(1)], 1 + TEST_EPSILON);
        }
    });
}
// Not attached, a callback cannot stop the routine or read its private solves:
// on CPLEX a registered one makes it fail outright.
TYPED_TEST_P(ProgressCallbackSharingTest, compute_iis_runs_neither_callback) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x = model.add_binary_variables(3);
        model.add_constraint(x(0) + x(1) + x(2) >= 2);
        model.add_constraint(x(0) + x(1) <= 0);
        std::size_t num_candidates = 0, num_progress = 0;
        model.set_candidate_solution_callback(
            [&](auto &) { ++num_candidates; });
        model.set_progress_callback([&](auto & handle) {
            ++num_progress;
            handle.interrupt();
        });
        const auto iis = model.compute_iis();
        ASSERT_TRUE(outcome_is<iis_outcome::irreducible>(iis.get_outcome()));
        ASSERT_EQ(num_candidates, 0u);
        ASSERT_EQ(num_progress, 0u);
    });
}

REGISTER_TYPED_TEST_SUITE_P(ProgressCallbackSharingTest,
                            both_callbacks_run_in_one_solve,
                            refused_candidates_stay_refused,
                            compute_iis_runs_neither_callback);

}  // namespace mippp
