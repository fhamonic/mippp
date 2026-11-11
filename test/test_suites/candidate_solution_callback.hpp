#pragma once

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <utility>
#include <vector>

#include "mippp/linear_constraint.hpp"
#include "mippp/model_concepts.hpp"

namespace mippp {

template <typename T>
struct CandidateSolutionCallbackTest : public T {
    using typename T::model_type;
    static_assert(milp_model<model_type>);
    static_assert(has_candidate_solution_callback<model_type>);
    static_assert(
        !has_enumerable_variables<
            candidate_solution_callback_handle_t<model_type>, model_type>);
    static_assert(
        !has_enumerable_constraints<
            candidate_solution_callback_handle_t<model_type>, model_type>);
};
TYPED_TEST_SUITE_P(CandidateSolutionCallbackTest);
GTEST_ALLOW_UNINSTANTIATED_PARAMETERIZED_TEST(CandidateSolutionCallbackTest);

TYPED_TEST_P(CandidateSolutionCallbackTest, reads_every_candidate) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto a = model.add_binary_variable();
        auto b = model.add_binary_variable();
        auto c = model.add_binary_variable();
        model.set_maximization();
        model.set_objective(3 * a + 2 * b + c);
        model.add_constraint(a + b + c <= 2);

        int num_candidates = 0;
        int num_invalid = 0;
        model.set_candidate_solution_callback([&](auto & handle) {
            ++num_candidates;
            auto solution = handle.get_solution();
            for(auto x : {a, b, c}) {
                if(solution[x] < -TEST_EPSILON ||
                   solution[x] > 1 + TEST_EPSILON)
                    ++num_invalid;
            }
            if(solution[a] + solution[b] + solution[c] > 2 + TEST_EPSILON)
                ++num_invalid;
        });
        model.solve();

        ASSERT_GE(num_candidates, 1);
        ASSERT_EQ(num_invalid, 0);
        ASSERT_NEAR(model.get_solution_value(), 5, TEST_EPSILON);
    });
}

TYPED_TEST_P(CandidateSolutionCallbackTest, reports_candidate_value) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto a = model.add_binary_variable();
        auto b = model.add_binary_variable();
        auto c = model.add_binary_variable();
        model.set_maximization();
        model.set_objective(3 * a + 2 * b + c + 10);
        model.add_constraint(a + b + c <= 2);

        int num_candidates = 0;
        int num_invalid = 0;
        model.set_candidate_solution_callback([&](auto & handle) {
            ++num_candidates;
            auto solution = handle.get_solution();
            const double expected =
                3 * solution[a] + 2 * solution[b] + solution[c] + 10;
            if(std::fabs(handle.get_solution_value() - expected) > TEST_EPSILON)
                ++num_invalid;
        });
        model.solve();

        ASSERT_GE(num_candidates, 1);
        ASSERT_EQ(num_invalid, 0);
        ASSERT_NEAR(model.get_solution_value(), 15, TEST_EPSILON);
    });
}

// Registered by the setter, the native user data would point at the moved-from
// model.
TYPED_TEST_P(CandidateSolutionCallbackTest, a_moved_model_keeps_its_callback) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto a = model.add_binary_variable();
        auto b = model.add_binary_variable();
        model.set_maximization();
        model.set_objective(3 * a + 2 * b);
        model.add_constraint(a + b <= 1);
        int num_candidates = 0;
        model.set_candidate_solution_callback(
            [&](auto &) { ++num_candidates; });
        auto moved = std::move(model);
        moved.solve();
        ASSERT_GE(num_candidates, 1);
        ASSERT_NEAR(moved.get_solution_value(), 3, TEST_EPSILON);
    });
}
TYPED_TEST_P(CandidateSolutionCallbackTest,
             a_second_callback_replaces_the_first) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto a = model.add_binary_variable();
        auto b = model.add_binary_variable();
        model.set_maximization();
        model.set_objective(3 * a + 2 * b);
        model.add_constraint(a + b <= 1);
        int first = 0, second = 0;
        model.set_candidate_solution_callback([&](auto &) { ++first; });
        model.set_candidate_solution_callback([&](auto &) { ++second; });
        model.solve();
        ASSERT_EQ(first, 0);
        ASSERT_GE(second, 1);
    });
}

// A separator may keep a candidate, to compare the next one with it. A point
// reading through the handle would then read a frame that is gone.
TYPED_TEST_P(CandidateSolutionCallbackTest, a_kept_candidate_stays_readable) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        using handle_type = candidate_solution_callback_handle_t<
            typename TestFixture::model_type>;
        using point_type =
            decltype(std::declval<handle_type &>().get_solution());
        auto model = this->new_model();
        auto a = model.add_binary_variable();
        auto b = model.add_binary_variable();
        auto c = model.add_binary_variable();
        model.set_maximization();
        model.set_objective(3 * a + 2 * b + c);
        model.add_constraint(a + b + c <= 2);

        struct kept_candidate {
            point_type point;
            std::array<double, 3> values;
        };
        std::vector<kept_candidate> candidates;
        model.set_candidate_solution_callback([&](auto & handle) {
            auto point = handle.get_solution();
            const std::array values = {point[a], point[b], point[c]};
            candidates.emplace_back(std::move(point), values);
        });
        model.solve();

        ASSERT_FALSE(candidates.empty());
        for(auto & [point, values] : candidates) {
            ASSERT_DOUBLE_EQ(point[a], values[0]);
            ASSERT_DOUBLE_EQ(point[b], values[1]);
            ASSERT_DOUBLE_EQ(point[c], values[2]);
        }
    });
}

// The point is the callback's own copy: writing it corrects a candidate
// before it is handed back, and leaves the solver's candidate as it was.
TYPED_TEST_P(CandidateSolutionCallbackTest, the_candidate_can_be_written) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto a = model.add_binary_variable();
        auto b = model.add_binary_variable();
        auto c = model.add_binary_variable();
        model.set_maximization();
        model.set_objective(3 * a + 2 * b + c);
        model.add_constraint(a + b + c <= 2);

        int num_candidates = 0;
        int num_invalid = 0;
        model.set_candidate_solution_callback([&](auto & handle) {
            ++num_candidates;
            auto point = handle.get_solution();
            const double b_value = point[b];
            point[a] = 0.25;
            if(std::fabs(point[a] - 0.25) > TEST_EPSILON ||
               std::fabs(point[b] - b_value) > TEST_EPSILON)
                ++num_invalid;
        });
        model.solve();

        ASSERT_GE(num_candidates, 1);
        ASSERT_EQ(num_invalid, 0);
        ASSERT_NEAR(model.get_solution_value(), 5, TEST_EPSILON);
        ASSERT_NEAR(model.get_solution()[a], 1, TEST_EPSILON);
    });
}

REGISTER_TYPED_TEST_SUITE_P(CandidateSolutionCallbackTest,
                            reads_every_candidate, reports_candidate_value,
                            a_moved_model_keeps_its_callback,
                            a_second_callback_replaces_the_first,
                            a_kept_candidate_stays_readable,
                            the_candidate_can_be_written);

}  // namespace mippp
