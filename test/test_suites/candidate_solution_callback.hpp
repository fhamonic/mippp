#pragma once

#include <gtest/gtest.h>
#include <cmath>

#include "mippp/linear_constraint.hpp"
#include "mippp/model_concepts.hpp"

namespace mippp {

template <typename T>
struct CandidateSolutionCallbackTest : public T {
    using typename T::model_type;
    static_assert(milp_model<model_type>);
    static_assert(has_candidate_solution_callback<model_type>);
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

REGISTER_TYPED_TEST_SUITE_P(CandidateSolutionCallbackTest,
                            reads_every_candidate, reports_candidate_value);

}  // namespace mippp
