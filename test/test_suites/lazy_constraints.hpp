#pragma once

#undef NDEBUG
#include <gtest/gtest.h>

#include "mippp/linear_constraint.hpp"
#include "mippp/model_concepts.hpp"

namespace mippp {

template <typename T>
struct LazyConstraintsTest : public T {
    using typename T::model_type;
    static_assert(milp_model<model_type>);
    static_assert(has_candidate_solution_callback<model_type>);
    static_assert(
        has_lazy_constraints<candidate_solution_callback_handle_t<model_type>,
                             model_type>);
};
TYPED_TEST_SUITE_P(LazyConstraintsTest);
GTEST_ALLOW_UNINSTANTIATED_PARAMETERIZED_TEST(LazyConstraintsTest);

namespace detail {
template <typename Test, typename Separate>
void lazy_constraints_cut_off_candidates(Test & test, Separate && separate) {
    using namespace operators;
    auto model = test.new_model();
    auto a = model.add_binary_variable();
    auto b = model.add_binary_variable();
    auto c = model.add_binary_variable();
    model.set_maximization();
    model.set_objective(3 * a + 2 * b + c);

    int num_cuts = 0;
    model.set_candidate_solution_callback([&](auto & handle) {
        auto solution = handle.get_solution();
        if(solution[a] + solution[b] + solution[c] <= 2 + TEST_EPSILON) return;
        ++num_cuts;
        separate(handle, a + b + c <= 2);
    });
    model.solve();

    ASSERT_GE(num_cuts, 1);
    ASSERT_NEAR(model.get_solution_value(), 5, TEST_EPSILON);
    auto solution = model.get_solution();
    ASSERT_NEAR(solution[a], 1, TEST_EPSILON);
    ASSERT_NEAR(solution[b], 1, TEST_EPSILON);
    ASSERT_NEAR(solution[c], 0, TEST_EPSILON);
}
}  // namespace detail

TYPED_TEST_P(LazyConstraintsTest, cuts_off_candidates) {
    this->SkipOnLicenseError([this]() {
        detail::lazy_constraints_cut_off_candidates(
            *this,
            [](auto & handle, auto && lc) { handle.add_lazy_constraint(lc); });
    });
}

TYPED_TEST_P(LazyConstraintsTest, cuts_off_candidates_distinct_variables) {
    this->SkipOnLicenseError([this]() {
        detail::lazy_constraints_cut_off_candidates(
            *this, [](auto & handle, auto && lc) {
                handle.add_lazy_constraint(distinct_variables, lc);
            });
    });
}

REGISTER_TYPED_TEST_SUITE_P(LazyConstraintsTest, cuts_off_candidates,
                            cuts_off_candidates_distinct_variables);

}  // namespace mippp
