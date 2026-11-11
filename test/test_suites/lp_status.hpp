#pragma once

#include <gtest/gtest.h>

#include <vector>

#include "mippp/linear_constraint.hpp"
#include "mippp/model_concepts.hpp"

namespace mippp {

// The LPs of LpStatusTest, built here so that a suite running them under a
// setting of its own, as LpAlgorithmTest does, runs the same ones.
namespace lp_status_cases {

enum class verdict { optimal, unbounded, infeasible };

template <typename M>
struct point {
    model_variable_t<M> x, y;
};

// x <= 3, y >= 1 and x + y <= 5: bounded towards the vertex (3, 2), unbounded
// the other way
template <typename M>
point<M> build_box(M & model, bool maximize, double x_coef, double y_coef) {
    using namespace operators;
    auto x = model.add_variable({.upper_bound = 3});
    auto y = model.add_variable({.lower_bound = 1});
    if(maximize)
        model.set_maximization();
    else
        model.set_minimization();
    model.set_objective(x_coef * x + y_coef * y);
    model.add_constraint(x + y <= 5);
    return {x, y};
}

// not a huge finite bound the solver honours: GLPK solved such a column to
// 1.8e308 and reported optimal
template <has_modifiable_variable_bounds M>
void build_unbounded_after_removing_a_bound(M & model) {
    using namespace operators;
    auto x = model.add_variable({.upper_bound = 3});
    auto y = model.add_variable({.lower_bound = 1, .upper_bound = 4});
    model.set_variable_upper_bound(x, model.infinity());
    model.set_maximization();
    model.set_objective(x + y);
    model.add_constraint(x >= y);
}

// y <= 1 under y >= 2 x - 2 and y >= 3 - x, which leave y >= 4/3
template <typename M>
void build_infeasible(M & model, bool maximize) {
    using namespace operators;
    auto x = model.add_variable();
    auto y = model.add_variable({.upper_bound = 1});
    if(maximize)
        model.set_maximization();
    else
        model.set_minimization();
    model.set_objective(-2 * x + -1.5 * y);
    model.add_constraint(y >= -2 + 2 * x);
    model.add_constraint(y >= 3 - x);
}

template <typename M>
void refine(M & model) {
    if constexpr(has_refinable_lp_status<M>) {
        if(is<status::infeasible_or_unbounded>(model.get_status()))
            model.refine_lp_status();
    }
}

template <typename M>
struct solved_case {
    const char * name;
    verdict expected;
    double value;  // of an optimal case
    void (*build)(M &);
};

template <typename M>
std::vector<solved_case<M>> solved_cases() {
    using enum verdict;
    std::vector<solved_case<M>> cases{
        {"max_bounded", optimal, 9.,
         [](M & model) { build_box(model, true, 2., 1.5); }},
        {"min_bounded", optimal, -9.,
         [](M & model) { build_box(model, false, -2., -1.5); }},
        {"max_unbounded", unbounded, 0.,
         [](M & model) { build_box(model, true, -2., -1.5); }},
        {"min_unbounded", unbounded, 0.,
         [](M & model) { build_box(model, false, 2., 1.5); }},
        {"max_infeasible", infeasible, 0.,
         [](M & model) { build_infeasible(model, true); }},
        {"min_infeasible", infeasible, 0.,
         [](M & model) { build_infeasible(model, false); }}};
    if constexpr(has_modifiable_variable_bounds<M>)
        cases.push_back(
            {"unbounded_after_removing_a_bound", unbounded, 0.,
             [](M & model) { build_unbounded_after_removing_a_bound(model); }});
    return cases;
}

}  // namespace lp_status_cases

template <typename T>
struct LpStatusTest : public T {
    using typename T::model_type;
    static_assert(lp_model<model_type>);
};
TYPED_TEST_SUITE_P(LpStatusTest);
GTEST_ALLOW_UNINSTANTIATED_PARAMETERIZED_TEST(LpStatusTest);

TYPED_TEST_P(LpStatusTest, not_solved) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        ASSERT_TRUE(is<status::unknown>(model.get_status()));
    });
}
TYPED_TEST_P(LpStatusTest, max_bounded) {
    this->SkipOnLicenseError([this]() {
        auto model = this->new_model();
        const auto [x, y] = lp_status_cases::build_box(model, true, 2., 1.5);
        model.solve();
        ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
        ASSERT_DOUBLE_EQ(model.get_solution_value(), 9.0);
        auto solution = model.get_solution();
        ASSERT_DOUBLE_EQ(solution[x], 3.0);
        ASSERT_DOUBLE_EQ(solution[y], 2.0);
    });
}
TYPED_TEST_P(LpStatusTest, min_bounded) {
    this->SkipOnLicenseError([this]() {
        auto model = this->new_model();
        const auto [x, y] = lp_status_cases::build_box(model, false, -2., -1.5);
        model.solve();
        ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
        ASSERT_DOUBLE_EQ(model.get_solution_value(), -9.0);
        auto solution = model.get_solution();
        ASSERT_DOUBLE_EQ(solution[x], 3.0);
        ASSERT_DOUBLE_EQ(solution[y], 2.0);
    });
}
TYPED_TEST_P(LpStatusTest, max_unbounded) {
    this->SkipOnLicenseError([this]() {
        auto model = this->new_model();
        lp_status_cases::build_box(model, true, -2., -1.5);
        model.solve();
        lp_status_cases::refine(model);
        ASSERT_TRUE(is_a<status::unbounded>(model.get_status()));
    });
}
TYPED_TEST_P(LpStatusTest, min_unbounded) {
    this->SkipOnLicenseError([this]() {
        auto model = this->new_model();
        lp_status_cases::build_box(model, false, 2., 1.5);
        model.solve();
        lp_status_cases::refine(model);
        ASSERT_TRUE(is_a<status::unbounded>(model.get_status()));
    });
}
TYPED_TEST_P(LpStatusTest, unbounded_after_removing_a_bound) {
    this->SkipOnLicenseError([this]() {
        auto model = this->new_model();
        if constexpr(has_modifiable_variable_bounds<decltype(model)>) {
            lp_status_cases::build_unbounded_after_removing_a_bound(model);
            model.solve();
            lp_status_cases::refine(model);
            ASSERT_TRUE(is_a<status::unbounded>(model.get_status()));
        }
    });
}
TYPED_TEST_P(LpStatusTest, max_infeasible) {
    this->SkipOnLicenseError([this]() {
        auto model = this->new_model();
        lp_status_cases::build_infeasible(model, true);
        model.solve();
        lp_status_cases::refine(model);
        ASSERT_TRUE(is_a<status::infeasible>(model.get_status()));
    });
}
TYPED_TEST_P(LpStatusTest, min_infeasible) {
    this->SkipOnLicenseError([this]() {
        auto model = this->new_model();
        lp_status_cases::build_infeasible(model, false);
        model.solve();
        lp_status_cases::refine(model);
        ASSERT_TRUE(is_a<status::infeasible>(model.get_status()));
    });
}

// Without columns there is nothing to optimize, yet a row can be violated:
// a backend that skips its solver must not report such a model optimal.
TYPED_TEST_P(LpStatusTest, constant_row_without_variables_is_not_optimal) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        constexpr auto & no_terms =
            empty_linear_expression<model_variable_t<decltype(model)>, double>;
        model.add_constraint(no_terms >= 1.0);
        model.solve();
        ASSERT_TRUE(is_a<status::infeasible>(model.get_status()) ||
                    is<status::unknown>(model.get_status()));
    });
}

REGISTER_TYPED_TEST_SUITE_P(LpStatusTest, not_solved, max_bounded, min_bounded,
                            max_unbounded, min_unbounded,
                            unbounded_after_removing_a_bound, max_infeasible,
                            min_infeasible,
                            constant_row_without_variables_is_not_optimal);

}  // namespace mippp
