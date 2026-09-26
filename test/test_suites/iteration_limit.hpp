#pragma once

#include <gtest/gtest.h>

#include <cstddef>
#include <limits>
#include <random>
#include <ranges>
#include <vector>

#include "mippp/linear_constraint.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/quadratic_expression.hpp"

namespace mippp {

template <typename T>
struct IterationLimitTest : public T {
    using typename T::model_type;
    static_assert(lp_model<model_type>);
    static_assert(has_iteration_limit<model_type>);

    enum class objective { linear, quadratic };

    // Dense rows of positive coefficients over x >= 0: bounded, left whole by
    // presolve, and many iterations away from optimal. The quadratic
    // objective runs the QP solver of a QP model rather than its simplex
    // method; its minimum, x = 1, lies inside the rows, away from the bounds
    // the solver starts from. HiGHS 1.10 fails on minima the rows cut off.
    static void build(model_type & model, objective kind) {
        using namespace operators;
        constexpr std::size_t size = 200;
        std::mt19937 rng(42);
        std::uniform_int_distribution<int> coef(1, 1000);
        auto columns = std::views::iota(std::size_t{0}, size);
        auto x = model.add_variables(size);
        std::vector<int> row(size);
        for(int & a : row) a = coef(rng);
        if constexpr(qp_model<model_type>) {
            if(kind == objective::quadratic) {
                model.set_minimization();
                model.set_quadratic_objective(
                    xsum(columns, [&](auto j) { return square(x(j) - 1.0); }));
            }
        }
        if(kind == objective::linear) {
            model.set_maximization();
            model.set_objective(
                xsum(columns, [&](auto j) { return row[j] * x(j); }));
        }
        for(std::size_t i = 0; i < size; ++i) {
            for(int & a : row) a = coef(rng);
            model.add_constraint(xsum(columns, [&](auto j) {
                                     return row[j] * x(j);
                                 }) <= 1000.0 * static_cast<double>(size));
        }
    }
};
TYPED_TEST_SUITE_P(IterationLimitTest);
GTEST_ALLOW_UNINSTANTIATED_PARAMETERIZED_TEST(IterationLimitTest);

TYPED_TEST_P(IterationLimitTest, set_get_iteration_limit) {
    this->SkipOnLicenseError([this]() {
        auto model = this->new_model();
        for(std::size_t n : {1u, 7u, 1000u, 123456u}) {
            model.set_iteration_limit(n);
            ASSERT_EQ(model.get_iteration_limit(), n);
        }
    });
}

// Neither the default, read back and set again, nor a limit beyond what the
// solver can represent may stop a solve.
TYPED_TEST_P(IterationLimitTest, unlimited_by_default) {
    using objective = typename TestFixture::objective;
    this->SkipOnLicenseError([this]() {
        for(const bool beyond_range : {false, true}) {
            auto model = this->new_model();
            TestFixture::build(model, objective::linear);
            model.set_iteration_limit(
                beyond_range ? std::numeric_limits<std::size_t>::max()
                             : model.get_iteration_limit());
            model.solve();
            ASSERT_TRUE(is_a<status::optimal>(model.get_status()))
                << (beyond_range ? "with the largest std::size_t"
                                 : "with the default read back");
        }
    });
}

// Resolving to optimal once the limit is lifted shows that the limit, not the
// instance, stopped the first solve.
TYPED_TEST_P(IterationLimitTest, stops_and_resumes) {
    using model_type = typename TestFixture::model_type;
    using objective = typename TestFixture::objective;
    this->SkipOnLicenseError([this]() {
        auto stop_and_resume = [this](objective kind) {
            auto model = this->new_model();
            TestFixture::build(model, kind);
            const std::size_t unlimited = model.get_iteration_limit();
            const char * name =
                kind == objective::linear ? "linear" : "quadratic";
            model.set_iteration_limit(1);
            model.solve();
            ASSERT_TRUE(is_a<status::iteration_limit>(model.get_status()))
                << "with a " << name << " objective";
            model.set_iteration_limit(unlimited);
            model.solve();
            ASSERT_TRUE(is_a<status::optimal>(model.get_status()))
                << "with a " << name << " objective";
        };
        stop_and_resume(objective::linear);
        if constexpr(qp_model<model_type>)
            stop_and_resume(objective::quadratic);
    });
}

REGISTER_TYPED_TEST_SUITE_P(IterationLimitTest, set_get_iteration_limit,
                            unlimited_by_default, stops_and_resumes);

}  // namespace mippp
