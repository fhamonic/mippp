#pragma once

#undef NDEBUG
#include <gtest/gtest.h>

#include "mippp/linear_constraint.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/quadratic_expression.hpp"

#include "assert_helper.hpp"

namespace mippp {

template <typename T>
struct ReadableQuadraticObjectiveTest : public T {
    using typename T::model_type;
    static_assert(has_readable_quadratic_objective<model_type>);
};
TYPED_TEST_SUITE_P(ReadableQuadraticObjectiveTest);
GTEST_ALLOW_UNINSTANTIATED_PARAMETERIZED_TEST(ReadableQuadraticObjectiveTest);

TYPED_TEST_P(ReadableQuadraticObjectiveTest, get_quadratic_objective) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x = model.add_variable();
        auto y = model.add_variable();
        auto z = model.add_variable();
        model.add_constraint(x + y + z <= 10);
        model.set_quadratic_objective(2 * x * x + 3 * x * y + 4 * z * y -
                                      5 * y + z + 7);
        ASSERT_QUAD_EXPR(model.get_quadratic_objective(),
                         {{x, x, 2.0}, {x, y, 3.0}, {y, z, 4.0}},
                         {{x, 0.0}, {y, -5.0}, {z, 1.0}}, 7.0);
        // get_objective() reads the linear part only
        ASSERT_LIN_EXPR(model.get_objective(), {{x, 0.0}, {y, -5.0}, {z, 1.0}},
                        7.0);
    });
}
TYPED_TEST_P(ReadableQuadraticObjectiveTest, linear_objective_reads_back) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x = model.add_variable();
        auto y = model.add_variable();
        model.set_quadratic_objective(x * y + x);
        model.set_objective(2 * x - y + 1);
        ASSERT_QUAD_EXPR(model.get_quadratic_objective(), {},
                         {{x, 2.0}, {y, -1.0}}, 1.0);
    });
}

REGISTER_TYPED_TEST_SUITE_P(ReadableQuadraticObjectiveTest,
                            get_quadratic_objective,
                            linear_objective_reads_back);

}  // namespace mippp
