#pragma once

#undef NDEBUG
#include <gtest/gtest.h>

#include "mippp/model_concepts.hpp"

namespace mippp {

template <typename T>
struct ReadableConstraintBoundsTest : public T {
    using typename T::model_type;
    static_assert(has_readable_constraint_bounds<model_type>);
};
TYPED_TEST_SUITE_P(ReadableConstraintBoundsTest);
GTEST_ALLOW_UNINSTANTIATED_PARAMETERIZED_TEST(ReadableConstraintBoundsTest);

TYPED_TEST_P(ReadableConstraintBoundsTest, get_constraint_lower_bound) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x1 = model.add_variable();
        auto x2 = model.add_variable();
        auto c1 = model.add_constraint(2 * x1 + x2 >= 5);
        auto c2 = model.add_constraint(x1 + 2 * x2 <= 11);
        auto c3 = model.add_constraint(x1 + x2 == 8);
        ASSERT_EQ(model.get_constraint_lower_bound(c1), 5.0);
        ASSERT_LE(model.get_constraint_lower_bound(c2), -TEST_INFINITY);
        ASSERT_EQ(model.get_constraint_lower_bound(c3), 8.0);
        if constexpr(has_ranged_constraints<typename TestFixture::model_type>) {
            auto c4 = model.add_ranged_constraint(x1 + x2, 1.0, 3.0);
            ASSERT_EQ(model.get_constraint_lower_bound(c4), 1.0);
        }
    });
}
TYPED_TEST_P(ReadableConstraintBoundsTest, get_constraint_upper_bound) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x1 = model.add_variable();
        auto x2 = model.add_variable();
        auto c1 = model.add_constraint(2 * x1 + x2 >= 5);
        auto c2 = model.add_constraint(x1 + 2 * x2 <= 11);
        auto c3 = model.add_constraint(x1 + x2 == 8);
        ASSERT_GE(model.get_constraint_upper_bound(c1), TEST_INFINITY);
        ASSERT_EQ(model.get_constraint_upper_bound(c2), 11.0);
        ASSERT_EQ(model.get_constraint_upper_bound(c3), 8.0);
        if constexpr(has_ranged_constraints<typename TestFixture::model_type>) {
            auto c4 = model.add_ranged_constraint(x1 + x2, 1.0, 3.0);
            ASSERT_EQ(model.get_constraint_upper_bound(c4), 3.0);
        }
    });
}

REGISTER_TYPED_TEST_SUITE_P(ReadableConstraintBoundsTest,
                            get_constraint_lower_bound,
                            get_constraint_upper_bound);

}  // namespace mippp
