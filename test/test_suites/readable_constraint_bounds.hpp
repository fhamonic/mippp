#pragma once

#include <gtest/gtest.h>

#include <ranges>

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
        ASSERT_EQ(model.get_constraint_lower_bound(c2), -model.infinity());
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
        ASSERT_EQ(model.get_constraint_upper_bound(c1), model.infinity());
        ASSERT_EQ(model.get_constraint_upper_bound(c2), 11.0);
        ASSERT_EQ(model.get_constraint_upper_bound(c3), 8.0);
        if constexpr(has_ranged_constraints<typename TestFixture::model_type>) {
            auto c4 = model.add_ranged_constraint(x1 + x2, 1.0, 3.0);
            ASSERT_EQ(model.get_constraint_upper_bound(c4), 3.0);
        }
    });
}
TYPED_TEST_P(ReadableConstraintBoundsTest, bounds_of_rows_added_in_bulk) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x1 = model.add_variable();
        auto x2 = model.add_variable();
        auto le = model.add_constraints(
            std::views::iota(0, 2), [&](int i) { return x1 + x2 <= 3 + i; });
        auto ge = model.add_constraints(
            std::views::iota(0, 2), [&](int i) { return x1 - x2 >= 1 + i; });
        auto eq = model.add_constraints(std::views::iota(0, 2), [&](int i) {
            return 2 * x1 + x2 == 5 + i;
        });
        for(int i = 0; i < 2; ++i) {
            ASSERT_EQ(model.get_constraint_lower_bound(le(i)),
                      -model.infinity());
            ASSERT_EQ(model.get_constraint_upper_bound(le(i)), 3.0 + i);
            ASSERT_EQ(model.get_constraint_lower_bound(ge(i)), 1.0 + i);
            ASSERT_EQ(model.get_constraint_upper_bound(ge(i)),
                      model.infinity());
            ASSERT_EQ(model.get_constraint_lower_bound(eq(i)), 5.0 + i);
            ASSERT_EQ(model.get_constraint_upper_bound(eq(i)), 5.0 + i);
        }
    });
}

REGISTER_TYPED_TEST_SUITE_P(ReadableConstraintBoundsTest,
                            get_constraint_lower_bound,
                            get_constraint_upper_bound,
                            bounds_of_rows_added_in_bulk);

}  // namespace mippp
