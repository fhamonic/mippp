#pragma once
#undef NDEBUG
#include <gtest/gtest.h>

#include "mippp/linear_constraint.hpp"
#include "mippp/model_concepts.hpp"

#include "assert_helper.hpp"

namespace fhamonic::mippp {

template <typename T>
struct ReadableConstraintsTest : public T {
    using typename T::model_type;
    static_assert(has_readable_constraints<model_type>);
};
TYPED_TEST_SUITE_P(ReadableConstraintsTest);

TYPED_TEST_P(ReadableConstraintsTest, get_constraint_lhs) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x1 = model.add_variable();
        auto x2 = model.add_variable();
        auto x3 = model.add_variable();
        auto c1 = model.add_constraint(2 * x1 + 2 * x2 - x3 >= 5);
        auto c2 = model.add_constraint(4 * x1 + x2 + 2 * x3 <= 11);
        auto c3 = model.add_constraint(3 * x1 + 4 * x2 + 2 * x3 == 8);
        ASSERT_LIN_TERMS(model.get_constraint_lhs(c1),
                         {{x1, 2.0}, {x2, 2.0}, {x3, -1.0}});
        ASSERT_LIN_TERMS(model.get_constraint_lhs(c2),
                         {{x1, 4.0}, {x2, 1.0}, {x3, 2.0}});
        ASSERT_LIN_TERMS(model.get_constraint_lhs(c3),
                         {{x1, 3.0}, {x2, 4.0}, {x3, 2.0}});
    });
}
TYPED_TEST_P(ReadableConstraintsTest, get_constraint_sense) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x1 = model.add_variable();
        auto x2 = model.add_variable();
        auto x3 = model.add_variable();
        auto c1 = model.add_constraint(2 * x1 + 2 * x2 - x3 >= 5);
        auto c2 = model.add_constraint(4 * x1 + x2 + 2 * x3 <= 11);
        auto c3 = model.add_constraint(3 * x1 + 4 * x2 + 2 * x3 == 8);
        ASSERT_EQ(model.get_constraint_sense(c1),
                  constraint_sense::greater_equal);
        ASSERT_EQ(model.get_constraint_sense(c2), constraint_sense::less_equal);
        ASSERT_EQ(model.get_constraint_sense(c3), constraint_sense::equal);
    });
}
TYPED_TEST_P(ReadableConstraintsTest, get_constraint_rhs) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x1 = model.add_variable();
        auto x2 = model.add_variable();
        auto x3 = model.add_variable();
        auto c1 = model.add_constraint(2 * x1 + 2 * x2 - x3 >= 5);
        auto c2 = model.add_constraint(4 * x1 + x2 + 2 * x3 <= 11);
        auto c3 = model.add_constraint(3 * x1 + 4 * x2 + 2 * x3 == 8);
        ASSERT_EQ(model.get_constraint_rhs(c1), 5);
        ASSERT_EQ(model.get_constraint_rhs(c2), 11);
        ASSERT_EQ(model.get_constraint_rhs(c3), 8);
    });
}
TYPED_TEST_P(ReadableConstraintsTest, get_constraint) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x1 = model.add_variable();
        auto x2 = model.add_variable();
        auto x3 = model.add_variable();
        auto c1 = model.add_constraint(2 * x1 + 2 * x2 - x3 >= 5);
        auto c2 = model.add_constraint(4 * x1 + x2 + 2 * x3 <= 11);
        auto c3 = model.add_constraint(3 * x1 + 4 * x2 + 2 * x3 == 8);
        ASSERT_CONSTRAINT(model.get_constraint(c1),
                          {{x1, 2.0}, {x2, 2.0}, {x3, -1.0}},
                          constraint_sense::greater_equal, 5);
        ASSERT_CONSTRAINT(model.get_constraint(c2),
                          {{x1, 4.0}, {x2, 1.0}, {x3, 2.0}},
                          constraint_sense::less_equal, 11);
        ASSERT_CONSTRAINT(model.get_constraint(c3),
                          {{x1, 3.0}, {x2, 4.0}, {x3, 2.0}},
                          constraint_sense::equal, 8);
    });
}

REGISTER_TYPED_TEST_SUITE_P(ReadableConstraintsTest, get_constraint_lhs,
                            get_constraint_sense, get_constraint_rhs,
                            get_constraint);

}  // namespace fhamonic::mippp