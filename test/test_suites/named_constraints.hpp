#pragma once

#undef NDEBUG
#include <gtest/gtest.h>

#include <format>
#include <ranges>
#include <string>
#include <vector>

#include "mippp/linear_constraint.hpp"
#include "mippp/model_concepts.hpp"

namespace mippp {

template <typename T>
struct NamedConstraintsTest : public T {
    using typename T::model_type;
    static_assert(has_named_constraints<model_type>);
};
TYPED_TEST_SUITE_P(NamedConstraintsTest);
GTEST_ALLOW_UNINSTANTIATED_PARAMETERIZED_TEST(NamedConstraintsTest);

TYPED_TEST_P(NamedConstraintsTest, set_constraint_name) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x = model.add_variable();
        auto c1 = model.add_constraint(x <= 1);
        auto c2 = model.add_constraint(x >= 0);
        model.set_constraint_name(c1, "C1");
        model.set_constraint_name(c2, "C2");
        ASSERT_EQ(model.get_constraint_name(c1), "C1");
        ASSERT_EQ(model.get_constraint_name(c2), "C2");
        model.set_constraint_name(
            c2, "LongNameSoThatSmallStringOptimizationIsNotUsed");
        ASSERT_EQ(model.get_constraint_name(c2),
                  "LongNameSoThatSmallStringOptimizationIsNotUsed");
    });
}
TYPED_TEST_P(NamedConstraintsTest, add_named_constraints) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x = model.add_variable();
        auto y = model.add_variable();
        model.add_constraint(x + y <= 4);
        auto rows = model.add_constraints(
            named(std::views::iota(0, 3),
                  [](int i) { return std::format("row_{}", i); }),
            [&](int i) { return i * x + y <= 2; });
        ASSERT_EQ(model.num_constraints(), 4);
        ASSERT_EQ(rows(0).id(), 1);
        ASSERT_EQ(rows(2).id(), 3);
        ASSERT_THROW(rows(3), std::out_of_range);
        ASSERT_EQ(model.get_constraint_name(rows(0)), "row_0");
        ASSERT_EQ(model.get_constraint_name(rows(1)), "row_1");
        ASSERT_EQ(model.get_constraint_name(rows(2)), "row_2");
    });
}
TYPED_TEST_P(NamedConstraintsTest, add_indexed_named_constraints) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x = model.add_variable();
        std::vector<int> keys = {5, 2};
        auto rows = model.add_constraints(
            indexed_named(
                keys, [](int k) { return k; },
                [](int k) { return "K" + std::to_string(k); }),
            [&](int k) { return k * x <= 1; });
        ASSERT_EQ(model.num_constraints(), 2);
        ASSERT_EQ(rows(5).id(), 0);
        ASSERT_EQ(rows(2).id(), 1);
        ASSERT_THROW(rows(3), std::out_of_range);
        ASSERT_EQ(model.get_constraint_name(rows(5)), "K5");
        ASSERT_EQ(model.get_constraint_name(rows(2)), "K2");
    });
}

REGISTER_TYPED_TEST_SUITE_P(NamedConstraintsTest, set_constraint_name,
                            add_named_constraints,
                            add_indexed_named_constraints);

}  // namespace mippp
