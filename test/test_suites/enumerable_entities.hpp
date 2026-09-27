#pragma once

#include <gtest/gtest.h>

#include <cstddef>
#include <ranges>
#include <vector>

#include "mippp/linear_constraint.hpp"
#include "mippp/model_concepts.hpp"

#include "assert_helper.hpp"

namespace mippp {

// The cases after remove_variable and id recycling are in RemoveVariableTest,
// which runs exactly where has_remove_variable holds.
template <typename T>
struct EnumerableEntitiesTest : public T {
    using typename T::model_type;
    static_assert(has_enumerable_variables<model_type>);
    static_assert(has_enumerable_constraints<model_type>);
};
TYPED_TEST_SUITE_P(EnumerableEntitiesTest);
GTEST_ALLOW_UNINSTANTIATED_PARAMETERIZED_TEST(EnumerableEntitiesTest);

TYPED_TEST_P(EnumerableEntitiesTest, empty_model_lists_nothing) {
    this->SkipOnLicenseError([this]() {
        auto model = this->new_model();
        EXPECT_TRUE(std::ranges::empty(model.variables()));
        EXPECT_TRUE(std::ranges::empty(model.constraints()));
    });
}

TYPED_TEST_P(EnumerableEntitiesTest, lists_every_addition_in_id_order) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        using M = typename TestFixture::model_type;
        auto model = this->new_model();
        std::vector<model_variable_t<M>> added;
        added.push_back(model.add_variable());
        for(auto v : model.add_variables(3)) added.push_back(v);
        for(auto v : model.add_variables(2, [](int i) { return i; }))
            added.push_back(v);
        for(auto v : model.add_variables(std::views::iota(0, 2)))
            added.push_back(v);
        if constexpr(has_named_variables<M>)
            added.push_back(model.add_named_variable("n"));
        if constexpr(milp_model<M>) {
            added.push_back(model.add_integer_variable());
            for(auto v : model.add_binary_variables(2)) added.push_back(v);
        }
        std::vector<model_constraint_t<M>> rows;
        rows.push_back(model.add_constraint(added[0] + added[1] >= 1));
        for(auto c : model.add_constraints(std::views::iota(0, 3), [&](int i) {
                return added[static_cast<std::size_t>(i)] <= 4;
            }))
            rows.push_back(c);
        if constexpr(has_ranged_constraints<M>)
            rows.push_back(model.add_ranged_constraint(added[2], -1.0, 1.0));
        if constexpr(has_column_generation<M>)
            added.push_back(model.add_column({{rows[0], 1.0}}));
        EXPECT_ENUMERATED_VARIABLES(model, added);
        EXPECT_ENUMERATED_CONSTRAINTS(model, rows);
    });
}

TYPED_TEST_P(EnumerableEntitiesTest, snapshot_ignores_later_additions) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        auto model = this->new_model();
        auto x = model.add_variable();
        auto c = model.add_constraint(x >= 0);
        const auto vars = model.variables();
        const auto rows = model.constraints();
        auto y = model.add_variable();
        auto d = model.add_constraint(y >= 0);
        EXPECT_EQ(entity_uids(vars), std::vector<std::size_t>{x.uid()});
        EXPECT_EQ(entity_uids(rows), std::vector<std::size_t>{c.uid()});
        EXPECT_ENUMERATED_VARIABLES(model, std::vector{x, y});
        EXPECT_ENUMERATED_CONSTRAINTS(model, std::vector{c, d});

        const auto outliving = [this]() {
            auto short_lived = this->new_model();
            short_lived.add_variables(2);
            return short_lived.variables();
        }();
        EXPECT_EQ(entity_uids(outliving), (std::vector<std::size_t>{0, 1}));
    });
}

TYPED_TEST_P(EnumerableEntitiesTest, listed_handles_reach_their_entities) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        using M = typename TestFixture::model_type;
        auto model = this->new_model();
        auto xs = model.add_variables(4);
        for(auto v : model.variables())
            model.add_constraint(v >= static_cast<double>(v.uid()) + 1.0);
        if constexpr(has_readable_constraint_rhs<M>) {
            std::vector<double> rhs;
            for(auto c : model.constraints())
                rhs.push_back(model.get_constraint_rhs(c));
            EXPECT_EQ(rhs, (std::vector<double>{1.0, 2.0, 3.0, 4.0}));
        }
        if constexpr(has_modifiable_variable_bounds<M> &&
                     has_readable_variable_bounds<M>) {
            for(auto v : model.variables())
                model.set_variable_upper_bound(
                    v, 10.0 + static_cast<double>(v.uid()));
            for(auto x : xs)
                EXPECT_EQ(model.get_variable_upper_bound(x),
                          10.0 + static_cast<double>(x.uid()));
        }
        model.set_minimization();
        model.set_objective(xsum(model.variables()));
        model.solve();
        ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
        auto solution = model.get_solution();
        for(auto x : xs)
            EXPECT_NEAR(solution[x], static_cast<double>(x.uid()) + 1.0,
                        TEST_EPSILON);
    });
}

REGISTER_TYPED_TEST_SUITE_P(EnumerableEntitiesTest, empty_model_lists_nothing,
                            lists_every_addition_in_id_order,
                            snapshot_ignores_later_additions,
                            listed_handles_reach_their_entities);

}  // namespace mippp
