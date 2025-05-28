#pragma once
#undef NDEBUG
#include <gtest/gtest.h>

#include "mippp/linear_constraint.hpp"
#include "mippp/model_concepts.hpp"

namespace fhamonic::mippp {

template <typename T>
struct ColumGenerationTest : public T {
    using typename T::model_type;
    static_assert(has_column_generation<model_type>);
};
TYPED_TEST_SUITE_P(ColumGenerationTest);

TYPED_TEST_P(ColumGenerationTest, add_column_entries) {
    using namespace operators;
    auto model = this->construct_model();
    auto x1 = model.add_variable();
    auto c1 = model.add_constraint(2 * x1 <= 5);
    auto x2 = model.add_column({{c1, 3.0}});
    auto c2 = model.add_constraint(4 * x1 + x2 <= 11);
    std::vector<std::pair<decltype(c1), double>> col = {
        std::make_pair(c1, 1.0), std::make_pair(c2, 2.0)};
    auto x3 = model.add_column(col);
    auto c3 = model.add_constraint(3 * x1 + 4 * x2 + 2 * x3 <= 8);
    model.set_maximization();
    model.set_objective(5 * x1 + 4 * x2 + 3 * x3);
    model.solve();
    ASSERT_NEAR(model.get_solution_value(), 13.0, TEST_EPSILON);
    auto solution = model.get_solution();
    ASSERT_NEAR(solution[x1], 2.0, TEST_EPSILON);
    ASSERT_NEAR(solution[x2], 0.0, TEST_EPSILON);
    ASSERT_NEAR(solution[x3], 1.0, TEST_EPSILON);
}

REGISTER_TYPED_TEST_SUITE_P(ColumGenerationTest, add_column_entries);

}  // namespace fhamonic::mippp