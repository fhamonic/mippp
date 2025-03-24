#include <gtest/gtest.h>

#include "assert_helper.hpp"

#include "mippp/model_concepts.hpp"

#include "mippp/model/cbc_milp_model.hpp"
#include "mippp/model/clp_lp_model.hpp"
#include "mippp/model/glpk_lp_model.hpp"
#include "mippp/model/grb_lp_model.hpp"
#include "mippp/model/grb_milp_model.hpp"
#include "mippp/model/highs_lp_model.hpp"
#include "mippp/model/soplex_lp_model.hpp"

using namespace fhamonic::mippp;
using namespace fhamonic::mippp::operators;

struct clp_lp_model_test {
    using T = clp_lp_model;
    clp_api api;
    auto construct_model() const { return T(api); }
    static_assert(lp_model<T>);
    static_assert(has_readable_objective<T>);
    static_assert(has_readable_variables_bounds<T>);
    // static_assert(has_readable_constraint_lhs<T>);
    static_assert(has_readable_constraint_sense<T>);
    static_assert(has_readable_constraint_rhs<T>);
    // static_assert(has_readable_constraints<T>);
    static_assert(has_lp_status<T>);
};
struct cbc_milp_model_test {
    using T = cbc_milp_model;
    cbc_api api;
    auto construct_model() const { return T(api); }
    static_assert(lp_model<T>);
    static_assert(has_readable_objective<T>);
    static_assert(has_readable_variables_bounds<T>);
    // static_assert(has_readable_constraint_lhs<T>);
    static_assert(has_readable_constraint_sense<T>);
    static_assert(has_readable_constraint_rhs<T>);
    // static_assert(has_readable_constraints<T>);
};
struct grb_lp_model_test {
    using T = grb_lp_model;
    grb_api api;
    auto construct_model() const { return T(api); }
    static_assert(lp_model<T>);
    static_assert(has_readable_objective<T>);
    static_assert(has_readable_variables_bounds<T>);
    // static_assert(has_readable_constraint_lhs<T>);
    static_assert(has_readable_constraint_sense<T>);
    static_assert(has_readable_constraint_rhs<T>);
    // static_assert(has_readable_constraints<T>);
    static_assert(has_lp_status<T>);
};
struct grb_milp_model_test {
    using T = grb_milp_model;
    grb_api api;
    auto construct_model() const { return T(api); }
    static_assert(lp_model<T>);
    static_assert(has_readable_objective<T>);
    static_assert(has_readable_variables_bounds<T>);
    // static_assert(has_readable_constraint_lhs<T>);
    static_assert(has_readable_constraint_sense<T>);
    static_assert(has_readable_constraint_rhs<T>);
    // static_assert(has_readable_constraints<T>);
};
struct soplex_lp_model_test {
    using T = soplex_lp_model;
    soplex_api api;
    auto construct_model() const { return T(api); }
    static_assert(lp_model<T>);
};
struct glpk_lp_model_test {
    using T = glpk_lp_model;
    glpk_api api;
    auto construct_model() const { return T(api); }
    static_assert(lp_model<T>);
    static_assert(has_lp_status<T>);
};
struct highs_lp_model_test {
    using T = highs_lp_model;
    highs_api api{"highs", "/usr/local/lib"};
    auto construct_model() const { return T(api); }
    static_assert(lp_model<T>);
    static_assert(has_lp_status<T>);
};

using Models =
    ::testing::Types<clp_lp_model_test, cbc_milp_model_test, grb_lp_model_test,
                     grb_milp_model_test, soplex_lp_model_test,
                     glpk_lp_model_test, highs_lp_model_test>;

template <typename T>
class ModelTest : public ::testing::Test {
private:
    T m;

public:
    auto construct_model() const { return m.construct_model(); }
};

TYPED_TEST_SUITE(ModelTest, Models);

TYPED_TEST(ModelTest, add_variable) {
    using T = TypeParam::T;
    auto model = this->construct_model();
    auto x = model.add_variable();
    ASSERT_EQ(model.num_variables(), 1);
    ASSERT_EQ(x.id(), 0);
    if constexpr(has_readable_objective<T>) {
        ASSERT_EQ(model.get_objective_coefficient(x), 0.0);
    }
    if constexpr(has_readable_variables_bounds<T>) {
        ASSERT_EQ(model.get_variable_lower_bound(x), 0.0);
        // ASSERT_EQ(model.get_variable_upper_bound(x), M::infinity);
    }
}
TYPED_TEST(ModelTest, add_variable_2) {
    using T = TypeParam::T;
    auto model = this->construct_model();
    auto x = model.add_variable();
    auto y = model.add_variable();
    ASSERT_EQ(model.num_variables(), 2);
    ASSERT_EQ(x.id(), 0);
    ASSERT_EQ(y.id(), 1);
    if constexpr(has_readable_objective<T>) {
        ASSERT_EQ(model.get_objective_coefficient(x), 0.0);
        ASSERT_EQ(model.get_objective_coefficient(y), 0.0);
    }
    if constexpr(has_readable_variables_bounds<T>) {
        ASSERT_EQ(model.get_variable_lower_bound(x), 0.0);
        // ASSERT_EQ(model.get_variable_upper_bound(x), M::infinity);
        ASSERT_EQ(model.get_variable_lower_bound(y), 0.0);
        // ASSERT_EQ(model.get_variable_upper_bound(y), M::infinity);
    }
}
TYPED_TEST(ModelTest, add_variable_w_params) {
    using T = TypeParam::T;
    T model = this->construct_model();
    auto x = model.add_variable({.obj_coef = 7.0, .upper_bound = 13.0});
    auto y = model.add_variable({.lower_bound = 2});
    ASSERT_EQ(model.num_variables(), 2);
    ASSERT_EQ(x.id(), 0);
    ASSERT_EQ(y.id(), 1);
    if constexpr(has_readable_objective<T>) {
        ASSERT_EQ(model.get_objective_coefficient(x), 7.0);
        ASSERT_EQ(model.get_objective_coefficient(y), 0.0);
    }
    if constexpr(has_readable_variables_bounds<T>) {
        ASSERT_EQ(model.get_variable_lower_bound(x), 0.0);
        ASSERT_EQ(model.get_variable_upper_bound(x), 13.0);
        ASSERT_EQ(model.get_variable_lower_bound(y), 2.0);
        // ASSERT_EQ(model.get_variable_upper_bound(y), M::infinity);
    }
}
TYPED_TEST(ModelTest, optimize_empty_min) {
    using T = TypeParam::T;
    T model = this->construct_model();
    model.set_minimization();
    model.optimize();
    if constexpr(has_lp_status<T>) {
        ASSERT_EQ(model.get_lp_status().value(), lp_status::optimal);
    }
    ASSERT_EQ(model.get_solution_value(), 0.0);
}
TYPED_TEST(ModelTest, optimize_empty_max) {
    using T = TypeParam::T;
    T model = this->construct_model();
    model.set_maximization();
    model.optimize();
    if constexpr(has_lp_status<T>) {
        ASSERT_EQ(model.get_lp_status().value(), lp_status::optimal);
    }
    ASSERT_EQ(model.get_solution_value(), 0.0);
}
TYPED_TEST(ModelTest, add_constraint_and_get) {
    using T = TypeParam::T;
    T model = this->construct_model();
    auto x = model.add_variable();
    auto y = model.add_variable();
    auto c = model.add_constraint(1.5 * x + 4 * y <= 5);
    ASSERT_EQ(model.num_constraints(), 1);
    // if constexpr(has_readable_constraint_lhs<T>) {
    //     ASSERT_EQ_SETS(model.get_constraint_lhs(c),
    //                    {{x.id(), 1.5}, {y.id(), 4.0}});
    // }
    if constexpr(has_readable_constraint_sense<T>) {
        ASSERT_EQ(model.get_constraint_sense(c),
                  constraint_relation::less_equal_zero);
    }
    if constexpr(has_readable_constraint_rhs<T>) {
        ASSERT_EQ(model.get_constraint_rhs(c), 5.0);
    }
}
TYPED_TEST(ModelTest, add_constraint_and_optimize_max_bounded) {
    using T = TypeParam::T;
    T model = this->construct_model();
    auto x = model.add_variable({.upper_bound = 3});
    auto y = model.add_variable({.lower_bound = 1});
    model.set_maximization();
    model.set_objective(2 * x + 1.5 * y);
    auto c = model.add_constraint(x + y <= 5);
    model.optimize();
    if constexpr(has_lp_status<T>) {
        ASSERT_EQ(model.get_lp_status().value(), lp_status::optimal);
    }
    ASSERT_EQ(model.get_solution_value(), 9.0);
    auto solution = model.get_solution();
    ASSERT_EQ(solution[x], 3.0);
    ASSERT_EQ(solution[y], 2);
    if constexpr(has_dual_solution<T>) {
        auto dual_solution = model.get_dual_solution();
        ASSERT_EQ(dual_solution[c], 1.5);
    }
}
TYPED_TEST(ModelTest, add_constraint_and_optimize_min_bounded) {
    using T = TypeParam::T;
    T model = this->construct_model();
    auto x = model.add_variable({.upper_bound = 3});
    auto y = model.add_variable({.lower_bound = 1});
    model.set_minimization();
    model.set_objective(-2 * x - 1.5 * y);
    auto c = model.add_constraint(x + y <= 5);
    model.optimize();
    if constexpr(has_lp_status<T>) {
        ASSERT_EQ(model.get_lp_status().value(), lp_status::optimal);
    }
    ASSERT_EQ(model.get_solution_value(), -9.0);
    auto solution = model.get_solution();
    ASSERT_EQ(solution[x], 3.0);
    ASSERT_EQ(solution[y], 2);
    if constexpr(has_dual_solution<T>) {
        auto dual_solution = model.get_dual_solution();
        ASSERT_EQ(dual_solution[c], -1.5);
    }
}
TYPED_TEST(ModelTest, add_constraint_and_optimize_max_unbounded) {
    using T = TypeParam::T;
    T model = this->construct_model();
    auto x = model.add_variable(
        {.lower_bound = -std::numeric_limits<double>::infinity(),
         .upper_bound = 3});
    auto y = model.add_variable({.lower_bound = 1});
    model.set_maximization();
    model.set_objective(-2 * x + -1.5 * y);
    model.add_constraint(x + y <= 5);
    model.optimize();
    if constexpr(has_lp_status<T>) {
        ASSERT_EQ(model.get_lp_status().value(), lp_status::unbounded);
    }
}
TYPED_TEST(ModelTest, add_constraint_and_optimize_min_unbounded) {
    using T = TypeParam::T;
    T model = this->construct_model();
    auto x = model.add_variable(
        {.lower_bound = -std::numeric_limits<double>::infinity(),
         .upper_bound = 3});
    auto y = model.add_variable({.lower_bound = 1});
    model.set_minimization();
    model.set_objective(2 * x + 1.5 * y);
    model.add_constraint(x + y <= 5);
    model.optimize();
    if constexpr(has_lp_status<T>) {
        ASSERT_EQ(model.get_lp_status().value(), lp_status::unbounded);
    }
}
TYPED_TEST(ModelTest, add_constraint_and_optimize_max_infeasible) {
    using T = TypeParam::T;
    T model = this->construct_model();
    auto x = model.add_variable(
        {.lower_bound = -std::numeric_limits<double>::infinity(),
         .upper_bound = 3});
    auto y = model.add_variable({.lower_bound = 1});
    model.set_maximization();
    model.set_objective(-2 * x + -1.5 * y);
    model.add_constraint(x + y <= 4);
    model.add_constraint(x + y >= 5);
    model.optimize();
    if constexpr(has_lp_status<T>) {
        ASSERT_EQ(model.get_lp_status().value(), lp_status::infeasible);
    }
}
TYPED_TEST(ModelTest, add_constraint_and_optimize_min_infeasible) {
    using T = TypeParam::T;
    T model = this->construct_model();
    auto x = model.add_variable(
        {.lower_bound = -std::numeric_limits<double>::infinity(),
         .upper_bound = 3});
    auto y = model.add_variable({.lower_bound = 1});
    model.set_minimization();
    model.set_objective(2 * x + 1.5 * y);
    model.add_constraint(x + y <= 4);
    model.add_constraint(x + y >= 5);
    model.optimize();
    if constexpr(has_lp_status<T>) {
        ASSERT_EQ(model.get_lp_status().value(), lp_status::infeasible);
    }
}
// TYPED_TEST(ModelTest, set_variable_bounds) {
//     using T = TypeParam::T;
//     if constexpr(has_modifiable_variables_bounds<T>) {
//         using T = TypeParam::T;
//         T model = this->construct_model();
//         auto x = model.add_variable({.obj_coef = 7.0, .upper_bound = 13.0});
//         auto y = model.add_variable({.lower_bound = 2});
//         if constexpr(has_readable_objective<T>) {
//             ASSERT_EQ(model.get_objective_coefficient(x), 7.0);
//             ASSERT_EQ(model.get_objective_coefficient(y), 0.0);
//         }
//         if constexpr(has_readable_variables_bounds<T>) {
//             ASSERT_EQ(model.get_variable_lower_bound(x), 0.0);
//             ASSERT_EQ(model.get_variable_upper_bound(x), 13.0);
//             ASSERT_EQ(model.get_variable_lower_bound(y), 2.0);
//             // ASSERT_EQ(model.get_variable_upper_bound(y), M::infinity);
//         }
//         model.set_variable_lower_bound(x, 3.0);
//         model.set_variable_upper_bound(y, 15.0);
//         if constexpr(has_readable_objective<T>) {
//             ASSERT_EQ(model.get_objective_coefficient(x), 7.0);
//             ASSERT_EQ(model.get_objective_coefficient(y), 0.0);
//         }
//         if constexpr(has_readable_variables_bounds<T>) {
//             ASSERT_EQ(model.get_variable_lower_bound(x), 3.0);
//             ASSERT_EQ(model.get_variable_upper_bound(x), 13.0);
//             ASSERT_EQ(model.get_variable_lower_bound(y), 2.0);
//             ASSERT_EQ(model.get_variable_upper_bound(y), 15.0);
//         }
//     }
// }