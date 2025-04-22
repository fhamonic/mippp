#undef NDEBUG
#include <gtest/gtest.h>

#include "assert_helper.hpp"

#include "mippp/model_concepts.hpp"

#include "mippp/solvers/cbc/all.hpp"
#include "mippp/solvers/clp/all.hpp"
#include "mippp/solvers/glpk/all.hpp"
#include "mippp/solvers/gurobi/all.hpp"
#include "mippp/solvers/highs/all.hpp"
#include "mippp/solvers/scip/all.hpp"
#include "mippp/solvers/soplex/all.hpp"

// #include "mippp/solvers/cplex/all.hpp"
// #include "mippp/solvers/mosek/all.hpp"

using namespace fhamonic::mippp;
using namespace fhamonic::mippp::operators;

GTEST_TEST(any, test) {
    // mosek_api api("mosek64",
    // "/home/plaiseek/Softwares/mosek/11.0/tools/platform/linux64x86/bin");
    // cplex_api api(
    //     "cplex2212",
    // "/home/plaiseek/Softwares/cplex-community/cplex/bin/x86-64_linux");

    auto indices = std::views::iota(0, 9);
    auto values = std::views::iota(1, 10);
    auto coords = std::views::transform(
        indices, [](auto && i) { return std::make_pair(i / 3, i % 3); });

    cbc_api api;
    cbc_milp model(api);

    auto X_vars =
        model.add_binary_variables(9 * 9 * 9, [](int i, int j, int value) {
            return (81 * i) + (9 * j) + (value - 1);
        });

    for(auto i : indices) {
        for(auto j : indices) {
            model.add_constraint(
                xsum(values, [&](auto && v) { return X_vars(i, j, v); }) == 1);
        }
    }
    for(auto v : values) {
        for(auto i : indices) {
            model.add_constraint(
                xsum(indices, [&](auto && j) { return X_vars(i, j, v); }) == 1);
            model.add_constraint(
                xsum(indices, [&](auto && j) { return X_vars(j, i, v); }) == 1);
        }
        for(auto && [bi, bj] : coords) {
            model.add_constraint(xsum(coords, [&](auto && p) {
                                     return X_vars(3 * bi + p.first,
                                                   3 * bj + p.second, v);
                                 }) == 1);
        }
    }

    for(auto && x :
        {X_vars(0, 2, 8), X_vars(0, 4, 1), X_vars(0, 8, 9), X_vars(1, 0, 6),
         X_vars(1, 2, 1), X_vars(1, 4, 9), X_vars(1, 6, 3), X_vars(1, 7, 2),
         X_vars(2, 1, 4), X_vars(2, 4, 3), X_vars(2, 5, 7), X_vars(2, 8, 5),
         X_vars(3, 1, 3), X_vars(3, 2, 5), X_vars(3, 5, 8), X_vars(3, 6, 2),
         X_vars(4, 2, 2), X_vars(4, 3, 6), X_vars(4, 4, 5), X_vars(4, 6, 8),
         X_vars(5, 2, 4), X_vars(5, 5, 1), X_vars(5, 6, 7), X_vars(5, 7, 5),
         X_vars(6, 0, 5), X_vars(6, 3, 3), X_vars(6, 4, 4), X_vars(6, 7, 8),
         X_vars(7, 1, 9), X_vars(7, 2, 7), X_vars(7, 4, 8), X_vars(7, 6, 5),
         X_vars(7, 8, 6), X_vars(8, 0, 1), X_vars(8, 4, 6), X_vars(8, 6, 9)}) {
        model.set_variable_lower_bound(x, 1);
    }

    model.solve();
    auto solution = model.get_solution();

    for(auto i : indices) {
        for(auto j : indices) {
            for(auto v : values) {
                if(solution[X_vars(i, j, v)]) std::cout << ' ' << v;
            }
        }
        std::cout << std::endl;
    }

    ASSERT_TRUE(false);
}

//*
struct clp_lp_test {
    using T = clp_lp;
    clp_api api;
    auto construct_model() const { return T(api); }
    static_assert(lp_model<T>);
    static_assert(has_readable_objective<T>);
    static_assert(has_modifiable_objective<T>);
    static_assert(has_readable_variables_bounds<T>);
    // static_assert(has_readable_constraint_lhs<T>);
    static_assert(has_readable_constraint_sense<T>);
    static_assert(has_readable_constraint_rhs<T>);
    // static_assert(has_readable_constraints<T>);
    static_assert(has_lp_status<T>);
    static_assert(has_dual_solution<T>);
};
struct cbc_milp_test {
    using T = cbc_milp;
    cbc_api api;
    auto construct_model() const { return T(api); }
    static_assert(lp_model<T>);
    static_assert(milp_model<T>);
    static_assert(has_readable_objective<T>);
    static_assert(has_modifiable_objective<T>);
    static_assert(has_readable_variables_bounds<T>);
    // static_assert(has_readable_constraint_lhs<T>);
    static_assert(has_readable_constraint_sense<T>);
    static_assert(has_readable_constraint_rhs<T>);
    // static_assert(has_readable_constraints<T>);
};
struct grb_lp_test {
    using T = grb_lp;
    grb_api api;
    auto construct_model() const { return T(api); }
    static_assert(lp_model<T>);
    static_assert(has_readable_objective<T>);
    static_assert(has_modifiable_objective<T>);
    static_assert(has_readable_variables_bounds<T>);
    // static_assert(has_readable_constraint_lhs<T>);
    static_assert(has_readable_constraint_sense<T>);
    static_assert(has_readable_constraint_rhs<T>);
    // static_assert(has_readable_constraints<T>);
    static_assert(has_lp_status<T>);
    static_assert(has_dual_solution<T>);
};
struct grb_milp_test {
    using T = grb_milp;
    grb_api api;
    auto construct_model() const { return T(api); }
    static_assert(lp_model<T>);
    static_assert(milp_model<T>);
    static_assert(has_readable_objective<T>);
    static_assert(has_modifiable_objective<T>);
    static_assert(has_readable_variables_bounds<T>);
    // static_assert(has_readable_constraint_lhs<T>);
    static_assert(has_readable_constraint_sense<T>);
    static_assert(has_readable_constraint_rhs<T>);
    // static_assert(has_readable_constraints<T>);
};
struct soplex_lp_test {
    using T = soplex_lp;
    soplex_api api;
    auto construct_model() const { return T(api); }
    static_assert(lp_model<T>);
    static_assert(has_dual_solution<T>);
};
struct glpk_lp_test {
    using T = glpk_lp;
    glpk_api api;
    auto construct_model() const { return T(api); }
    static_assert(lp_model<T>);
    static_assert(has_lp_status<T>);
    static_assert(has_dual_solution<T>);
};
struct highs_lp_test {
    using T = highs_lp;
    highs_api api{"highs", "/usr/local/lib"};
    auto construct_model() const { return T(api); }
    static_assert(lp_model<T>);
    // static_assert(has_modifiable_objective<T>);
    static_assert(has_lp_status<T>);
    static_assert(has_dual_solution<T>);
};
struct scip_milp_test {
    using T = scip_milp;
    scip_api api;
    auto construct_model() const { return T(api); }
    static_assert(lp_model<T>);
    static_assert(has_modifiable_objective<T>);
};
// struct mosek_lp_test {
//     using T = mosek_lp;
//     mosek_api api{
//         "mosek64",
//         "/home/plaiseek/Softwares/mosek/11.0/tools/platform/linux64x86/bin"};
//     auto construct_model() const { return T(api); }
//     static_assert(lp_model<T>);
// };
// struct cplex_lp_test {
//     using T = cplex_lp;
//     cplex_api api{
//         "cplex2212",
//         "/home/plaiseek/Softwares/cplex-community/cplex/bin/x86-64_linux"};
//     auto construct_model() const { return T(api); }
//     static_assert(lp_model<T>);
// };

// clang-format off
using Models = ::testing::Types<
        clp_lp_test,
        cbc_milp_test,
        grb_lp_test,
        grb_milp_test,
        soplex_lp_test,
        glpk_lp_test,
        // glpk_milp_test,
        highs_lp_test,
        // highs_milp_test,
        scip_milp_test
        // mosek_lp_test
        // mosek_milp_test
        // cplex_lp_test
        // cplex_milp_test
        // copt_lp_test
        // copt_milp_test
        // xprs_lp_test
        // xprs_milp_test
        >;
// clang-format on

#define EPSILON 1e-13
#define INFTY 1e100

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
        ASSERT_GE(model.get_variable_upper_bound(x), INFTY);
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
        ASSERT_GE(model.get_variable_upper_bound(x), INFTY);
        ASSERT_EQ(model.get_variable_lower_bound(y), 0.0);
        ASSERT_GE(model.get_variable_upper_bound(y), INFTY);
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
        ASSERT_EQ(model.get_variable_upper_bound(x), 13.0);
        ASSERT_LE(model.get_variable_lower_bound(x), -INFTY);
        ASSERT_EQ(model.get_variable_lower_bound(y), 2.0);
        ASSERT_GE(model.get_variable_upper_bound(y), INFTY);
    }
}
TYPED_TEST(ModelTest, add_variables) {
    using T = TypeParam::T;
    auto model = this->construct_model();

    auto F = model.add_variable();
    auto X_vars = model.add_variables(
        4, {.obj_coef = 1, .lower_bound = 2, .upper_bound = 5});
    auto Y_vars = model.add_variables(3);

    ASSERT_EQ(model.num_variables(), 8);
    ASSERT_EQ(F.id(), 0);
    ASSERT_EQ(X_vars[0].id(), 1);
    ASSERT_EQ(X_vars[1].id(), 2);
    ASSERT_EQ(X_vars[2].id(), 3);
    ASSERT_EQ(X_vars[3].id(), 4);
    ASSERT_EQ(Y_vars[0].id(), 5);
    ASSERT_EQ(Y_vars[1].id(), 6);
    ASSERT_EQ(Y_vars[2].id(), 7);

    ASSERT_EQ_RANGES(X_vars, {X_vars[0], X_vars[1], X_vars[2], X_vars[3]});
    ASSERT_EQ_RANGES(Y_vars, {Y_vars[0], Y_vars[1], Y_vars[2]});

    // ASSERT_DEATH creates subprocesses that increase computation time
    ASSERT_DEATH(X_vars[-1], ".*Assertion.*failed.*");
    ASSERT_DEATH(X_vars[4], ".*Assertion.*failed.*");
    ASSERT_DEATH(Y_vars[-1], ".*Assertion.*failed.*");
    ASSERT_DEATH(Y_vars[3], ".*Assertion.*failed.*");

    if constexpr(has_readable_objective<T>) {
        for(auto && x : X_vars)
            ASSERT_EQ(model.get_objective_coefficient(x), 1);
        for(auto && y : Y_vars)
            ASSERT_EQ(model.get_objective_coefficient(y), 0);
    }
    if constexpr(has_readable_variables_bounds<T>) {
        for(auto && x : X_vars) {
            ASSERT_EQ(model.get_variable_lower_bound(x), 2.0);
            ASSERT_EQ(model.get_variable_upper_bound(x), 5.0);
        }
        for(auto && y : Y_vars) {
            ASSERT_EQ(model.get_variable_lower_bound(y), 0.0);
            ASSERT_GE(model.get_variable_upper_bound(y), INFTY);
        }
    }
}
TYPED_TEST(ModelTest, add_indexed_variables) {
    using T = TypeParam::T;
    auto model = this->construct_model();

    auto F = model.add_variable();
    auto X_vars = model.add_variables(
        4, [](int a) { return 3 - a; },
        {.obj_coef = 1, .lower_bound = 2, .upper_bound = 5});
    auto Y_vars =
        model.add_variables(4, [](int a, int b) { return a * 2 + b; });

    ASSERT_EQ(model.num_variables(), 9);
    ASSERT_EQ(F.id(), 0);
    ASSERT_EQ(X_vars(0).id(), 4);
    ASSERT_EQ(X_vars(1).id(), 3);
    ASSERT_EQ(X_vars(2).id(), 2);
    ASSERT_EQ(X_vars(3).id(), 1);
    ASSERT_EQ(Y_vars(0, 0).id(), 5);
    ASSERT_EQ(Y_vars(0, 1).id(), 6);
    ASSERT_EQ(Y_vars(1, 0).id(), 7);
    ASSERT_EQ(Y_vars(1, 1).id(), 8);

    // ASSERT_DEATH creates subprocesses that increase computation time
    ASSERT_DEATH(X_vars(-1), ".*Assertion.*failed.*");
    ASSERT_DEATH(X_vars(4), ".*Assertion.*failed.*");
    ASSERT_DEATH(Y_vars(0, -1), ".*Assertion.*failed.*");
    ASSERT_DEATH(Y_vars(2, 0), ".*Assertion.*failed.*");

    ASSERT_EQ_RANGES(X_vars, {X_vars(3), X_vars(2), X_vars(1), X_vars(0)});
    ASSERT_EQ_RANGES(Y_vars,
                     {Y_vars(0, 0), Y_vars(0, 1), Y_vars(1, 0), Y_vars(1, 1)});

    if constexpr(has_readable_objective<T>) {
        for(auto && x : X_vars)
            ASSERT_EQ(model.get_objective_coefficient(x), 1);
        for(auto && y : Y_vars)
            ASSERT_EQ(model.get_objective_coefficient(y), 0);
    }
    if constexpr(has_readable_variables_bounds<T>) {
        for(auto && x : X_vars) {
            ASSERT_EQ(model.get_variable_lower_bound(x), 2.0);
            ASSERT_EQ(model.get_variable_upper_bound(x), 5.0);
        }
        for(auto && y : Y_vars) {
            ASSERT_EQ(model.get_variable_lower_bound(y), 0.0);
            ASSERT_GE(model.get_variable_upper_bound(y), INFTY);
        }
    }
}
TYPED_TEST(ModelTest, optimize_empty_min) {
    using T = TypeParam::T;
    T model = this->construct_model();
    model.set_minimization();
    model.solve();
    if constexpr(has_lp_status<T>) {
        ASSERT_EQ(model.get_lp_status().value(), lp_status::optimal);
    }
    ASSERT_DOUBLE_EQ(model.get_solution_value(), 0.0);
}
TYPED_TEST(ModelTest, optimize_empty_max) {
    using T = TypeParam::T;
    T model = this->construct_model();
    model.set_maximization();
    model.solve();
    if constexpr(has_lp_status<T>) {
        ASSERT_EQ(model.get_lp_status().value(), lp_status::optimal);
    }
    ASSERT_DOUBLE_EQ(model.get_solution_value(), 0.0);
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
        ASSERT_DOUBLE_EQ(model.get_constraint_rhs(c), 5.0);
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
    model.solve();
    if constexpr(has_lp_status<T>) {
        ASSERT_EQ(model.get_lp_status().value(), lp_status::optimal);
    }
    ASSERT_DOUBLE_EQ(model.get_solution_value(), 9.0);
    auto solution = model.get_solution();
    ASSERT_DOUBLE_EQ(solution[x], 3.0);
    ASSERT_DOUBLE_EQ(solution[y], 2);
    if constexpr(has_dual_solution<T>) {
        auto dual_solution = model.get_dual_solution();
        ASSERT_DOUBLE_EQ(dual_solution[c], 1.5);
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
    model.solve();
    if constexpr(has_lp_status<T>) {
        ASSERT_EQ(model.get_lp_status().value(), lp_status::optimal);
    }
    ASSERT_DOUBLE_EQ(model.get_solution_value(), -9.0);
    auto solution = model.get_solution();
    ASSERT_DOUBLE_EQ(solution[x], 3.0);
    ASSERT_DOUBLE_EQ(solution[y], 2);
    if constexpr(has_dual_solution<T>) {
        auto dual_solution = model.get_dual_solution();
        ASSERT_DOUBLE_EQ(dual_solution[c], -1.5);
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
    model.solve();
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
    model.solve();
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
    model.solve();
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
    model.solve();
    if constexpr(has_lp_status<T>) {
        ASSERT_EQ(model.get_lp_status().value(), lp_status::infeasible);
    }
}
TYPED_TEST(ModelTest, lp_example) {
    using T = TypeParam::T;
    T model = this->construct_model();
    auto x1 = model.add_variable();
    auto x2 = model.add_variable();
    auto x3 = model.add_variable();
    model.set_maximization();
    model.set_objective(5 * x1 + 4 * x2 + 3 * x3);
    auto c1 = model.add_constraint(2 * x1 + 3 * x2 + x3 <= 5);
    auto c2 = model.add_constraint(4 * x1 + x2 + 2 * x3 <= 11);
    auto c3 = model.add_constraint(3 * x1 + 4 * x2 + 2 * x3 <= 8);
    model.solve();
    if constexpr(has_lp_status<T>) {
        ASSERT_EQ(model.get_lp_status().value(), lp_status::optimal);
    }
    ASSERT_NEAR(model.get_solution_value(), 13.0, EPSILON);
    auto solution = model.get_solution();
    ASSERT_NEAR(solution[x1], 2.0, EPSILON);
    ASSERT_NEAR(solution[x2], 0.0, EPSILON);
    ASSERT_NEAR(solution[x3], 1.0, EPSILON);
    if constexpr(has_dual_solution<T>) {
        auto dual_solution = model.get_dual_solution();
        ASSERT_NEAR(dual_solution[c1], 1.0, EPSILON);
        ASSERT_NEAR(dual_solution[c2], 0.0, EPSILON);
        ASSERT_NEAR(dual_solution[c3], 1.0, EPSILON);
    }
}
TYPED_TEST(ModelTest, lp_example_set_objective_redundant_terms) {
    using T = TypeParam::T;
    T model = this->construct_model();
    auto x1 = model.add_variable();
    auto x2 = model.add_variable();
    auto x3 = model.add_variable();
    model.set_maximization();
    model.set_objective(2 * x1 + 4 * x2 + 3 * x1 + 3 * x3);
    auto c1 = model.add_constraint(2 * x1 + 3 * x2 + x3 <= 5);
    auto c2 = model.add_constraint(4 * x1 + x2 + 2 * x3 <= 11);
    auto c3 = model.add_constraint(3 * x1 + 4 * x2 + 2 * x3 <= 8);
    model.solve();
    if constexpr(has_lp_status<T>) {
        ASSERT_EQ(model.get_lp_status().value(), lp_status::optimal);
    }
    ASSERT_NEAR(model.get_solution_value(), 13.0, EPSILON);
    auto solution = model.get_solution();
    ASSERT_NEAR(solution[x1], 2.0, EPSILON);
    ASSERT_NEAR(solution[x2], 0.0, EPSILON);
    ASSERT_NEAR(solution[x3], 1.0, EPSILON);
    if constexpr(has_dual_solution<T>) {
        auto dual_solution = model.get_dual_solution();
        ASSERT_NEAR(dual_solution[c1], 1.0, EPSILON);
        ASSERT_NEAR(dual_solution[c2], 0.0, EPSILON);
        ASSERT_NEAR(dual_solution[c3], 1.0, EPSILON);
    }
}
TYPED_TEST(ModelTest, lp_example_add_constraint_redundant_terms) {
    using T = TypeParam::T;
    T model = this->construct_model();
    auto x1 = model.add_variable();
    auto x2 = model.add_variable();
    auto x3 = model.add_variable();
    model.set_maximization();
    model.set_objective(5 * x1 + 4 * x2 + 3 * x3);
    auto c1 = model.add_constraint(3 * x1 - x1 + 3 * x2 + x3 <= 5);
    auto c2 = model.add_constraint(4 * x1 + x3 + x2 + x3 <= 11);
    auto c3 = model.add_constraint(3 * x1 + 4 * x2 + 2 * x3 <= 8);
    model.solve();
    if constexpr(has_lp_status<T>) {
        ASSERT_EQ(model.get_lp_status().value(), lp_status::optimal);
    }
    ASSERT_NEAR(model.get_solution_value(), 13.0, EPSILON);
    auto solution = model.get_solution();
    ASSERT_NEAR(solution[x1], 2.0, EPSILON);
    ASSERT_NEAR(solution[x2], 0.0, EPSILON);
    ASSERT_NEAR(solution[x3], 1.0, EPSILON);
    if constexpr(has_dual_solution<T>) {
        auto dual_solution = model.get_dual_solution();
        ASSERT_NEAR(dual_solution[c1], 1.0, EPSILON);
        ASSERT_NEAR(dual_solution[c2], 0.0, EPSILON);
        ASSERT_NEAR(dual_solution[c3], 1.0, EPSILON);
    }
}
//*/