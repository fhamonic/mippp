#include <gtest/gtest.h>

#include "mippp/expressions/linear_expression_operators.hpp"
#include "mippp/constraints/linear_constraint_operators.hpp"
#include "mippp/model.hpp"

#include "assert_eq_ranges.hpp"

using namespace fhamonic::mippp;

using Var = variable<int, double>;

GTEST_TEST(grb_model, ctor) { Model<GrbTraits> model; }

GTEST_TEST(grb_model, add_var) {
    Model<GrbTraits> model;
    auto x = model.add_var();
    ASSERT_EQ(x.id(), 0);
    ASSERT_EQ(model.nb_variables(), 1);
}

GTEST_TEST(grb_model, add_var_default_options) {
    Model<GrbTraits> model;
    auto x = model.add_var();
    ASSERT_EQ(x.id(), 0);
    ASSERT_EQ(model.nb_variables(), 1);
    ASSERT_EQ(model.obj_coef(x), 0);
    ASSERT_EQ(model.lower_bound(x), 0);
    ASSERT_EQ(model.upper_bound(x), Model<GrbTraits>::INFTY);
    ASSERT_EQ(model.type(x), Model<GrbTraits>::ColType::CONTINUOUS);
}

GTEST_TEST(grb_model, add_var_custom_options) {
    Model<GrbTraits> model;
    auto x = model.add_var({.obj_coef = 6.14,
                            .lower_bound = 3.2,
                            .upper_bound = 9,
                            .type = Model<GrbTraits>::ColType::BINARY});
    ASSERT_EQ(x.id(), 0);
    ASSERT_EQ(model.nb_variables(), 1);
    ASSERT_EQ(model.obj_coef(x), 6.14);
    ASSERT_EQ(model.lower_bound(x), 3.2);
    ASSERT_EQ(model.upper_bound(x), 9);
    ASSERT_EQ(model.type(x), Model<GrbTraits>::ColType::BINARY);
}

GTEST_TEST(grb_model, add_vars) {
    Model<GrbTraits> model;
    auto x_vars = model.add_vars(5, [](int i) { return 4 - i; });
    ASSERT_EQ(model.nb_variables(), 5);
    for(int i = 0; i < 5; ++i) {
        ASSERT_EQ(x_vars(i).id(), 4 - i);
        ASSERT_EQ(model.obj_coef(x_vars(i)), 0);
        ASSERT_EQ(model.lower_bound(x_vars(i)), 0);
        ASSERT_EQ(model.upper_bound(x_vars(i)), Model<GrbTraits>::INFTY);
        ASSERT_EQ(model.type(x_vars(i)), Model<GrbTraits>::ColType::CONTINUOUS);
    }
}

GTEST_TEST(grb_model, add_obj) {
    Model<GrbTraits> model;
    auto x = model.add_var();
    auto y = model.add_var();
    model.add_obj(x - 3 * y);
    ASSERT_EQ(model.nb_variables(), 2);
    ASSERT_EQ(model.obj_coef(x), 1);
    ASSERT_EQ(model.obj_coef(y), -3);

    auto z = model.add_var();
    ASSERT_EQ(model.nb_variables(), 3);
    model.add_obj(-z + y);
    ASSERT_EQ(model.obj_coef(x), 1);
    ASSERT_EQ(model.obj_coef(y), -2);
    ASSERT_EQ(model.obj_coef(z), -1);
}

GTEST_TEST(grb_model, add_constraint) {
    Model<GrbTraits> model;
    auto x = model.add_var();
    auto y = model.add_var();
    auto z = model.add_var();

    auto constr_id = model.add_constraint(1 <= -z + y + 3*x <= 8);
    ASSERT_EQ(constr_id, 0);
    ASSERT_EQ(model.nb_constraints(), 1);
    ASSERT_EQ(model.nb_entries(), 3);

    auto constr = model.constraint(constr_id);
    ASSERT_EQ(constr.lower_bound(), 1);
    ASSERT_EQ(constr.upper_bound(), 8);
    ASSERT_EQ_RANGES(constr.variables(), {2, 1, 0});
    ASSERT_EQ_RANGES(constr.coefficients(), {-1.0, 1.0, 3.0});
}

GTEST_TEST(grb_model, build) {
    Model<GrbTraits> model;
    auto x = model.add_var({.lower_bound=0, .upper_bound=20});
    auto y = model.add_var({.upper_bound=12});
    model.add_obj(2*x + 3*y);
    model.add_constraint(x + y <= 30);

    auto solver_model = model.build();

    solver_model.optimize();
    std::vector<double> solution = solver_model.get_solution();

    ASSERT_EQ(solution[static_cast<std::size_t>(x.id())], 18);
    ASSERT_EQ(solution[static_cast<std::size_t>(y.id())], 12);
}
