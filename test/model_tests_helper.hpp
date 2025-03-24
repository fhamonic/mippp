#ifndef MODEL_TESTS_HELPER_HPP
#define MODEL_TESTS_HELPER_HPP

#include <gtest/gtest.h>

#include <range/v3/view/iota.hpp>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/mip_model.hpp"

#include "assert_helper.hpp"

using namespace fhamonic::mippp;

template <typename M, typename S>
class model_tests {
public:
    static void default_constructor() { M model; }
    static void add_variable_default_name_default_options() {
        M model;
        auto x = model.add_variable();
        auto y = model.add_variable();
        ASSERT_EQ(model.num_variables(), 2);
        ASSERT_EQ(x.id(), 0);
        ASSERT_EQ(model.obj_coef(x), 0);
        ASSERT_EQ(model.lower_bound(x), 0);
        ASSERT_EQ(model.upper_bound(x), M::infinity);
        ASSERT_EQ(model.type(x), M::var_category::continuous);
        ASSERT_EQ(model.name(x), "x0");
        ASSERT_EQ(y.id(), 1);
        ASSERT_EQ(model.obj_coef(y), 0);
        ASSERT_EQ(model.lower_bound(y), 0);
        ASSERT_EQ(model.upper_bound(y), M::infinity);
        ASSERT_EQ(model.type(y), M::var_category::continuous);
        ASSERT_EQ(model.name(y), "x1");
    }
    static void add_variable_default_name_custom_options() {
        M model;
        auto x = model.add_variable({.obj_coef = 6.14,
                                     .lower_bound = 3.2,
                                     .upper_bound = 9,
                                     .type = M::var_category::binary});
        auto y = model.add_variable({.lower_bound = -2.5});
        auto z = model.add_variable({.obj_coef = 3.6,
                                     .upper_bound = 10,
                                     .type = M::var_category::integer});
        ASSERT_EQ(model.num_variables(), 3);
        ASSERT_EQ(x.id(), 0);
        ASSERT_EQ(model.obj_coef(x), 6.14);
        ASSERT_EQ(model.lower_bound(x), 3.2);
        ASSERT_EQ(model.upper_bound(x), 9);
        ASSERT_EQ(model.type(x), M::var_category::binary);
        ASSERT_EQ(model.name(x), "x0");
        ASSERT_EQ(y.id(), 1);
        ASSERT_EQ(model.obj_coef(y), 0);
        ASSERT_EQ(model.lower_bound(y), -2.5);
        ASSERT_EQ(model.upper_bound(y), M::infinity);
        ASSERT_EQ(model.type(y), M::var_category::continuous);
        ASSERT_EQ(model.name(y), "x1");
        ASSERT_EQ(z.id(), 2);
        ASSERT_EQ(model.obj_coef(z), 3.6);
        ASSERT_EQ(model.lower_bound(z), 0);
        ASSERT_EQ(model.upper_bound(z), 10);
        ASSERT_EQ(model.type(z), M::var_category::integer);
        ASSERT_EQ(model.name(z), "x2");
    }
    static void add_variable_custom_name_default_options() {
        M model;
        auto x = model.add_variable("x");
        auto y = model.add_variable("y");
        ASSERT_EQ(model.num_variables(), 2);
        ASSERT_EQ(x.id(), 0);
        ASSERT_EQ(model.obj_coef(x), 0);
        ASSERT_EQ(model.lower_bound(x), 0);
        ASSERT_EQ(model.upper_bound(x), M::infinity);
        ASSERT_EQ(model.type(x), M::var_category::continuous);
        ASSERT_EQ(model.name(x), "x");
        ASSERT_EQ(y.id(), 1);
        ASSERT_EQ(model.obj_coef(y), 0);
        ASSERT_EQ(model.lower_bound(y), 0);
        ASSERT_EQ(model.upper_bound(y), M::infinity);
        ASSERT_EQ(model.type(y), M::var_category::continuous);
        ASSERT_EQ(model.name(y), "y");
    }
    static void add_variable_custom_name_custom_options() {
        M model;
        auto x = model.add_variable("x", {.obj_coef = 6.14,
                                          .lower_bound = 3.2,
                                          .upper_bound = 9,
                                          .type = M::var_category::binary});
        auto y = model.add_variable("y", {.lower_bound = -2.5});
        auto z = model.add_variable("z", {.obj_coef = 3.6,
                                          .upper_bound = 10,
                                          .type = M::var_category::integer});
        ASSERT_EQ(model.num_variables(), 3);
        ASSERT_EQ(x.id(), 0);
        ASSERT_EQ(model.obj_coef(x), 6.14);
        ASSERT_EQ(model.lower_bound(x), 3.2);
        ASSERT_EQ(model.upper_bound(x), 9);
        ASSERT_EQ(model.type(x), M::var_category::binary);
        ASSERT_EQ(model.name(x), "x");
        ASSERT_EQ(y.id(), 1);
        ASSERT_EQ(model.obj_coef(y), 0);
        ASSERT_EQ(model.lower_bound(y), -2.5);
        ASSERT_EQ(model.upper_bound(y), M::infinity);
        ASSERT_EQ(model.type(y), M::var_category::continuous);
        ASSERT_EQ(model.name(y), "y");
        ASSERT_EQ(z.id(), 2);
        ASSERT_EQ(model.obj_coef(z), 3.6);
        ASSERT_EQ(model.lower_bound(z), 0);
        ASSERT_EQ(model.upper_bound(z), 10);
        ASSERT_EQ(model.type(z), M::var_category::integer);
        ASSERT_EQ(model.name(z), "z");
    }
    static void add_variables_default_name_default_options() {
        M model;
        auto x_vars = model.add_variables(5, [](int i) { return 4 - i; });
        auto y_vars =
            model.add_variables(4, [](int i, int j) { return i * 2 + j; });
        ASSERT_EQ(model.num_variables(), 9);
        for(int i = 0; i < 5; ++i) {
            auto x = x_vars(i);
            ASSERT_EQ(x.id(), 4 - i);
            ASSERT_EQ(model.obj_coef(x), 0);
            ASSERT_EQ(model.lower_bound(x), 0);
            ASSERT_EQ(model.upper_bound(x), M::infinity);
            ASSERT_EQ(model.type(x), M::var_category::continuous);
            ASSERT_EQ(model.name(x), "x" + std::to_string(4 - i));
        }
        for(int i = 0; i < 4; ++i) {
            auto y = y_vars(i / 2, i % 2);
            ASSERT_EQ(y.id(), 5 + i);
            ASSERT_EQ(model.obj_coef(y), 0);
            ASSERT_EQ(model.lower_bound(y), 0);
            ASSERT_EQ(model.upper_bound(y), M::infinity);
            ASSERT_EQ(model.type(y), M::var_category::continuous);
            ASSERT_EQ(model.name(y), "x" + std::to_string(5 + i));
        }
    }
    static void add_variables_default_name_custom_options() {
        M model;
        auto x_vars = model.add_variables(5, [](int i) { return 4 - i; },
                                          {.obj_coef = 6.14,
                                           .lower_bound = 3.2,
                                           .upper_bound = 9,
                                           .type = M::var_category::binary});
        auto y_vars = model.add_variables(
            4, [](int i, int j) { return i * 2 + j; }, {.lower_bound = -2.5});
        auto z_vars = model.add_variables(2, [](int i) { return i; },
                                          {.obj_coef = 3.6,
                                           .upper_bound = 10,
                                           .type = M::var_category::integer});

        ASSERT_EQ(model.num_variables(), 11);
        for(int i = 0; i < 5; ++i) {
            auto x = x_vars(i);
            ASSERT_EQ(x.id(), 4 - i);
            ASSERT_EQ(model.obj_coef(x), 6.14);
            ASSERT_EQ(model.lower_bound(x), 3.2);
            ASSERT_EQ(model.upper_bound(x), 9);
            ASSERT_EQ(model.type(x), M::var_category::binary);
            ASSERT_EQ(model.name(x), "x" + std::to_string(4 - i));
        }
        for(int i = 0; i < 4; ++i) {
            auto y = y_vars(i / 2, i % 2);
            ASSERT_EQ(y.id(), 5 + i);
            ASSERT_EQ(model.obj_coef(y), 0);
            ASSERT_EQ(model.lower_bound(y), -2.5);
            ASSERT_EQ(model.upper_bound(y), M::infinity);
            ASSERT_EQ(model.type(y), M::var_category::continuous);
            ASSERT_EQ(model.name(y), "x" + std::to_string(5 + i));
        }
        for(int i = 0; i < 2; ++i) {
            auto z = z_vars(i);
            ASSERT_EQ(z.id(), 9 + i);
            ASSERT_EQ(model.obj_coef(z), 3.6);
            ASSERT_EQ(model.lower_bound(z), 0);
            ASSERT_EQ(model.upper_bound(z), 10);
            ASSERT_EQ(model.type(z), M::var_category::integer);
            ASSERT_EQ(model.name(z), "x" + std::to_string(9 + i));
        }
    }
    static void add_variables_custom_name_default_options() {
        M model;
        auto x_vars = model.add_variables(
            5, [](int i) { return 4 - i; },
            [](int i) { return "x(4-" + std::to_string(i) + ')'; });
        auto y_vars = model.add_variables(
            4, [](int i, int j) { return i * 2 + j; },
            [](int i, int j) {
                return "y(" + std::to_string(i) + ',' + std::to_string(j) + ')';
            });
        ASSERT_EQ(model.num_variables(), 9);
        for(int i = 0; i < 5; ++i) {
            auto x = x_vars(i);
            ASSERT_EQ(x.id(), 4 - i);
            ASSERT_EQ(model.obj_coef(x), 0);
            ASSERT_EQ(model.lower_bound(x), 0);
            ASSERT_EQ(model.upper_bound(x), M::infinity);
            ASSERT_EQ(model.type(x), M::var_category::continuous);
            ASSERT_EQ(model.name(x), "x(4-" + std::to_string(i) + ')');
        }
        for(int i = 0; i < 4; ++i) {
            auto y = y_vars(i / 2, i % 2);
            ASSERT_EQ(y.id(), 5 + i);
            ASSERT_EQ(model.obj_coef(y), 0);
            ASSERT_EQ(model.lower_bound(y), 0);
            ASSERT_EQ(model.upper_bound(y), M::infinity);
            ASSERT_EQ(model.type(y), M::var_category::continuous);
            ASSERT_EQ(model.name(y), "y(" + std::to_string(i / 2) + ',' +
                                         std::to_string(i % 2) + ')');
        }
    }
    static void add_variables_custom_name_custom_options() {
        M model;
        auto x_vars = model.add_variables(
            5, [](int i) { return 4 - i; },
            [](int i) { return "x(4-" + std::to_string(i) + ')'; },
            {.obj_coef = 6.14,
             .lower_bound = 3.2,
             .upper_bound = 9,
             .type = M::var_category::binary});
        auto y_vars = model.add_variables(
            4, [](int i, int j) { return i * 2 + j; },
            [](int i, int j) {
                return "y(" + std::to_string(i) + ',' + std::to_string(j) + ')';
            },
            {.obj_coef = 3.6,
             .upper_bound = 10,
             .type = M::var_category::integer});
        ASSERT_EQ(model.num_variables(), 9);
        for(int i = 0; i < 5; ++i) {
            auto x = x_vars(i);
            ASSERT_EQ(x.id(), 4 - i);
            ASSERT_EQ(model.obj_coef(x), 6.14);
            ASSERT_EQ(model.lower_bound(x), 3.2);
            ASSERT_EQ(model.upper_bound(x), 9);
            ASSERT_EQ(model.type(x), M::var_category::binary);
            ASSERT_EQ(model.name(x), "x(4-" + std::to_string(i) + ')');
        }
        for(int i = 0; i < 4; ++i) {
            auto y = y_vars(i / 2, i % 2);
            ASSERT_EQ(y.id(), 5 + i);
            ASSERT_EQ(model.obj_coef(y), 3.6);
            ASSERT_EQ(model.lower_bound(y), 0);
            ASSERT_EQ(model.upper_bound(y), 10);
            ASSERT_EQ(model.type(y), M::var_category::integer);
            ASSERT_EQ(model.name(y), "y(" + std::to_string(i / 2) + ',' +
                                         std::to_string(i % 2) + ')');
        }
    }
    static void add_to_objective() {
        using namespace fhamonic::mippp::operators;
        M model;
        auto x = model.add_variable();
        auto y = model.add_variable();
        model.add_to_objective(x - 3 * y);
        ASSERT_EQ(model.num_variables(), 2);
        ASSERT_EQ(model.obj_coef(x), 1);
        ASSERT_EQ(model.obj_coef(y), -3);

        auto z = model.add_variable();
        ASSERT_EQ(model.num_variables(), 3);
        model.add_to_objective(-z + y);
        ASSERT_EQ(model.obj_coef(x), 1);
        ASSERT_EQ(model.obj_coef(y), -2);
        ASSERT_EQ(model.obj_coef(z), -1);
    }
    static void get_objective() {
        using namespace fhamonic::mippp::operators;
        M model;
        auto x = model.add_variable();
        auto y = model.add_variable();
        auto z = model.add_variable();
        model.add_to_objective(x - 3 * y - z + y);
        auto obj = model.objective();
        ASSERT_EXPRESSION(obj, {0, 1, 2}, {1.0, -2.0, -1.0}, 0);
    }
    static void add_and_get_constraint() {
        using namespace fhamonic::mippp::operators;
        M model;
        auto x = model.add_variable();
        auto y = model.add_variable();
        auto z = model.add_variable();

        auto constr_id = model.add_constraint(-z + y + 3 * x <= 8);
        ASSERT_EQ(constr_id, 0);
        ASSERT_EQ(model.num_constraints(), 1);
        ASSERT_EQ(model.num_entries(), 3);

        auto constr = model.constraint(constr_id);
        ASSERT_EQ(linear_constraint_lower_bound(constr),
                  M::solver_traits::minus_infinity);
        ASSERT_EQ(linear_constraint_upper_bound(constr), 8);
        ASSERT_CONSTRAINT(constr, {2, 1, 0}, {-1.0, 1.0, 3.0},
                          M::solver_traits::minus_infinity, 8);
    }
    static void add_to_objective_xsum() {
        using namespace fhamonic::mippp::operators;
        M model;
        auto x_vars = model.add_variables(5, [](int i) { return 4 - i; });
        model.add_to_objective(xsum(ranges::views::iota(0, 4), [&](auto && i) {
            return 2.0 * x_vars(i);
        }));
        model.add_to_objective(xsum(ranges::views::iota(0, 4), x_vars));
    }
    static void add_constraint_xsum() {
        using namespace fhamonic::mippp::operators;
        M model;
        auto x_vars = model.add_variables(5, [](int i) { return 4 - i; });
        model.add_constraint(xsum(ranges::views::iota(0, 4), x_vars) <= 1);
        model.add_constraint(xsum(ranges::views::iota(0, 4), [&](auto && i) {
                                 return 2.0 * x_vars(i);
                             }) <= 1);
    }

    static void build_optimize() {
        using namespace fhamonic::mippp::operators;
        M model;
        auto x = model.add_variable({.lower_bound = 0, .upper_bound = 20});
        auto y = model.add_variable({.upper_bound = 12});
        model.add_to_objective(2 * x + 3 * y);
        model.add_constraint(x + y <= 30);

        S solver(model);

        ASSERT_EQ(solver.optimize(), S::ret_code::success);
        std::vector<double> solution = solver.get_solution();

        ASSERT_EQ(solution[static_cast<std::size_t>(x.id())], 18);
        ASSERT_EQ(solution[static_cast<std::size_t>(y.id())], 12);

        ASSERT_EQ(solver.get_solution_value(), 72.0);
    }
};
#endif  // MODEL_TESTS_HELPER_HPP