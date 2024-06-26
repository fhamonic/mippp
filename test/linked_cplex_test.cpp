#include <gtest/gtest.h>

#include "mippp/constraints/linear_constraint_operators.hpp"
#include "mippp/expressions/linear_expression_operators.hpp"
#include "mippp/mip_model.hpp"
#include "mippp/xsum.hpp"

#include "assert_eq_ranges.hpp"

#include "model_tests_helper.hpp"

using namespace fhamonic::mippp;

using Var = variable<int, double>;
using M = mip_model<linked_cplex_model_traits>;
using S = linked_cplex_solver;

GTEST_TEST(linked_cplex_model, default_constructor) {
    model_tests<M, S>::default_constructor();
}
GTEST_TEST(linked_cplex_model, add_variable_default_name_default_options) {
    model_tests<M, S>::add_variable_default_name_default_options();
}
GTEST_TEST(linked_cplex_model, add_variable_default_name_custom_options) {
    model_tests<M, S>::add_variable_default_name_custom_options();
}
GTEST_TEST(linked_cplex_model, add_variable_custom_name_default_options) {
    model_tests<M, S>::add_variable_custom_name_default_options();
}
GTEST_TEST(linked_cplex_model, add_variable_custom_name_custom_options) {
    model_tests<M, S>::add_variable_custom_name_custom_options();
}
GTEST_TEST(linked_cplex_model, add_variables_default_name_default_options) {
    model_tests<M, S>::add_variables_default_name_default_options();
}
GTEST_TEST(linked_cplex_model, add_variables_default_name_custom_options) {
    model_tests<M, S>::add_variables_default_name_custom_options();
}
GTEST_TEST(linked_cplex_model, add_variables_custom_name_default_options) {
    model_tests<M, S>::add_variables_custom_name_default_options();
}
GTEST_TEST(linked_cplex_model, add_variables_custom_name_custom_options) {
    model_tests<M, S>::add_variables_custom_name_custom_options();
}
GTEST_TEST(linked_cplex_model, add_to_objective) {
    model_tests<M, S>::add_to_objective();
}
GTEST_TEST(linked_cplex_model, get_objective) {
    model_tests<M, S>::get_objective();
}
GTEST_TEST(linked_cplex_model, add_and_get_constraint) {
    model_tests<M, S>::add_and_get_constraint();
}
GTEST_TEST(linked_cplex_model, build_optimize) {
    model_tests<M, S>::build_optimize();
}
GTEST_TEST(linked_cplex_model, add_to_objective_xsum) {
    model_tests<M, S>::add_to_objective_xsum();
}