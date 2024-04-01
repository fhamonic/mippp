#include <gtest/gtest.h>

#include "mippp/constraints/linear_constraint_operators.hpp"
#include "mippp/expressions/linear_expression_operators.hpp"
#include "mippp/mip_model.hpp"
#include "mippp/xsum.hpp"

#include "assert_eq_ranges.hpp"

#include "model_tests_helper.hpp"

using namespace fhamonic::mippp;

using Var = variable<int, double>;
using T = cli_solver_model_traits;
using M = mip_model<T>;
using S = cli_grb_solver;

GTEST_TEST(cli_grb_model, default_constructor) {
    if(!S::is_available()) GTEST_SKIP();
    model_tests<M, S>::default_constructor();
}
GTEST_TEST(cli_grb_model, add_variable_default_name_default_options) {
    if(!S::is_available()) GTEST_SKIP();
    model_tests<M, S>::add_variable_default_name_default_options();
}
GTEST_TEST(cli_grb_model, add_variable_default_name_custom_options) {
    if(!S::is_available()) GTEST_SKIP();
    model_tests<M, S>::add_variable_default_name_custom_options();
}
GTEST_TEST(cli_grb_model, add_variable_custom_name_default_options) {
    if(!S::is_available()) GTEST_SKIP();
    model_tests<M, S>::add_variable_custom_name_default_options();
}
GTEST_TEST(cli_grb_model, add_variable_custom_name_custom_options) {
    if(!S::is_available()) GTEST_SKIP();
    model_tests<M, S>::add_variable_custom_name_custom_options();
}
GTEST_TEST(cli_grb_model, add_variables_default_name_default_options) {
    if(!S::is_available()) GTEST_SKIP();
    model_tests<M, S>::add_variables_default_name_default_options();
}
GTEST_TEST(cli_grb_model, add_variables_default_name_custom_options) {
    if(!S::is_available()) GTEST_SKIP();
    model_tests<M, S>::add_variables_default_name_custom_options();
}
GTEST_TEST(cli_grb_model, add_variables_custom_name_default_options) {
    if(!S::is_available()) GTEST_SKIP();
    model_tests<M, S>::add_variables_custom_name_default_options();
}
GTEST_TEST(cli_grb_model, add_variables_custom_name_custom_options) {
    if(!S::is_available()) GTEST_SKIP();
    model_tests<M, S>::add_variables_custom_name_custom_options();
}
GTEST_TEST(cli_grb_model, add_to_objective) {
    if(!S::is_available()) GTEST_SKIP();
    model_tests<M, S>::add_to_objective();
}
GTEST_TEST(cli_grb_model, get_objective) {
    if(!S::is_available()) GTEST_SKIP();
    model_tests<M, S>::get_objective();
}

GTEST_TEST(cli_grb_model, add_and_get_constraint) {
    if(!S::is_available()) GTEST_SKIP();
    model_tests<M, S>::add_and_get_constraint();
}
GTEST_TEST(cli_grb_model, build_optimize) {
    if(!S::is_available()) GTEST_SKIP();
    model_tests<M, S>::build_optimize();
}
GTEST_TEST(cli_grb_model, add_to_objective_xsum) {
    if(!S::is_available()) GTEST_SKIP();
    model_tests<M, S>::add_to_objective_xsum();
}