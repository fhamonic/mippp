#include <gtest/gtest.h>

#include "mippp/constraints/linear_constraint_operators.hpp"
#include "mippp/expressions/linear_expression_operators.hpp"
#include "mippp/mip_model.hpp"
#include "mippp/xsum.hpp"

#include "assert_eq_ranges.hpp"

#include "model_tests_helper.hpp"

using namespace fhamonic::mippp;

using Var = variable<int, double>;
using T = cli_scip_traits;
using M = mip_model<T>;

GTEST_TEST(cli_scip_model, default_constructor) {
    if(!T::is_available()) GTEST_SKIP();
    model_tests<M>::default_constructor();
}
GTEST_TEST(cli_scip_model, add_variable_default_name_default_options) {
    if(!T::is_available()) GTEST_SKIP();
    model_tests<M>::add_variable_default_name_default_options();
}
GTEST_TEST(cli_scip_model, add_variable_default_name_custom_options) {
    if(!T::is_available()) GTEST_SKIP();
    model_tests<M>::add_variable_default_name_custom_options();
}
GTEST_TEST(cli_scip_model, add_variable_custom_name_default_options) {
    if(!T::is_available()) GTEST_SKIP();
    model_tests<M>::add_variable_custom_name_default_options();
}
GTEST_TEST(cli_scip_model, add_variable_custom_name_custom_options) {
    if(!T::is_available()) GTEST_SKIP();
    model_tests<M>::add_variable_custom_name_custom_options();
}
GTEST_TEST(cli_scip_model, add_variables_default_name_default_options) {
    if(!T::is_available()) GTEST_SKIP();
    model_tests<M>::add_variables_default_name_default_options();
}
GTEST_TEST(cli_scip_model, add_variables_default_name_custom_options) {
    if(!T::is_available()) GTEST_SKIP();
    model_tests<M>::add_variables_default_name_custom_options();
}
GTEST_TEST(cli_scip_model, add_variables_custom_name_default_options) {
    if(!T::is_available()) GTEST_SKIP();
    model_tests<M>::add_variables_custom_name_default_options();
}
GTEST_TEST(cli_scip_model, add_variables_custom_name_custom_options) {
    if(!T::is_available()) GTEST_SKIP();
    model_tests<M>::add_variables_custom_name_custom_options();
}
GTEST_TEST(cli_scip_model, add_to_objective) {
    if(!T::is_available()) GTEST_SKIP();
    model_tests<M>::add_to_objective();
}
GTEST_TEST(cli_scip_model, get_objective) {
    if(!T::is_available()) GTEST_SKIP();
    model_tests<M>::get_objective();
}

GTEST_TEST(cli_scip_model, add_and_get_constraint) {
    if(!T::is_available()) GTEST_SKIP();
    model_tests<M>::add_and_get_constraint();
}
GTEST_TEST(cli_scip_model, build_optimize) {
    if(!T::is_available()) GTEST_SKIP();
    model_tests<M>::build_optimize();
}
GTEST_TEST(cli_scip_model, add_to_objective_xsum) {
    if(!T::is_available()) GTEST_SKIP();
    model_tests<M>::add_to_objective_xsum();
}