#pragma once

#define INSTANTIATE_TEST(model_name, test_suite, model_type) \
    INSTANTIATE_TYPED_TEST_SUITE_P(model_name, test_suite,   \
                                   ::testing::Types<model_type>)

#define TEST_EPSILON 1e-10
#define TEST_INFINITY 1e20

#include "lp_model.hpp"
#include "milp_model.hpp"
#include "readable_objective.hpp"
#include "readable_variables_bounds.hpp"
#include "sudoku.hpp"