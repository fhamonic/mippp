#include <gtest/gtest.h>

#include "mippp/expressions/linear_expression_operations.hpp"
#include "mippp/expressions/linear_term.hpp"

#include "ranges_test_helper.hpp"

using namespace fhamonic::mippp;

GTEST_TEST(mippp_interface, test) {
    std::vector<int> vars = {1, 2};
    AssertRangesAreEqual(linear_expression_add(linear_term(1, 3.2), linear_term(2, 1.5)).variables(), vars);

}
