#include <gtest/gtest.h>

#include "mippp/model.hpp"

#include "assert_eq_ranges.hpp"

using namespace fhamonic::mippp;

using Var = variable<int, double>;

GTEST_TEST(model, ctor) {
    Model<GrbTraits> model;
    
}
