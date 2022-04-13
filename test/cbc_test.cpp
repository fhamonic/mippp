#include <gtest/gtest.h>

#include "mippp/model.hpp"

#include "assert_eq_ranges.hpp"

using namespace fhamonic::mippp;

using Var = variable<int, double>;

GTEST_TEST(cbc_model, ctor) {
    Model<CbcTraits> model;
}

GTEST_TEST(cbc_model, add_var) {
    Model<CbcTraits> model;
    auto x = model.add_var();
    ASSERT_EQ(x.id(), 0);
}

GTEST_TEST(cbc_model, add_var_default_options) {
    Model<CbcTraits> model;
    auto x = model.add_var();
    ASSERT_EQ(x.id(), 0);
    ASSERT_EQ(model.obj_coef(x), 0);
    ASSERT_EQ(model.lower_bound(x), 0);
    ASSERT_EQ(model.upper_bound(x), Model<CbcTraits>::INFTY);
    ASSERT_EQ(model.type(x), Model<CbcTraits>::ColType::CONTINUOUS);
}

GTEST_TEST(cbc_model, add_var_custom_options) {
    Model<CbcTraits> model;
    auto x = model.add_var({.obj_coef=6.14, .lower_bound=3.2, .upper_bound=9, .type=Model<CbcTraits>::ColType::BINARY});
    ASSERT_EQ(x.id(), 0);
    ASSERT_EQ(model.obj_coef(x), 6.14);
    ASSERT_EQ(model.lower_bound(x), 3.2);
    ASSERT_EQ(model.upper_bound(x), 9);
    ASSERT_EQ(model.type(x), Model<CbcTraits>::ColType::BINARY);
}

GTEST_TEST(cbc_model, add_vars) {
    Model<CbcTraits> model;
    auto x_vars = model.add_vars(5, [](int i){ return 4-i;});
    for(int i=0; i<5; ++i) {
        ASSERT_EQ(x_vars(i).id(), 4-i);
        ASSERT_EQ(model.obj_coef(x_vars(i)), 0);
        ASSERT_EQ(model.lower_bound(x_vars(i)), 0);
        ASSERT_EQ(model.upper_bound(x_vars(i)), Model<CbcTraits>::INFTY);
        ASSERT_EQ(model.type(x_vars(i)), Model<CbcTraits>::ColType::CONTINUOUS);
    }
}
