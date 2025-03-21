#include <gtest/gtest.h>

#include "assert_eq_ranges.hpp"

#include "mippp/model/cbc_milp_model.hpp"
#include "mippp/model/clp_lp_model.hpp"
#include "mippp/model/grb_milp_model.hpp"

using namespace fhamonic::mippp;

struct clp_lp_model_test {
    clp_api api;
    auto construct_model() const { return clp_lp_model(api); }
};
struct cbc_milp_model_test {
    cbc_api api;
    auto construct_model() const { return cbc_milp_model(api); }
};
struct grb_lp_model_test {
    grb_api api;
    auto construct_model() const { return grb_milp_model(api); }
};

using Models =
    ::testing::Types<clp_lp_model_test, cbc_milp_model_test, grb_lp_model_test>;

template <typename T>
class ModelTest : public ::testing::Test {
private:
    clp_lp_model_test m;

public:
    auto construct_model() const { return m.construct_model(); }
};

TYPED_TEST_SUITE(ModelTest, Models);

TYPED_TEST(ModelTest, add_variable) {
    auto model = this->construct_model();
    auto x = model.add_variable();
    auto y = model.add_variable();
    ASSERT_EQ(model.num_variables(), 2);
    ASSERT_EQ(x.id(), 0);
    ASSERT_EQ(y.id(), 1);

    ASSERT_EQ(model.get_objective_coefficient(x), 0.0);
    ASSERT_EQ(model.get_variable_lower_bound(x), 0.0);
    // ASSERT_EQ(model.get_variable_upper_bound(x), M::infinity);
    ASSERT_EQ(model.get_objective_coefficient(y), 0.0);
    ASSERT_EQ(model.get_variable_lower_bound(y), 0.0);
    // ASSERT_EQ(model.get_variable_upper_bound(y), M::infinity);
}
TYPED_TEST(ModelTest, add_variable_w_params) {
    auto model = this->construct_model();
    auto x = model.add_variable({.obj_coef = 7.0, .upper_bound = 13.0});
    auto y = model.add_variable({.lower_bound = 2});
    ASSERT_EQ(model.num_variables(), 2);
    ASSERT_EQ(x.id(), 0);
    ASSERT_EQ(y.id(), 1);

    ASSERT_EQ(model.get_objective_coefficient(x), 7.0);
    ASSERT_EQ(model.get_variable_lower_bound(x), 0.0);
    ASSERT_EQ(model.get_variable_upper_bound(x), 13.0);
    ASSERT_EQ(model.get_objective_coefficient(y), 0.0);
    ASSERT_EQ(model.get_variable_lower_bound(y), 2.0);
    // ASSERT_EQ(model.get_variable_upper_bound(y), M::infinity);
}
TYPED_TEST(ModelTest, set_variable_bounds) {
    GTEST_SKIP();
    auto model = this->construct_model();
    auto x = model.add_variable({.obj_coef = 7.0, .upper_bound = 13.0});
    auto y = model.add_variable({.lower_bound = 2});
    ASSERT_EQ(model.num_variables(), 2);
    ASSERT_EQ(x.id(), 0);
    ASSERT_EQ(y.id(), 1);

    ASSERT_EQ(model.get_objective_coefficient(x), 7.0);
    ASSERT_EQ(model.get_variable_lower_bound(x), 0.0);
    ASSERT_EQ(model.get_variable_upper_bound(x), 13.0);
    ASSERT_EQ(model.get_objective_coefficient(y), 0.0);
    ASSERT_EQ(model.get_variable_lower_bound(y), 2.0);
    // ASSERT_EQ(model.get_variable_upper_bound(y), M::infinity);
}