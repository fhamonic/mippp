#pragma once
#undef NDEBUG
#include <gtest/gtest.h>

#define INSTANTIATE_TEST(model_name, test_suite, model_type) \
    INSTANTIATE_TYPED_TEST_SUITE_P(model_name, test_suite,   \
                                   ::testing::Types<model_type>)

#define TEST_EPSILON 1e-7
#define TEST_INFINITY 1e20

template <typename Api, typename Model>
struct model_test : public ::testing::Test {
    using model_type = Model;
    inline static std::optional<const Api> api;
    static void construct_api() {
        if(api.has_value()) return;
        try {
            api.emplace();
        } catch(const std::exception & e) {
            api.reset();
            GTEST_SKIP() << e.what();
        }
    }
    template <typename... Args>
    static void construct_api(Args... args) {
        if(api.has_value()) return;
        try {
            api.emplace(args...);
        } catch(const std::exception & e) {
            api.reset();
            GTEST_SKIP() << e.what();
        }
    }
    auto new_model() const { return Model(api.value()); }
};

#include "candidate_solution_callback.hpp"
#include "column_generation.hpp"
#include "dual_solution.hpp"
#include "lp_model.hpp"
#include "lp_status.hpp"
#include "milp_model.hpp"
#include "modifiable_objective.hpp"
#include "modifiable_variables_bounds.hpp"
#include "named_variables.hpp"
#include "readable_objective.hpp"
#include "readable_variables_bounds.hpp"
#include "sudoku.hpp"