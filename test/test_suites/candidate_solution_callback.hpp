#pragma once
#undef NDEBUG
#include <gtest/gtest.h>

#include "mippp/linear_constraint.hpp"
#include "mippp/model_concepts.hpp"

namespace fhamonic::mippp {

template <typename T>
struct CandidateSolutionCallbackTest : public T {
    using typename T::model_type;
    static_assert(has_candidate_solution_callback<model_type>);
};
TYPED_TEST_SUITE_P(CandidateSolutionCallbackTest);

TYPED_TEST_P(CandidateSolutionCallbackTest, lp_behavior) {
    using namespace operators;
    auto model = this->new_model();
    auto x1 = model.add_variable();
    auto x2 = model.add_variable();
    auto x3 = model.add_variable();
    model.set_maximization();
    model.set_objective(5 * x1 + 4 * x2 + 3 * x3);
    auto c1 = model.add_constraint(2 * x1 + 2 * x2 - x3 >= 5);
    auto c2 = model.add_constraint(4 * x1 + x2 + 2 * x3 <= 11);
    auto c3 = model.add_constraint(3 * x1 + 4 * x2 + 2 * x3 == 8);
    model.set_candidate_solution_callback([](auto & handle) {

    });
    try {
        model.solve();
        // TODO
    } catch(const std::runtime_error & e) {
    }
}

REGISTER_TYPED_TEST_SUITE_P(CandidateSolutionCallbackTest, lp_behavior);

}  // namespace fhamonic::mippp