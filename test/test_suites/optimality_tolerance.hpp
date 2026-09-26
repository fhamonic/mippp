#pragma once

#include <gtest/gtest.h>

#include "mippp/model_concepts.hpp"

namespace mippp {

template <typename T>
struct OptimalityToleranceTest : public T {
    using typename T::model_type;
    static_assert(lp_model<model_type>);
    static_assert(has_optimality_tolerance<model_type>);
};
TYPED_TEST_SUITE_P(OptimalityToleranceTest);
GTEST_ALLOW_UNINSTANTIATED_PARAMETERIZED_TEST(OptimalityToleranceTest);

// Setting the other tolerances in between catches two setters writing one
// solver parameter.
TYPED_TEST_P(OptimalityToleranceTest, set_get_optimality_tolerance) {
    using model_type = typename TestFixture::model_type;
    this->SkipOnLicenseError([this]() {
        auto model = this->new_model();
        for(double tol : {1e-6, 1e-3, 0.05}) {
            model.set_optimality_tolerance(tol);
            if constexpr(has_feasibility_tolerance<model_type>)
                model.set_feasibility_tolerance(1e-8);
            if constexpr(has_integrality_tolerance<model_type>)
                model.set_integrality_tolerance(1e-7);
            ASSERT_NEAR(model.get_optimality_tolerance(), tol, 1e-9 * tol);
        }
    });
}

REGISTER_TYPED_TEST_SUITE_P(OptimalityToleranceTest,
                            set_get_optimality_tolerance);

}  // namespace mippp
