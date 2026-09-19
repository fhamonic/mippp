#pragma once

#undef NDEBUG
#include <gtest/gtest.h>

#include <cstdlib>
#include <optional>
#include <string>
#include <string_view>

#define INSTANTIATE_TEST(model_name, test_suite, model_type) \
    INSTANTIATE_TYPED_TEST_SUITE_P(model_name, test_suite,   \
                                   ::testing::Types<model_type>)

#define TEST_EPSILON 1e-6
#define TEST_INFINITY 1e20
// an optional constraint, for the add_constraints lambdas of the suites
#define OPT(cond, ...) ((cond) ? std::make_optional(__VA_ARGS__) : std::nullopt)

#include "mippp/detail/solver_library.hpp"
#include "mippp/utility/solver_exceptions.hpp"

// Solvers named in MIPPP_REQUIRED_SOLVERS (';'-separated, e.g. "CLP;CBC;GLPK")
// are ones the caller installed on purpose — CI. An api that fails to
// construct is then a packaging bug rather than an absent solver, so the run
// must fail instead of silently skipping every test of that backend.
// The names are the keys of the MIPPP_<key>_LIBRARY variables: CBC, CLP, COPT,
// CPLEX, GLPK, GUROBI, HIGHS, MOSEK, SCIP, SOPLEX and XPRESS.
inline bool is_required_solver(std::string_view solver_key) {
    const char * list = std::getenv("MIPPP_REQUIRED_SOLVERS");
    if(list == nullptr) return false;
    for(std::string_view rest(list); !rest.empty();) {
        const auto separator = rest.find(';');
        if(rest.substr(0, separator) == solver_key) return true;
        if(separator == std::string_view::npos) break;
        rest.remove_prefix(separator + 1);
    }
    return false;
}

// The compatibility matrix runs each backend's suites against every release
// it can obtain, so a release that passes but lies outside
// Api::validated_versions, or one inside that fails, fails this test on that
// row. A library reporting no version cannot be checked and skips.
#define MIPPP_API_VERSION_TEST(prefix, Api, solver_key)                   \
    TEST(prefix, loaded_release_is_a_validated_one) {                     \
        const Api * api = nullptr;                                        \
        try {                                                             \
            api = &Api::load();                                           \
        } catch(const std::exception & e) {                               \
            if(is_required_solver(solver_key)) FAIL() << e.what();        \
            GTEST_SKIP() << e.what();                                     \
        }                                                                 \
        if(!api->library_version())                                       \
            GTEST_SKIP() << api->library_path() << " reports no version"; \
        EXPECT_TRUE(mippp::detail::is_validated(Api::validated_versions,  \
                                                *api->library_version())) \
            << "loaded " << api->library_path() << " reporting "          \
            << mippp::to_string(*api->library_version());                 \
    }

template <typename Api, typename Model>
struct model_test : public ::testing::Test {
    using model_type = Model;
    inline static const Api * api = nullptr;
    // Why a *required* api is missing: the suite must then not be skipped, so
    // the failure is reported by SetUp(), see below.
    inline static std::string missing_required_api;

    template <typename... Args>
    static void construct_api(const char * solver_key, Args... args) {
        if(api != nullptr) return;
        try {
            api = &Api::load(args...);
        } catch(const std::exception & e) {
            if(is_required_solver(solver_key)) {
                missing_required_api =
                    std::string(solver_key) +
                    " is listed in MIPPP_REQUIRED_SOLVERS but its api could "
                    "not be constructed: " +
                    e.what();
                return;
            }
            GTEST_SKIP() << e.what();
        }
    }
    // A required solver must fail per test rather than from construct_api():
    // GoogleTest skips a whole suite whose SetUpTestSuite() failed, and ctest
    // turns any "[  SKIPPED ]" back into a pass (gtest_discover_tests sets
    // SKIP_REGULAR_EXPRESSION), which would swallow the failure.
    void SetUp() override {
        if(!missing_required_api.empty()) FAIL() << missing_required_api;
    }

    auto new_model() const { return Model(*api); }

    template <typename F>
    void SkipOnLicenseError(F && f) {
        try {
            f();
        } catch(const mippp::license_error & e) {
            GTEST_SKIP() << e.what();
        }
    }
};

#include "add_column.hpp"
#include "candidate_solution_callback.hpp"
#include "column_manager.hpp"
#include "cutting_stock.hpp"
#include "dual_solution.hpp"
#include "lazy_constraints.hpp"
#include "lp_fuzzy_tests.hpp"
#include "lp_model.hpp"
#include "lp_status.hpp"
#include "milp_model.hpp"
#include "mip_start.hpp"
#include "modifiable_objective.hpp"
#include "modifiable_variables_bounds.hpp"
#include "named_constraints.hpp"
#include "named_variables.hpp"
#include "qp_model.hpp"
#include "ranged_constraints.hpp"
#include "readable_constraint_bounds.hpp"
#include "readable_constraints.hpp"
#include "readable_objective.hpp"
#include "readable_quadratic_objective.hpp"
#include "readable_variables_bounds.hpp"
#include "reduced_costs.hpp"
#include "remove_variable.hpp"
#include "sudoku.hpp"
#include "time_limit.hpp"
#include "travelling_salesman.hpp"