#pragma once

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdlib>
#include <exception>
#include <optional>
#include <string>
#include <string_view>

#define INSTANTIATE_TEST(model_name, test_suite, model_type) \
    INSTANTIATE_TYPED_TEST_SUITE_P(model_name, test_suite,   \
                                   ::testing::Types<model_type>)

#define TEST_EPSILON 1e-6
// an optional constraint, for the add_constraints lambdas of the suites
#define OPT(cond, ...) ((cond) ? std::make_optional(__VA_ARGS__) : std::nullopt)

#include "mippp/detail/solver_library.hpp"
#include "mippp/utility/solver_exceptions.hpp"

// Solvers named in MIPPP_REQUIRED_SOLVERS (';'-separated, e.g. "CLP;CBC;GLPK")
// are ones the caller installed on purpose — CI. An api that fails to
// construct is then a packaging bug rather than an absent solver, so the run
// must fail instead of silently skipping every test of that backend. A license
// the solver refuses still skips, whether the api or the first model meets it.
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

// Opens the reason of every skip that stops a whole backend, so that the
// compatibility matrix can tell a solver that refused to run from tests that
// skip on their own: misc/tools/compat_matrix.py matches it verbatim.
inline constexpr std::string_view backend_unavailable = "backend unavailable: ";
// Opens the reason of a skip for a member the loaded release lacks, which
// makes the release partially supported; compat_matrix.py matches it too.
inline constexpr std::string_view release_lacks = "release lacks: ";

// "1.14:1.16,2:3": the half-open ranges of a claim, as compat_matrix.py
// reads them back from the properties of the version test
template <std::size_t N>
std::string claim_text(
    const std::array<mippp::solver_version_range, N> & ranges) {
    std::string text;
    for(const auto & r : ranges) {
        if(!text.empty()) text += ',';
        text += mippp::to_string(r.from) + ':' + mippp::to_string(r.before);
    }
    return text;
}

// The compatibility matrix runs each backend's suites against every release
// it can obtain, so a release outside both Api::validated_versions and
// Api::supported_versions fails this test on that row. Both claims, and the
// tier of the release, are recorded as properties for the matrix to check
// against what the row's other tests did. A library reporting no version
// cannot be checked and skips, and so does a license refused while the api
// loads, as model_test does with it.
#define MIPPP_API_VERSION_TEST(prefix, Api, solver_key)                   \
    TEST(prefix, loaded_release_is_a_supported_one) {                     \
        RecordProperty("validated_versions",                              \
                       claim_text(Api::validated_versions));              \
        RecordProperty("supported_versions",                              \
                       claim_text(Api::supported_ranges()));              \
        const Api * api = nullptr;                                        \
        try {                                                             \
            api = &Api::load();                                           \
        } catch(const mippp::license_error & e) {                         \
            GTEST_SKIP() << backend_unavailable << e.what();              \
        } catch(const std::exception & e) {                               \
            if(is_required_solver(solver_key)) FAIL() << e.what();        \
            GTEST_SKIP() << backend_unavailable << e.what();              \
        }                                                                 \
        if(!api->library_version())                                       \
            GTEST_SKIP() << api->library_path() << " reports no version"; \
        const auto support = Api::support_of(api->library_version());     \
        RecordProperty("release_support", mippp::to_string(support));     \
        EXPECT_NE(support, mippp::release_support::untested)              \
            << "loaded " << api->library_path() << " reporting "          \
            << mippp::to_string(*api->library_version()) << ", outside "  \
            << mippp::to_string(Api::supported_ranges());                 \
    }

template <typename Api, typename Model>
struct model_test : public ::testing::Test {
    using model_type = Model;
    inline static const Api * api = nullptr;
    // Why a *required* api is missing: the suite must then not be skipped, so
    // the failure is reported by SetUp(), see below.
    inline static std::string missing_required_api;
    // Why the backend cannot run in this process: its library could not be
    // loaded, or the solver refused its license, which most solvers only
    // check when a model is created.
    inline static std::string unavailable_reason;

    // Decided once per process, from SetUpTestSuite(): a backend that cannot
    // run then costs one library search and one model, not one per test.
    template <typename... Args>
    static void construct_api(const char * solver_key, Args... args) {
        if(api != nullptr || !missing_required_api.empty() ||
           !unavailable_reason.empty())
            return;
        try {
            api = &Api::load(args...);
        } catch(const mippp::license_error & e) {
            // only the loading of a required solver is asserted, not its
            // license
            unavailable_reason = e.what();
            return;
        } catch(const std::exception & e) {
            if(is_required_solver(solver_key))
                missing_required_api =
                    std::string(solver_key) +
                    " is listed in MIPPP_REQUIRED_SOLVERS but its api could "
                    "not be constructed: " +
                    e.what();
            else
                unavailable_reason = e.what();
            return;
        }
        try {
            [[maybe_unused]] const Model probe(*api);
        } catch(const mippp::license_error & e) {
            unavailable_reason = e.what();
        } catch(...) {
            // any other error is reported by each test that creates a model
        }
    }
    // Every test of a backend that cannot run stops here, before touching the
    // solver. Skipping in SetUp() rather than in SetUpTestSuite() keeps the
    // reason in each test's result, which a suite-level skip leaves empty,
    // and a required solver fails here because GoogleTest reports every test
    // of a suite whose SetUpTestSuite() failed as skipped.
    void SetUp() override {
        if(!missing_required_api.empty()) FAIL() << missing_required_api;
        if(!unavailable_reason.empty())
            GTEST_SKIP() << backend_unavailable << unavailable_reason;
    }

    auto new_model() const { return Model(*api); }

    // The shared suites run their bodies through this. A license refused by
    // a solve skips; so does a member the loaded release lacks, which keeps
    // the suites the same for every solver, except on a release the backend
    // claims in full, where it is a regression. A library reporting no
    // version (SoPlex) skips there too: only the compatibility matrix, by
    // package version, holds it to its claim.
    template <typename F>
    void SkipOnLicenseError(F && f) {
        try {
            f();
        } catch(const mippp::license_error & e) {
            GTEST_SKIP() << e.what();
        } catch(const mippp::feature_unavailable_error & e) {
            if(Api::support_of(api->library_version()) ==
               mippp::release_support::validated)
                FAIL() << "a validated release lacks a member: " << e.what();
            GTEST_SKIP() << release_lacks << e.what();
        }
    }
};

#include "add_column.hpp"
#include "candidate_solution_callback.hpp"
#include "column_manager.hpp"
#include "cutting_stock.hpp"
#include "dual_solution.hpp"
#include "enumerable_entities.hpp"
#include "iis.hpp"
#include "iis_by_deletion.hpp"
#include "integrality_tolerance.hpp"
#include "iteration_limit.hpp"
#include "lazy_constraints.hpp"
#include "lp_fuzzy_tests.hpp"
#include "lp_model.hpp"
#include "lp_status.hpp"
#include "milp_model.hpp"
#include "mip_gap.hpp"
#include "mip_start.hpp"
#include "modifiable_constraint_bounds.hpp"
#include "modifiable_objective.hpp"
#include "modifiable_variables_bounds.hpp"
#include "named_constraints.hpp"
#include "named_variables.hpp"
#include "optimality_tolerance.hpp"
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
#include "time_limit_incumbent.hpp"
#include "travelling_salesman.hpp"
#include "verbosity.hpp"