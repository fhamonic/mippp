// The code of docs/solving/progress.md. The page shows the sections between the
// --8<-- markers, so they stay plain user code; the tests below them check what
// the page says about them.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <ostream>
#include <ranges>
#include <sstream>
#include <vector>

#include "mippp/solvers/glpk/all.hpp"
// --8<-- [start:includes]
#include "mippp/solvers/highs/all.hpp"
// --8<-- [end:includes]

using namespace mippp;

#include "test_suites/all.hpp"

namespace progress_page {

// --8<-- [start:report]
template <typename Model>
    requires has_progress_callback<Model>
void solve_with_report(Model & model, std::chrono::milliseconds every,
                       std::ostream & out) {
    auto last = std::chrono::steady_clock::now() - every;
    model.set_progress_callback([&](auto & handle) {
        const auto now = std::chrono::steady_clock::now();
        if(now - last < every) return;
        last = now;
        out << "incumbent " << handle.get_solution_value() << ", bound "
            << handle.get_dual_bound();
        if constexpr(has_node_count<progress_callback_handle_t<Model>>)
            out << ", " << handle.get_node_count() << " nodes";
        out << '\n';
    });
    model.solve();
}
// --8<-- [end:report]

// --8<-- [start:target]
// Stops as soon as the incumbent reaches a target, which +inf never does.
template <typename Model>
    requires has_progress_callback<Model>
void minimize_down_to(Model & model, double target) {
    model.set_progress_callback([target](auto & handle) {
        if(handle.get_solution_value() <= target) handle.interrupt();
    });
    model.solve();
}
// --8<-- [end:target]

}  // namespace progress_page

namespace {

template <typename Model>
void build_knapsack(Model & model) {
    using namespace mippp::operators;
    const std::size_t rows = 3, cols = 16;
    std::vector<std::vector<double>> a(rows, std::vector<double>(cols));
    std::vector<double> c(cols), b(rows);
    for(std::size_t i = 0; i < rows; ++i) {
        for(std::size_t j = 0; j < cols; ++j) {
            a[i][j] =
                static_cast<double>(1 + (i * 37 + j * 91 + i * j * 13) % 100);
            c[j] += a[i][j];
            b[i] += a[i][j] / 2;
        }
    }
    auto x = model.add_binary_variables(cols);
    const auto col_ids = std::views::iota(std::size_t{0}, cols);
    model.set_minimization();
    model.set_objective(xsum(col_ids, [&](auto j) { return (-c[j]) * x(j); }));
    for(std::size_t i = 0; i < rows; ++i)
        model.add_constraint(xsum(col_ids, [&, i](auto j) {
                                 return a[i][j] * x(j);
                             }) <= std::floor(b[i]));
}

}  // namespace

struct progress_page_highs_milp : model_test<highs_api, highs_milp> {
    static void SetUpTestSuite() { construct_api("HIGHS"); }
};
struct progress_page_glpk_milp : model_test<glpk_api, glpk_milp> {
    static void SetUpTestSuite() { construct_api("GLPK"); }
};

template <typename Fixture>
void check_report(Fixture & fixture) {
    fixture.SkipOnLicenseError([&]() {
        auto model = fixture.new_model();
        build_knapsack(model);
        std::ostringstream out;
        progress_page::solve_with_report(model, std::chrono::milliseconds(0),
                                         out);
        ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
        ASSERT_NE(out.str().find("incumbent "), std::string::npos);
        ASSERT_NE(out.str().find(", bound "), std::string::npos);
    });
}
template <typename Fixture>
void check_target(Fixture & fixture) {
    fixture.SkipOnLicenseError([&]() {
        auto reference = fixture.new_model();
        build_knapsack(reference);
        reference.solve();
        const double optimum = reference.get_solution_value();

        auto model = fixture.new_model();
        build_knapsack(model);
        const double target = 0.5 * optimum;
        progress_page::minimize_down_to(model, target);
        ASSERT_TRUE(is_a<status::interrupted>(model.get_status()));
        ASSERT_LE(model.get_solution_value(), target + TEST_EPSILON);
    });
}

TEST_F(progress_page_highs_milp, report_shows_the_search) {
    check_report(*this);
}
TEST_F(progress_page_glpk_milp, report_shows_the_search) {
    check_report(*this);
}
TEST_F(progress_page_highs_milp, a_target_ends_the_solve) {
    check_target(*this);
}
TEST_F(progress_page_glpk_milp, a_target_ends_the_solve) {
    check_target(*this);
}
