// The code of docs/algorithms/branch-and-cut.md. The page shows the sections
// between the --8<-- markers, so they stay plain user code; the tests below
// them check what the page says about them. No open-source backend has what
// the sections need, so the tests run on Gurobi, in the builds that compile
// its tests (MIPPP_TEST_WITH_GUROBI, see test/CMakeLists.txt).

#include <gtest/gtest.h>

#include <cstddef>
#include <ranges>
#include <type_traits>

#include "mippp/model_concepts.hpp"

#ifdef MIPPP_TEST_WITH_GUROBI
#include "mippp/solvers/gurobi/all.hpp"
#endif

using namespace mippp;

#include "test_suites/all.hpp"

namespace branch_and_cut_page {

// --8<-- [start:repair]
// For a minimization whose variable `eta` stands for a cost the model does not
// state. `cost_of(point)` is that cost at the integer part of a point, and
// `cut_at(point)` the constraint `eta >= ...` that says so, tight at the point.
template <typename Model, typename CostOf, typename CutAt>
    requires has_solution_injection<candidate_solution_callback_handle_t<Model>,
                                    Model>
void set_benders_callback(Model & model, model_variable_t<Model> eta,
                          CostOf cost_of, CutAt cut_at) {
    model.set_candidate_solution_callback([=](auto & handle) {
        auto point = handle.get_solution();
        const auto cost = cost_of(point);
        if(point[eta] >= cost - 1e-6) return;  // priced right: accepted
        handle.add_lazy_constraint(cut_at(point));
        point[eta] = cost;  // the same integer part, at its true cost
        handle.add_solution(point);
    });
}
// --8<-- [end:repair]

// --8<-- [start:both]
// `separator(point, add)` calls `add(constraint)` on every constraint of its
// family that the point violates.
template <typename Model, typename Separator>
    requires has_candidate_solution_callback<Model> &&
             has_node_relaxation_callback<Model>
void set_separator(Model & model, Separator separator) {
    const auto separate = [=](auto & handle) {
        using handle_type = std::remove_cvref_t<decltype(handle)>;
        separator(handle.get_solution(), [&](auto && constraint) {
            if constexpr(has_user_cuts<handle_type, Model>)
                handle.add_user_cut(constraint);
            else
                handle.add_lazy_constraint(constraint);
        });
    };
    model.set_candidate_solution_callback(separate);  // feasibility
    model.set_node_relaxation_callback(separate);     // strength
}
// --8<-- [end:both]

}  // namespace branch_and_cut_page

#ifdef MIPPP_TEST_WITH_GUROBI

namespace {

struct branch_and_cut_page_test : model_test<gurobi_api, gurobi_milp> {
    static void SetUpTestSuite() { construct_api("GUROBI"); }
};

}  // namespace

TEST_F(branch_and_cut_page_test, the_benders_callback_reaches_the_optimum) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        using location = mippp::detail::epigraph_location;
        auto model = this->new_model();
        auto Y = model.add_binary_variables(location::num_facilities);
        auto eta = model.add_variable();
        const auto facilities =
            std::views::iota(std::size_t{0}, location::num_facilities);
        model.set_minimization();
        model.set_objective(xsum(facilities,
                                 [&](const std::size_t i) {
                                     return location::opening_cost(i) * Y(i);
                                 }) +
                            eta);
        model.add_constraint(xsum(facilities, Y) >= 1);

        const auto open_at = [=](const auto & point) {
            location::facility_set open;
            for(const std::size_t i : facilities) open[i] = point[Y(i)] > 0.5;
            return open;
        };
        int num_cuts = 0;
        branch_and_cut_page::set_benders_callback(
            model, eta,
            [=](const auto & point) {
                return location::assignment_cost(open_at(point));
            },
            [=, &num_cuts](const auto & point) {
                using namespace operators;
                ++num_cuts;
                const auto open = open_at(point);
                return eta +
                           xsum(facilities,
                                [=](const std::size_t i) {
                                    return location::cut_coefficient(open, i) *
                                           Y(i);
                                }) >=
                       location::assignment_cost(open);
            });
        model.solve();

        ASSERT_GE(num_cuts, 1);
        ASSERT_TRUE(is<status::optimal>(model.get_status()));
        ASSERT_NEAR(model.get_solution_value(), location::optimum(),
                    TEST_EPSILON);
    });
}

TEST_F(branch_and_cut_page_test, one_separator_eliminates_subtours) {
    this->SkipOnLicenseError([this]() {
        using namespace operators;
        using cities = mippp::detail::eight_cities;
        auto model = this->new_model();
        auto x = cities::build(model);
        int num_rows = 0;
        branch_and_cut_page::set_separator(
            model, [&, x](const auto & point, auto && add) {
                using namespace operators;
                for(const auto & piece : cities::pieces(
                        [&](const std::size_t a) { return point[x(a)]; })) {
                    ++num_rows;
                    add(xsum(cities::arcs_inside(piece), x) <=
                        static_cast<double>(piece.size()) - 1);
                }
            });
        model.solve();

        ASSERT_GE(num_rows, 1);
        ASSERT_TRUE(is<status::optimal>(model.get_status()));
        ASSERT_NEAR(model.get_solution_value(), cities::shortest_tour(),
                    TEST_EPSILON);
    });
}

#endif  // MIPPP_TEST_WITH_GUROBI
