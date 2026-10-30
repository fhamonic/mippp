// The code of the column manager section of
// docs/algorithms/column-generation.md. The page shows the sections between
// the --8<-- markers, so they stay plain user code; the tests below them
// check the optimum the page states.

#include <gtest/gtest.h>

#include <cstddef>
#include <optional>
#include <ranges>
#include <utility>
#include <vector>

#include "melon/algorithm/unbounded_knapsack_bnb.hpp"

#include "mippp/solvers/clp/all.hpp"
#include "mippp/solvers/highs/all.hpp"
// --8<-- [start:includes]
#include "mippp/utility/column_manager.hpp"
// --8<-- [end:includes]

using namespace mippp;

#include "test_suites/all.hpp"

namespace column_generation_page {

// here rather than in each function: on MSVC a block-scope using-directive
// does not reach into a generic lambda
using namespace mippp::operators;

constexpr int roll_length = 100;
const std::vector<int> length = {14, 31, 36, 45};  // piece lengths
const std::vector<double> demand = {211, 395, 610, 97};

// the pattern of best value at these duals, if it beats the cost of a roll
std::optional<std::vector<int>> price(const std::vector<double> & value) {
    melon::unbounded_knapsack_bnb knapsack(
        std::views::iota(std::size_t{0}, length.size()),
        [&](std::size_t o) { return value[o]; }, length, roll_length);
    if(knapsack.run().solution_value() <= 1.0 + 1e-9) return std::nullopt;
    std::vector<int> pattern(length.size(), 0);
    for(auto && [o, count] : knapsack.solution_items())
        pattern[o] = static_cast<int>(count);
    return pattern;
}

template <typename Model>
double cutting_stock(Model & model) {
    const std::size_t m = length.size();
    auto orders = std::views::iota(std::size_t{0}, m);
    model.set_minimization();
    // --8<-- [start:manager]
    // a column's seed is its pattern: the number of pieces of each length
    using pattern = std::vector<int>;
    colgen::column_manager<
        Model, pattern, colgen::property_list<colgen::reduced_cost>,
        colgen::property_list<colgen::reduced_cost, colgen::age>>
        columns;
    // --8<-- [end:manager]
    // --8<-- [start:rows]
    auto demand_constrs = model.add_constraints(orders, [&](std::size_t o) {
        return empty_linear_expression<model_variable_t<Model>, double> >=
               demand[o];
    });
    auto add_pattern = [&](Model & master, const auto & entry) {
        const pattern & p = entry.seed;
        return master.add_column(
            orders | std::views::filter([&](std::size_t o) {
                return p[o] > 0;
            }) | std::views::transform([&](std::size_t o) {
                return std::make_pair(demand_constrs(o),
                                      static_cast<double>(p[o]));
            }),
            {.obj_coef = 1, .lower_bound = 0});
    };
    // a feasible start: rolls cut into pieces of a single length
    for(std::size_t o : orders) {
        pattern p(m, 0);
        p[o] = roll_length / length[o];
        columns.emplace_master_column(model, std::move(p), add_pattern);
    }
    // --8<-- [end:rows]
    // --8<-- [start:round]
    for(;;) {
        model.solve();
        // read the master before activate_columns and evict_columns modify it
        auto duals = model.get_dual_solution();
        std::vector<double> value(m);
        for(std::size_t o : orders) value[o] = duals[demand_constrs(o)];

        if(auto p = price(value)) columns.emplace_pool_column(std::move(*p));
        columns.update_columns([&](const auto & entry) {
            double rc = 1.0;  // the cost of a roll
            for(std::size_t o : orders) rc -= value[o] * entry.seed[o];
            return colgen::priced{rc};
        });
        if(columns.activate_columns(
               model, colgen::negative<colgen::reduced_cost>{1e-9},
               add_pattern) == 0)
            break;  // converged, and the master is still as solved last
        // columns in the master for more than 3 rounds, now unattractive
        columns.evict_columns(
            model, colgen::conjunction<colgen::above<colgen::age>,
                                       colgen::positive<colgen::reduced_cost>>{
                       3, 1e-6});
    }
    // --8<-- [end:round]
    return model.get_solution_value();
}

}  // namespace column_generation_page

struct column_generation_page_highs_lp : model_test<highs_api, highs_lp> {
    static void SetUpTestSuite() { construct_api("HIGHS"); }
};
struct column_generation_page_clp_lp : model_test<clp_api, clp_lp> {
    static void SetUpTestSuite() { construct_api("CLP"); }
};

template <typename Fixture>
void check_cutting_stock(Fixture & fixture) {
    fixture.SkipOnLicenseError([&]() {
        auto model = fixture.new_model();
        ASSERT_NEAR(column_generation_page::cutting_stock(model), 452.25,
                    TEST_EPSILON);
    });
}

TEST_F(column_generation_page_highs_lp, cutting_stock_reaches_the_optimum) {
    check_cutting_stock(*this);
}
TEST_F(column_generation_page_clp_lp, cutting_stock_reaches_the_optimum) {
    check_cutting_stock(*this);
}
