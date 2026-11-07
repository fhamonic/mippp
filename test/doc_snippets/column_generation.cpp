// The code of docs/algorithms/column-generation.md. The page shows the
// sections between the --8<-- markers, so they stay plain user code; the tests
// below them check what the page says of each.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <ranges>
#include <utility>
#include <variant>
#include <vector>

#include "mippp/solvers/clp/all.hpp"
// --8<-- [start:includes]
#include "melon/algorithm/unbounded_knapsack_bnb.hpp"
#include "mippp/solvers/highs/all.hpp"
#include "mippp/utility/column_manager.hpp"
// --8<-- [end:includes]

using namespace mippp;

#include "test_suites/all.hpp"

namespace column_generation_page {

// here rather than in each function: on MSVC a block-scope using-directive
// does not reach into a generic lambda
using namespace mippp::operators;

// what a run leaves for the tests to check
struct run {
    double value = 0.0;  // the master's optimum, read after the loop
    std::size_t master_columns = 0;
    std::size_t pool_columns = 0;
    std::size_t activations = 0;  // counted by pool_first only
    // the sums of the results of emplace_pool_columns, by several_per_round
    std::size_t inserted = 0;
    std::size_t already_in_pool = 0;
    std::size_t already_in_master = 0;
};

template <typename Model, typename Columns>
run finish(Model & model, const Columns & columns) {
    run r;
    r.value = model.get_solution_value();
    r.master_columns = columns.num_master_columns();
    r.pool_columns = columns.num_pool_columns();
    return r;
}

// what the integer endgame leaves for the tests to check
struct integer_plan {
    double rolls = 0.0;          // the optimum of the MILP
    std::vector<double> pieces;  // the pieces of each length the plan cuts
    bool integral = true;
};

// A cutting stock instance and its pricers. The functions holding the sections
// are its members, so that the sections read the page's names (length,
// demand, price) while the tests also run them on a larger instance.
struct cutting_stock {
    int roll_length;
    std::vector<int> length;  // piece lengths
    std::vector<double> demand;
    std::size_t pricer_calls = 0;  // by price and price_several

    cutting_stock(int roll, std::vector<int> lengths,
                  std::vector<double> demands)
        : roll_length(roll)
        , length(std::move(lengths))
        , demand(std::move(demands)) {}

    // the pattern of best value at these duals, if it beats the cost of a roll
    std::optional<std::vector<int>> price(const std::vector<double> & value) {
        ++pricer_calls;
        // --8<-- [start:price]
        melon::unbounded_knapsack_bnb knapsack(
            std::views::iota(std::size_t{0}, length.size()),
            [&](std::size_t o) { return value[o]; }, length, roll_length);
        if(knapsack.run().solution_value() <= 1.0 + 1e-9) return std::nullopt;
        std::vector<int> pattern(length.size(), 0);
        for(auto && [o, count] : knapsack.solution_items())
            pattern[o] = static_cast<int>(count);
        return pattern;
        // --8<-- [end:price]
    }

    // For each length, the best pattern that cuts at least one piece of it, if
    // it beats the cost of a roll, without repeating a pattern. A pattern cuts
    // some piece, so the best of them is the best pattern: a round in which
    // this finds nothing is one in which price() finds nothing.
    std::vector<std::vector<int>> price_several(
        const std::vector<double> & value) {
        ++pricer_calls;
        std::vector<std::vector<int>> patterns;
        for(std::size_t forced = 0; forced < length.size(); ++forced) {
            melon::unbounded_knapsack_bnb knapsack(
                std::views::iota(std::size_t{0}, length.size()),
                [&](std::size_t o) { return value[o]; }, length,
                roll_length - length[forced]);
            if(value[forced] + knapsack.run().solution_value() <= 1.0 + 1e-9)
                continue;
            std::vector<int> pattern(length.size(), 0);
            for(auto && [o, count] : knapsack.solution_items())
                pattern[o] = static_cast<int>(count);
            ++pattern[forced];
            if(std::ranges::find(patterns, pattern) == patterns.end())
                patterns.push_back(std::move(pattern));
        }
        return patterns;
    }

    template <typename Model>
    double classic(Model & model);
    template <typename Model>
    run first_example(Model & model);
    template <typename Model>
    run evicting(Model & model);
    template <typename Model>
    run several_per_round(Model & model);
    template <typename Model>
    run pool_first(Model & model);
    template <typename Model, typename Milp>
    integer_plan price_and_branch(Model & model, Milp & milp);
    template <typename Model>
    run nonbasic_eviction(Model & model);
    template <typename Model>
    run relative_tolerance(Model & model);
    template <typename Model>
    run with_costs(Model & model);

private:
    // the rows and the initial columns of first_example, for the functions
    // whose sections show only what they change
    template <typename Model>
    auto demand_rows(Model & model) {
        model.set_minimization();
        return model.add_constraints(
            std::views::iota(std::size_t{0}, length.size()),
            [&](std::size_t o) {
                return empty_linear_expression<model_variable_t<Model>,
                                               double> >= demand[o];
            });
    }
    template <typename Rows>
    static auto pattern_adder(Rows & rows) {
        return [&rows](auto & master, const auto & entry) {
            return master.add_column(std::views::zip(rows, entry.seed),
                                     {.obj_coef = 1, .lower_bound = 0});
        };
    }
    template <typename Model, typename Columns, typename Add,
              typename... DataArgs>
    void single_length_columns(Model & model, Columns & columns, Add & add,
                               const DataArgs &... data_args) {
        for(std::size_t o = 0; o < length.size(); ++o) {
            std::vector<int> p(length.size(), 0);
            p[o] = roll_length / length[o];
            columns.emplace_master_column(model, std::move(p), add,
                                          data_args...);
        }
    }
};

// the instance of the page
cutting_stock page_instance() {
    return {100, {14, 31, 36, 45}, {211, 395, 610, 97}};
}
// ten lengths: enough rounds for evicted columns to come back
cutting_stock larger_instance() {
    return {100,
            {12, 15, 18, 22, 25, 28, 33, 37, 41, 47},
            {150, 210, 180, 90, 250, 120, 300, 80, 140, 60}};
}

template <typename Model>
double cutting_stock::classic(Model & model) {
    // --8<-- [start:classic_master]
    const std::size_t m = length.size();
    auto orders = std::views::iota(std::size_t{0}, m);
    model.set_minimization();
    auto demand_constrs = model.add_constraints(orders, [&](std::size_t o) {
        return empty_linear_expression<model_variable_t<Model>, double> >=
               demand[o];
    });
    for(std::size_t o : orders)  // a roll cut into pieces of a single length
        model.add_column(
            {{demand_constrs(o), static_cast<double>(roll_length / length[o])}},
            {.obj_coef = 1, .lower_bound = 0});
    // --8<-- [end:classic_master]
    // --8<-- [start:classic_round]
    for(;;) {
        model.solve();
        auto duals = model.get_dual_solution();
        melon::unbounded_knapsack_bnb knapsack(
            orders, [&](std::size_t o) { return duals[demand_constrs(o)]; },
            length, roll_length);
        if(knapsack.run().solution_value() <= 1.0 + 1e-9)
            break;  // no pattern beats the cost of a new roll
        auto entries = knapsack.solution_items() |
                       std::views::transform([&](const auto & item) {
                           const auto & [o, count] = item;
                           return std::make_pair(demand_constrs(o),
                                                 static_cast<double>(count));
                       });
        model.add_column(entries, {.obj_coef = 1, .lower_bound = 0});
    }
    // --8<-- [end:classic_round]
    return model.get_solution_value();
}

template <typename Model>
run cutting_stock::first_example(Model & model) {
    // --8<-- [start:first_master]
    const std::size_t m = length.size();
    auto orders = std::views::iota(std::size_t{0}, m);
    model.set_minimization();
    using pattern = std::vector<int>;  // the number of pieces of each length
    colgen::column_manager<Model, pattern,
                           colgen::property_list<colgen::reduced_cost>>
        columns;
    auto demand_constrs = model.add_constraints(orders, [&](std::size_t o) {
        return empty_linear_expression<model_variable_t<Model>, double> >=
               demand[o];
    });
    auto add_pattern = [&](Model & master, const auto & entry) {
        return master.add_column(std::views::zip(demand_constrs, entry.seed),
                                 {.obj_coef = 1, .lower_bound = 0});
    };
    for(std::size_t o : orders) {  // a feasible start: one length per roll
        pattern p(m, 0);
        p[o] = roll_length / length[o];
        columns.emplace_master_column(model, std::move(p), add_pattern);
    }
    // --8<-- [end:first_master]
    // --8<-- [start:first_round]
    do {
        model.solve();
        auto duals = model.get_dual_solution();
        std::vector<double> value(m);
        for(std::size_t o : orders) value[o] = duals[demand_constrs(o)];
        if(auto p = price(value)) columns.emplace_pool_column(std::move(*p));
        columns.update_pool_columns([&](const auto & entry) {
            double rc = 1.0;  // the cost of a roll
            for(std::size_t o : orders) rc -= value[o] * entry.seed[o];
            return colgen::priced{rc};
        });
    } while(columns.activate_columns(
        model, colgen::negative<colgen::reduced_cost>{1e-9}, add_pattern));
    // --8<-- [end:first_round]
    return finish(model, columns);
}

template <typename Model>
run cutting_stock::evicting(Model & model) {
    const std::size_t m = length.size();
    auto orders = std::views::iota(std::size_t{0}, m);
    using pattern = std::vector<int>;
    // --8<-- [start:evicting_manager]
    colgen::column_manager<
        Model, pattern, colgen::property_list<colgen::reduced_cost>,
        colgen::property_list<colgen::reduced_cost, colgen::age>>
        columns;
    // --8<-- [end:evicting_manager]
    auto demand_constrs = demand_rows(model);
    auto add_pattern = pattern_adder(demand_constrs);
    single_length_columns(model, columns, add_pattern);
    // --8<-- [start:evicting_round]
    for(;;) {
        model.solve();
        // read the master before activate_columns and evict_columns modify it
        auto duals = model.get_dual_solution();
        std::vector<double> value(m);
        for(std::size_t o : orders) value[o] = duals[demand_constrs(o)];
        if(auto p = price(value)) columns.emplace_pool_column(std::move(*p));
        columns.update_columns([&](const auto & entry) {  // pool and master
            double rc = 1.0;
            for(std::size_t o : orders) rc -= value[o] * entry.seed[o];
            return colgen::priced{rc};
        });
        if(columns.activate_columns(
               model, colgen::negative<colgen::reduced_cost>{1e-9},
               add_pattern) == 0)
            break;  // converged, and the master is still as solved last
        // in the master for more than one round, and now unattractive
        columns.evict_columns(
            model, colgen::conjunction<colgen::above<colgen::age>,
                                       colgen::positive<colgen::reduced_cost>>{
                       1, 1e-6});
    }
    // --8<-- [end:evicting_round]
    return finish(model, columns);
}

template <typename Model>
run cutting_stock::several_per_round(Model & model) {
    const std::size_t m = length.size();
    auto orders = std::views::iota(std::size_t{0}, m);
    using pattern = std::vector<int>;
    colgen::column_manager<Model, pattern,
                           colgen::property_list<colgen::reduced_cost>>
        columns;
    auto demand_constrs = demand_rows(model);
    auto add_pattern = pattern_adder(demand_constrs);
    single_length_columns(model, columns, add_pattern);
    run counts;
    for(;;) {
        model.solve();
        auto duals = model.get_dual_solution();
        std::vector<double> value(m);
        for(std::size_t o : orders) value[o] = duals[demand_constrs(o)];
        // --8<-- [start:several_emplace]
        auto found = columns.emplace_pool_columns(price_several(value));
        // --8<-- [end:several_emplace]
        counts.inserted += found.num_inserted;
        counts.already_in_pool += found.num_already_in_pool;
        counts.already_in_master += found.num_already_in_master;
        columns.update_pool_columns([&](const auto & entry) {
            double rc = 1.0;
            for(std::size_t o : orders) rc -= value[o] * entry.seed[o];
            return colgen::priced{rc};
        });
        // --8<-- [start:several_activate]
        auto by_reduced_cost = [](const auto & a, const auto & b) {
            return a.state.template get<colgen::reduced_cost>() <
                   b.state.template get<colgen::reduced_cost>();
        };
        if(columns.activate_columns(
               model, colgen::negative<colgen::reduced_cost>{1e-9}, add_pattern,
               colgen::at_most_k_best{2, by_reduced_cost}) == 0)
            break;
        // --8<-- [end:several_activate]
    }
    run r = finish(model, columns);
    r.inserted = counts.inserted;
    r.already_in_pool = counts.already_in_pool;
    r.already_in_master = counts.already_in_master;
    return r;
}

template <typename Model>
run cutting_stock::pool_first(Model & model) {
    const std::size_t m = length.size();
    auto orders = std::views::iota(std::size_t{0}, m);
    using pattern = std::vector<int>;
    colgen::column_manager<
        Model, pattern, colgen::property_list<colgen::reduced_cost>,
        colgen::property_list<colgen::reduced_cost, colgen::age>>
        columns;
    auto demand_constrs = demand_rows(model);
    std::size_t activations = 0;
    auto add_pattern = [&, add = pattern_adder(demand_constrs)](
                           Model & master, const auto & entry) {
        ++activations;
        return add(master, entry);
    };
    single_length_columns(model, columns, add_pattern);
    activations = 0;
    // --8<-- [start:pool_first_round]
    auto improving = colgen::negative<colgen::reduced_cost>{1e-9};
    for(;;) {
        model.solve();
        auto duals = model.get_dual_solution();
        std::vector<double> value(m);
        for(std::size_t o : orders) value[o] = duals[demand_constrs(o)];
        auto reprice = [&](const auto & entry) {
            double rc = 1.0;
            for(std::size_t o : orders) rc -= value[o] * entry.seed[o];
            return colgen::priced{rc};
        };
        columns.update_columns(reprice);  // every known column
        if(std::ranges::none_of(columns.pool_columns(), improving)) {
            if(auto p = price(value)) {  // only when no pool column improves
                columns.emplace_pool_column(std::move(*p));
                columns.update_pool_columns(reprice);
            }
        }
        if(columns.activate_columns(model, improving, add_pattern) == 0) break;
        columns.evict_columns(
            model, colgen::conjunction<colgen::above<colgen::age>,
                                       colgen::positive<colgen::reduced_cost>>{
                       1, 1e-6});
    }
    // --8<-- [end:pool_first_round]
    run r = finish(model, columns);
    r.activations = activations;
    return r;
}

template <typename Model, typename Milp>
integer_plan cutting_stock::price_and_branch(Model & model, Milp & milp) {
    const std::size_t m = length.size();
    auto orders = std::views::iota(std::size_t{0}, m);
    using pattern = std::vector<int>;
    colgen::column_manager<Model, pattern,
                           colgen::property_list<colgen::reduced_cost>>
        columns;
    auto demand_constrs = demand_rows(model);
    auto add_pattern = pattern_adder(demand_constrs);
    single_length_columns(model, columns, add_pattern);
    do {
        model.solve();
        auto duals = model.get_dual_solution();
        std::vector<double> value(m);
        for(std::size_t o : orders) value[o] = duals[demand_constrs(o)];
        if(auto p = price(value)) columns.emplace_pool_column(std::move(*p));
        columns.update_pool_columns([&](const auto & entry) {
            double rc = 1.0;
            for(std::size_t o : orders) rc -= value[o] * entry.seed[o];
            return colgen::priced{rc};
        });
    } while(columns.activate_columns(
        model, colgen::negative<colgen::reduced_cost>{1e-9}, add_pattern));
    // --8<-- [start:price_and_branch]
    milp.set_minimization();
    auto milp_demand = milp.add_constraints(orders, [&](std::size_t o) {
        return empty_linear_expression<model_variable_t<Milp>, double> >=
               demand[o];
    });
    std::vector<std::pair<pattern, model_variable_t<Milp>>> plan;
    auto add_integer_pattern = [&](const auto & entry) {
        auto rolls = milp.add_column(std::views::zip(milp_demand, entry.seed),
                                     {.obj_coef = 1, .lower_bound = 0});
        milp.set_integer(rolls);
        plan.emplace_back(entry.seed, rolls);
    };
    std::ranges::for_each(columns.master_columns(), add_integer_pattern);
    std::ranges::for_each(columns.pool_columns(), add_integer_pattern);
    milp.solve();
    // --8<-- [end:price_and_branch]
    integer_plan result;
    result.rolls = milp.get_solution_value();
    result.pieces.assign(m, 0.0);
    auto solution = milp.get_solution();
    for(const auto & [p, rolls] : plan) {
        const double x = solution[rolls];
        if(std::abs(x - std::round(x)) > 1e-6) result.integral = false;
        for(std::size_t o : orders) result.pieces[o] += p[o] * x;
    }
    return result;
}

template <typename Model>
run cutting_stock::nonbasic_eviction(Model & model) {
    const std::size_t m = length.size();
    auto orders = std::views::iota(std::size_t{0}, m);
    using pattern = std::vector<int>;
    // --8<-- [start:nonbasic_manager]
    using column_status = model_variable_basis_status_t<Model>;
    colgen::column_manager<
        Model, pattern, colgen::property_list<colgen::reduced_cost>,
        colgen::property_list<colgen::reduced_cost,
                              colgen::variable_status<column_status>,
                              colgen::age>>
        columns;
    // --8<-- [end:nonbasic_manager]
    auto demand_constrs = demand_rows(model);
    auto add_pattern = pattern_adder(demand_constrs);
    single_length_columns(model, columns, add_pattern);
    // --8<-- [start:nonbasic_round]
    auto nonbasic = [](const auto & entry) {
        return !is<basis_status::basic>(
            entry.state.template get<colgen::variable_status<column_status>>());
    };
    for(;;) {
        model.solve();
        auto duals = model.get_dual_solution();
        auto reduced_costs = model.get_reduced_costs();
        auto solution = model.get_solution();
        auto basis = model.get_basis();
        std::vector<double> value(m);
        for(std::size_t o : orders) value[o] = duals[demand_constrs(o)];
        if(auto p = price(value)) columns.emplace_pool_column(std::move(*p));
        columns.update_columns(
            [&](const auto & entry) {  // a pool column, from the duals
                double rc = 1.0;
                for(std::size_t o : orders) rc -= value[o] * entry.seed[o];
                return colgen::priced{rc};
            },
            [&](const auto & entry) {  // a master column, from the solver
                const auto & var = entry.state.var;
                return colgen::master_refreshed<column_status>{
                    reduced_costs[var], solution[var], basis.get_status(var)};
            });
        if(columns.activate_columns(
               model, colgen::negative<colgen::reduced_cost>{1e-9},
               add_pattern) == 0)
            break;
        columns.evict_columns(
            model,
            colgen::conjunction{colgen::above<colgen::age>{1}, nonbasic,
                                colgen::positive<colgen::reduced_cost>{1e-6}});
    }
    // --8<-- [end:nonbasic_round]
    return finish(model, columns);
}

template <typename Model>
run cutting_stock::relative_tolerance(Model & model) {
    const std::size_t m = length.size();
    auto orders = std::views::iota(std::size_t{0}, m);
    using pattern = std::vector<int>;
    colgen::column_manager<Model, pattern,
                           colgen::property_list<colgen::reduced_cost>>
        columns;
    auto demand_constrs = demand_rows(model);
    auto add_pattern = pattern_adder(demand_constrs);
    single_length_columns(model, columns, add_pattern);
    for(;;) {
        model.solve();
        auto duals = model.get_dual_solution();
        std::vector<double> value(m);
        for(std::size_t o : orders) value[o] = duals[demand_constrs(o)];
        if(auto p = price(value)) columns.emplace_pool_column(std::move(*p));
        columns.update_pool_columns([&](const auto & entry) {
            double rc = 1.0;
            for(std::size_t o : orders) rc -= value[o] * entry.seed[o];
            return colgen::priced{rc};
        });
        // --8<-- [start:relative_tolerance]
        // improving by more than 1e-7 of what its pieces are worth, plus 1e-9
        auto improving = [&](const auto & entry) {
            double worth = 0.0;
            for(std::size_t o : orders) worth += value[o] * entry.seed[o];
            return entry.state.template get<colgen::reduced_cost>() <
                   -1e-7 * worth - 1e-9;
        };
        if(columns.activate_columns(model, improving, add_pattern) == 0) break;
        // --8<-- [end:relative_tolerance]
    }
    return finish(model, columns);
}

template <typename Model>
run cutting_stock::with_costs(Model & model) {
    const std::size_t m = length.size();
    auto orders = std::views::iota(std::size_t{0}, m);
    using pattern = std::vector<int>;
    auto demand_constrs = demand_rows(model);
    // --8<-- [start:data_manager]
    colgen::column_manager<Model, pattern,
                           colgen::property_list<colgen::reduced_cost>,
                           colgen::property_list<>, double>  // a pattern's cost
        columns;
    auto add_pattern = [&](Model & master, const auto & entry) {
        return master.add_column(std::views::zip(demand_constrs, entry.seed),
                                 {.obj_coef = entry.data, .lower_bound = 0});
    };
    for(std::size_t o : orders) {
        pattern p(m, 0);
        p[o] = roll_length / length[o];
        columns.emplace_master_column(model, std::move(p), add_pattern, 1.0);
    }
    // --8<-- [end:data_manager]
    do {
        model.solve();
        auto duals = model.get_dual_solution();
        std::vector<double> value(m);
        for(std::size_t o : orders) value[o] = duals[demand_constrs(o)];
        // --8<-- [start:data_round]
        if(auto p = price(value))
            columns.emplace_pool_column(std::move(*p), 1.0);  // one roll
        columns.update_pool_columns([&](const auto & entry) {
            double rc = entry.data;
            for(std::size_t o : orders) rc -= value[o] * entry.seed[o];
            return colgen::priced{rc};
        });
        // --8<-- [end:data_round]
    } while(columns.activate_columns(
        model, colgen::negative<colgen::reduced_cost>{1e-9}, add_pattern));
    return finish(model, columns);
}

}  // namespace column_generation_page

namespace {

using column_generation_page::cutting_stock;

constexpr double page_optimum = 452.25;
constexpr double larger_optimum = 9889.0 / 24.0;

// the duals of the master of single-length patterns
std::vector<double> single_length_duals(const cutting_stock & instance) {
    std::vector<double> value;
    for(int l : instance.length)
        value.push_back(1.0 / (instance.roll_length / l));
    return value;
}

double worth(const std::vector<int> & pattern,
             const std::vector<double> & value) {
    double sum = 0.0;
    for(std::size_t o = 0; o < pattern.size(); ++o)
        sum += value[o] * pattern[o];
    return sum;
}

void check_price_several(cutting_stock instance) {
    const auto value = single_length_duals(instance);
    const auto patterns = instance.price_several(value);
    ASSERT_GE(patterns.size(), 2u);
    double best = 0.0;
    for(std::size_t i = 0; i < patterns.size(); ++i) {
        int used = 0;
        for(std::size_t o = 0; o < instance.length.size(); ++o)
            used += patterns[i][o] * instance.length[o];
        EXPECT_LE(used, instance.roll_length);
        EXPECT_GT(worth(patterns[i], value), 1.0 + 1e-9);  // improving
        for(std::size_t j = 0; j < i; ++j) EXPECT_NE(patterns[i], patterns[j]);
        best = std::max(best, worth(patterns[i], value));
    }
    const auto single = instance.price(value);
    ASSERT_TRUE(single.has_value());
    EXPECT_NEAR(best, worth(*single, value), TEST_EPSILON);
}

}  // namespace

// The page says a column activated in a round reads bare nonbasic until its
// first master_refreshed: variable_status starts from the first alternative.
TEST(column_generation_page, a_fresh_master_column_reads_nonbasic) {
    EXPECT_TRUE(std::holds_alternative<basis_status::nonbasic>(
        model_variable_basis_status_t<highs_lp>{}));
    EXPECT_TRUE(std::holds_alternative<basis_status::nonbasic>(
        model_variable_basis_status_t<clp_lp>{}));
}

TEST(column_generation_page,
     price_several_patterns_are_distinct_improving_and_exact) {
    check_price_several(column_generation_page::page_instance());
    check_price_several(column_generation_page::larger_instance());
}

struct column_generation_page_highs_lp : model_test<highs_api, highs_lp> {
    static void SetUpTestSuite() { construct_api("HIGHS"); }
};
struct column_generation_page_clp_lp : model_test<clp_api, clp_lp> {
    static void SetUpTestSuite() { construct_api("CLP"); }
};
struct column_generation_page_highs_milp : model_test<highs_api, highs_milp> {
    static void SetUpTestSuite() { construct_api("HIGHS"); }
};

template <typename Fixture>
void check_classic(Fixture & fixture) {
    fixture.SkipOnLicenseError([&]() {
        auto model = fixture.new_model();
        auto instance = column_generation_page::page_instance();
        ASSERT_NEAR(instance.classic(model), page_optimum, TEST_EPSILON);
    });
}

template <typename Fixture>
void check_first_example(Fixture & fixture) {
    fixture.SkipOnLicenseError([&]() {
        auto model = fixture.new_model();
        auto instance = column_generation_page::page_instance();
        const auto r = instance.first_example(model);
        ASSERT_NEAR(r.value, page_optimum, TEST_EPSILON);
        EXPECT_EQ(instance.pricer_calls, 3u);  // one per round
        EXPECT_EQ(r.master_columns, 6u);
        EXPECT_EQ(r.pool_columns, 0u);
    });
}

template <typename Fixture>
void check_evicting(Fixture & fixture) {
    fixture.SkipOnLicenseError([&]() {
        auto model = fixture.new_model();
        auto instance = column_generation_page::page_instance();
        const auto r = instance.evicting(model);
        ASSERT_NEAR(r.value, page_optimum, TEST_EPSILON);
        // one column fewer in the master than in the first example
        EXPECT_EQ(r.master_columns, 5u);
        EXPECT_EQ(r.pool_columns, 1u);
    });
}

template <typename Fixture>
void check_several_per_round(Fixture & fixture) {
    fixture.SkipOnLicenseError([&]() {
        auto model = fixture.new_model();
        auto instance = column_generation_page::page_instance();
        const auto r = instance.several_per_round(model);
        ASSERT_NEAR(r.value, page_optimum, TEST_EPSILON);
        EXPECT_EQ(instance.pricer_calls, 2u);  // two rounds instead of three
        EXPECT_EQ(r.inserted, 3u);
        EXPECT_EQ(r.already_in_master, 0u);
        EXPECT_EQ(r.master_columns, 6u);  // two activated
        EXPECT_EQ(r.pool_columns, 1u);

        auto larger_model = fixture.new_model();
        auto larger = column_generation_page::larger_instance();
        const auto lr = larger.several_per_round(larger_model);
        ASSERT_NEAR(lr.value, larger_optimum, TEST_EPSILON);
        // improving patterns left in the pool by at_most_k_best, priced again
        EXPECT_GT(lr.already_in_pool, 0u);
        EXPECT_EQ(lr.already_in_master, 0u);
    });
}

template <typename Fixture>
void check_pool_first(Fixture & fixture) {
    fixture.SkipOnLicenseError([&]() {
        // Every column the pricer returns enters the master in its round, so
        // activations beyond their number are evicted columns that came back
        // without the pricer. The initial columns are not activations.
        auto model = fixture.new_model();
        auto instance = column_generation_page::page_instance();
        const auto r = instance.pool_first(model);
        ASSERT_NEAR(r.value, page_optimum, TEST_EPSILON);
        EXPECT_EQ(r.activations + instance.length.size(),
                  r.master_columns + r.pool_columns);  // none came back

        auto larger_model = fixture.new_model();
        auto larger = column_generation_page::larger_instance();
        const auto lr = larger.pool_first(larger_model);
        ASSERT_NEAR(lr.value, larger_optimum, TEST_EPSILON);
        EXPECT_GT(lr.activations + larger.length.size(),
                  lr.master_columns + lr.pool_columns);
    });
}

template <typename Fixture>
void check_nonbasic_eviction(Fixture & fixture) {
    fixture.SkipOnLicenseError([&]() {
        auto model = fixture.new_model();
        auto instance = column_generation_page::page_instance();
        const auto r = instance.nonbasic_eviction(model);
        ASSERT_NEAR(r.value, page_optimum, TEST_EPSILON);
        EXPECT_EQ(r.master_columns, 5u);
        EXPECT_EQ(r.pool_columns, 1u);
    });
}

template <typename Fixture>
void check_relative_tolerance(Fixture & fixture) {
    fixture.SkipOnLicenseError([&]() {
        auto model = fixture.new_model();
        auto instance = column_generation_page::page_instance();
        ASSERT_NEAR(instance.relative_tolerance(model).value, page_optimum,
                    TEST_EPSILON);
    });
}

template <typename Fixture>
void check_with_costs(Fixture & fixture) {
    fixture.SkipOnLicenseError([&]() {
        auto model = fixture.new_model();
        auto instance = column_generation_page::page_instance();
        ASSERT_NEAR(instance.with_costs(model).value, page_optimum,
                    TEST_EPSILON);
    });
}

TEST_F(column_generation_page_highs_lp, classic_loop_reaches_the_optimum) {
    check_classic(*this);
}
TEST_F(column_generation_page_clp_lp, classic_loop_reaches_the_optimum) {
    check_classic(*this);
}
TEST_F(column_generation_page_highs_lp, first_example_reaches_the_optimum) {
    check_first_example(*this);
}
TEST_F(column_generation_page_clp_lp, first_example_reaches_the_optimum) {
    check_first_example(*this);
}
TEST_F(column_generation_page_highs_lp, eviction_keeps_a_smaller_master) {
    check_evicting(*this);
}
TEST_F(column_generation_page_clp_lp, eviction_keeps_a_smaller_master) {
    check_evicting(*this);
}
TEST_F(column_generation_page_highs_lp, several_per_round_saves_a_round) {
    check_several_per_round(*this);
}
TEST_F(column_generation_page_clp_lp, several_per_round_saves_a_round) {
    check_several_per_round(*this);
}
TEST_F(column_generation_page_highs_lp,
       pool_first_brings_evicted_columns_back) {
    check_pool_first(*this);
}
TEST_F(column_generation_page_clp_lp, pool_first_brings_evicted_columns_back) {
    check_pool_first(*this);
}
TEST_F(column_generation_page_highs_lp,
       nonbasic_eviction_keeps_a_smaller_master) {
    check_nonbasic_eviction(*this);
}
TEST_F(column_generation_page_clp_lp,
       nonbasic_eviction_keeps_a_smaller_master) {
    check_nonbasic_eviction(*this);
}
TEST_F(column_generation_page_highs_lp, relative_tolerance_reaches_optimum) {
    check_relative_tolerance(*this);
}
TEST_F(column_generation_page_clp_lp, relative_tolerance_reaches_optimum) {
    check_relative_tolerance(*this);
}
TEST_F(column_generation_page_highs_lp, column_costs_reach_the_optimum) {
    check_with_costs(*this);
}
TEST_F(column_generation_page_clp_lp, column_costs_reach_the_optimum) {
    check_with_costs(*this);
}

TEST_F(column_generation_page_highs_milp, price_and_branch_covers_demand) {
    SkipOnLicenseError([&]() {
        highs_lp master(*api);
        auto milp = new_model();
        auto instance = column_generation_page::page_instance();
        const auto plan = instance.price_and_branch(master, milp);
        EXPECT_GE(plan.rolls, std::ceil(page_optimum) - TEST_EPSILON);
        EXPECT_TRUE(plan.integral);
        for(std::size_t o = 0; o < instance.demand.size(); ++o)
            EXPECT_GE(plan.pieces[o], instance.demand[o] - TEST_EPSILON);
    });
}
