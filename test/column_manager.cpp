#include <gtest/gtest.h>

#include <algorithm>
#include <concepts>
#include <cstddef>
#include <functional>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include "mippp/model_entities.hpp"
#include "mippp/utility/column_manager.hpp"

using namespace mippp;
using namespace mippp::colgen;

namespace {

// A minimal stand-in for a solver model : the column manager only needs
// model_variable_t / model_scalar_t, which are deduced from add_variable, and
// lets the add/remove callbacks materialize the actual model modifications,
// so no real solver is required to unit test the pool <-> master bookkeeping
// and the selection.
struct fake_model {
    int next_variable = 0;
    model_variable<int, double> add_variable() {
        return model_variable<int, double>(next_variable++);
    }
};
// models remove_variable(s), for the default remove callback of
// evict_columns
struct removable_fake_model : fake_model {
    std::vector<int> removed;
    void remove_variable(model_variable<int, double> v) {
        removed.push_back(v.id());
    }
    template <typename R>
    void remove_variables(R && variables) {
        for(auto && v : variables) remove_variable(v);
    }
};

// shaped like the column status of a basis_snapshot, bare nonbasic first: a
// master column holds it until its first master_refreshed
using fake_basis_status =
    std::variant<basis_status::nonbasic, basis_status::basic,
                 basis_status::nonbasic_at_lower_bound,
                 basis_status::nonbasic_at_upper_bound,
                 basis_status::nonbasic_fixed, basis_status::superbasic,
                 basis_status::nonbasic_free>;

// times_activated is listed in both states so that it survives the
// pool <-> master transitions (see transfer_common_properties)
using pool_properties = property_list<reduced_cost, age, times_activated>;
using master_properties =
    property_list<reduced_cost, age, variable_value,
                  variable_status<fake_basis_status>, times_activated>;
using manager =
    column_manager<fake_model, int, pool_properties, master_properties>;

// reads a single column's property from the views (the states are only
// exposed through pool_columns()/master_columns())
template <typename P, typename Manager>
std::optional<typename P::value_type> pool_get(const Manager & columns,
                                               int seed) {
    for(auto && e : columns.pool_columns())
        if(e.seed == seed) return e.state.template get<P>();
    return std::nullopt;
}
template <typename P, typename Manager>
std::optional<typename P::value_type> master_get(const Manager & columns,
                                                 int seed) {
    for(auto && e : columns.master_columns())
        if(e.seed == seed) return e.state.template get<P>();
    return std::nullopt;
}
template <typename Manager>
std::vector<int> pool_seeds(const Manager & columns) {
    std::vector<int> seeds;
    for(auto && e : columns.pool_columns()) seeds.push_back(e.seed);
    return seeds;
}

// the recurring add callback of the tests that ignore the entries
auto make_variable = [](auto & m, const auto &) { return m.add_variable(); };

// a pricing callback reading the reduced costs from the given table
auto price_from(const std::unordered_map<int, double> & rc) {
    return [&rc](const auto & e) { return priced{rc.at(e.seed)}; };
}

auto by_reduced_cost = [](const auto & a, const auto & b) {
    return a.state.template get<reduced_cost>() <
           b.state.template get<reduced_cost>();
};

}  // namespace

///////////////////////////////////////////////////////////////////////////////
///////////////// States embed exactly the declared properties ////////////////
///////////////////////////////////////////////////////////////////////////////

GTEST_TEST(column_manager, states_embed_declared_properties) {
    using pool_state = manager::in_pool_state;
    using master_state = manager::in_master_state;
    static_assert(pool_state::num_properties() == 3);
    static_assert(master_state::num_properties() == 5);
    static_assert(pool_state::has_property<reduced_cost>());
    static_assert(master_state::has_property<times_activated>());
    // solution-dependent properties only make sense for master columns
    static_assert(!pool_state::has_property<variable_value>());
    static_assert(master_state::has_property<variable_value>());
    static_assert(
        master_state::has_property<variable_status<fake_basis_status>>());
    // a Data leaves the states alone: it is stored beside them
    using data_manager = column_manager<fake_model, int, pool_properties,
                                        master_properties, double>;
    static_assert(std::same_as<data_manager::in_pool_state, pool_state>);
    static_assert(std::same_as<data_manager::in_master_state, master_state>);
    static_assert(
        std::same_as<pool_state, in_pool<reduced_cost, age, times_activated>>);
}

///////////////////////////////////////////////////////////////////////////////
////////// Emplacement partitions the columns between pool and master /////////
///////////////////////////////////////////////////////////////////////////////

GTEST_TEST(column_manager, emplacement_and_bookkeeping) {
    manager columns;
    fake_model model;
    ASSERT_TRUE(columns.emplace_pool_column(1));
    ASSERT_FALSE(columns.emplace_pool_column(1));  // already known
    ASSERT_TRUE(columns.emplace_master_column(2, model.add_variable()));
    ASSERT_FALSE(columns.emplace_master_column(2, model.add_variable()));

    EXPECT_EQ(columns.num_columns(), 2u);
    EXPECT_EQ(columns.num_pool_columns(), 1u);
    EXPECT_EQ(columns.num_master_columns(), 1u);
    EXPECT_TRUE(columns.contains(1));
    EXPECT_TRUE(columns.contains(2));
    EXPECT_FALSE(columns.contains(3));

    // master_variable distinguishes the two states
    EXPECT_FALSE(columns.master_variable(1).has_value());
    EXPECT_EQ(columns.master_variable(2)->id(), 0);
    EXPECT_FALSE(columns.master_variable(3).has_value());
    // registering an initial master column counts as an activation
    EXPECT_EQ(master_get<times_activated>(columns, 2), 1u);
}

GTEST_TEST(column_manager, emplace_master_column_through_add_callback) {
    manager columns;
    fake_model model;
    int num_calls = 0;
    auto add = [&](fake_model & m, const auto & e) {
        ++num_calls;
        EXPECT_EQ(e.seed, 5);
        return m.add_variable();
    };
    ASSERT_TRUE(columns.emplace_master_column(model, 5, add));
    EXPECT_EQ(num_calls, 1);
    EXPECT_EQ(columns.master_variable(5)->id(), 0);
    EXPECT_EQ(master_get<times_activated>(columns, 5), 1u);
    EXPECT_EQ(columns.num_master_columns(), 1u);

    // a known seed creates no variable
    EXPECT_FALSE(columns.emplace_master_column(model, 5, add));
    columns.emplace_pool_column(6);
    EXPECT_FALSE(columns.emplace_master_column(model, 6, add));
    EXPECT_EQ(num_calls, 1);
    EXPECT_EQ(model.next_variable, 1);

    // a throwing add callback registers nothing
    auto throwing_add = [](fake_model &,
                           const auto &) -> model_variable<int, double> {
        throw std::runtime_error("add");
    };
    EXPECT_THROW(columns.emplace_master_column(model, 7, throwing_add),
                 std::runtime_error);
    EXPECT_FALSE(columns.contains(7));
    EXPECT_EQ(columns.num_columns(), 2u);
    EXPECT_EQ(columns.num_master_columns(), 1u);
}

GTEST_TEST(column_manager, emplace_columns_reports_duplicates) {
    manager columns;
    fake_model model;
    columns.emplace_pool_column(1);
    columns.emplace_master_column(2, model.add_variable());

    // num_already_in_master flags a pricing oracle regenerating master
    // columns : stale duals or a cycling process
    auto result = columns.emplace_pool_columns(std::vector<int>{1, 2, 3, 4});
    EXPECT_EQ(result.num_inserted, 2u);
    EXPECT_EQ(result.num_already_in_pool, 1u);
    EXPECT_EQ(result.num_already_in_master, 1u);
    EXPECT_EQ(columns.num_pool_columns(), 3u);
    EXPECT_EQ(columns.num_master_columns(), 1u);
}

///////////////////////////////////////////////////////////////////////////////
///////// Updates broadcast events to the targeted columns' properties ////////
///////////////////////////////////////////////////////////////////////////////

GTEST_TEST(column_manager, pool_updates_reach_properties) {
    // 'age' must tick once per round that carries a reduced cost
    manager columns;
    columns.emplace_pool_column(1);
    columns.emplace_pool_column(2);
    columns.emplace_pool_column(3);
    ASSERT_EQ(columns.num_pool_columns(), 3u);

    const std::unordered_map<int, double> rc = {{1, -1.5}, {2, 2.0}, {3, -3.0}};
    columns.update_pool_columns(price_from(rc));

    for(int seed : {1, 2, 3}) {
        EXPECT_EQ(pool_get<reduced_cost>(columns, seed), rc.at(seed));
        EXPECT_EQ(pool_get<age>(columns, seed), 1u);  // one round elapsed
    }

    columns.update_pool_columns(price_from(rc));
    for(int seed : {1, 2, 3}) EXPECT_EQ(pool_get<age>(columns, seed), 2u);
}

GTEST_TEST(column_manager, master_refresh_updates_value_and_basis_status) {
    // a multi-property event (reduced cost + primal value + basis status)
    // updates every carried property in a single broadcast
    manager columns;
    fake_model model;
    columns.emplace_master_column(10, model.add_variable());
    columns.emplace_pool_column(20);

    columns.update_master_columns([&](const auto & e) {
        return master_refreshed<fake_basis_status>{
            -4.0, 2.5 * e.state.var.id(),
            basis_status::nonbasic_at_upper_bound{}};
    });

    EXPECT_EQ(master_get<reduced_cost>(columns, 10), -4.0);
    EXPECT_EQ(master_get<variable_value>(columns, 10), 0.0);  // var == 0
    const auto status =
        master_get<variable_status<fake_basis_status>>(columns, 10);
    ASSERT_TRUE(status.has_value());
    EXPECT_TRUE(
        std::holds_alternative<basis_status::nonbasic_at_upper_bound>(*status));
    EXPECT_EQ(master_get<age>(columns, 10), 1u);

    // the pool column was not touched by update_master_columns
    EXPECT_EQ(pool_get<age>(columns, 20), 0u);
}

GTEST_TEST(column_manager, update_columns_reaches_both_states) {
    manager columns;
    fake_model model;
    columns.emplace_pool_column(1);
    columns.emplace_master_column(2, model.add_variable());

    // the single-callback overload broadcasts one event to every column
    columns.update_columns([](const auto & e) { return priced{e.seed * 1.0}; });
    EXPECT_EQ(pool_get<reduced_cost>(columns, 1), 1.0);
    EXPECT_EQ(master_get<reduced_cost>(columns, 2), 2.0);

    // the two-callback overload builds a distinct event per state
    columns.update_columns([](const auto &) { return priced{-1.0}; },
                           [](const auto &) { return priced{-2.0}; });
    EXPECT_EQ(pool_get<reduced_cost>(columns, 1), -1.0);
    EXPECT_EQ(master_get<reduced_cost>(columns, 2), -2.0);
}

///////////////////////////////////////////////////////////////////////////////
///////// Activation and eviction move exactly the selected columns ///////////
///////////////////////////////////////////////////////////////////////////////

GTEST_TEST(column_manager, activation_selects_matching_columns) {
    // the add callback observes each activated entry
    manager columns;
    fake_model model;
    for(int seed : {1, 2, 3, 4}) columns.emplace_pool_column(seed);

    const std::unordered_map<int, double> rc = {
        {1, -1.0}, {2, 0.5}, {3, -2.0}, {4, 0.0}};
    columns.update_pool_columns(price_from(rc));

    std::vector<int> activated_seeds;
    auto add_column = [&](fake_model & m, const auto & e) {
        EXPECT_LT(e.state.template get<reduced_cost>(), 0.0)
            << "activated a non-improving column (seed " << e.seed << ")";
        activated_seeds.push_back(e.seed);
        return m.add_variable();
    };

    EXPECT_EQ(
        columns.activate_columns(model, negative<reduced_cost>{}, add_column),
        2u);
    EXPECT_EQ(columns.num_master_columns(), 2u);
    // in the order the seeds were emplaced
    EXPECT_EQ(activated_seeds, (std::vector<int>{1, 3}));

    EXPECT_TRUE(columns.master_variable(1).has_value());
    EXPECT_TRUE(columns.master_variable(3).has_value());
    EXPECT_FALSE(columns.master_variable(2).has_value());
    EXPECT_FALSE(columns.master_variable(4).has_value());
    for(auto && [seed, state] : columns.master_columns())
        EXPECT_LT(state.get<reduced_cost>(), 0.0) << seed;
}

GTEST_TEST(column_manager, eviction_reads_state_in_remove_callback) {
    // the remove callback reads the state of each evicted column and the
    // evicted columns fall back to the pool with their reduced cost preserved
    manager columns;
    fake_model model;
    for(int seed : {1, 2, 3})
        columns.emplace_master_column(seed, model.add_variable());
    ASSERT_EQ(columns.num_master_columns(), 3u);

    const std::unordered_map<int, double> rc = {{1, 1.0}, {2, -0.5}, {3, 3.0}};
    columns.update_master_columns(price_from(rc));

    std::vector<int> evicted_seeds;
    auto remove_columns = [&](fake_model &, auto && evicted_entries) {
        for(auto && [seed, state] : evicted_entries) {
            EXPECT_GT(state.template get<reduced_cost>(), 0.0)
                << "evicted an attractive column (seed " << seed << ")";
            evicted_seeds.push_back(seed);
        }
    };

    EXPECT_EQ(
        columns.evict_columns(model, positive<reduced_cost>{}, remove_columns),
        2u);
    EXPECT_EQ(evicted_seeds, (std::vector<int>{1, 3}));

    EXPECT_EQ(columns.num_master_columns(), 1u);
    EXPECT_EQ(columns.num_pool_columns(), 2u);
    EXPECT_TRUE(columns.master_variable(2).has_value());  // attractive, kept
    // the reduced cost survives the master -> pool transition
    EXPECT_EQ(pool_get<reduced_cost>(columns, 1), 1.0);
    EXPECT_EQ(pool_get<reduced_cost>(columns, 3), 3.0);
    EXPECT_EQ(pool_get<age>(columns, 1), 0u);  // age reset on deactivation
}

GTEST_TEST(column_manager, default_remove_callback_removes_variables) {
    column_manager<removable_fake_model, int, property_list<reduced_cost>,
                   property_list<reduced_cost>>
        columns;
    removable_fake_model model;
    for(int seed : {1, 2, 3})
        columns.emplace_master_column(seed, model.add_variable());
    columns.update_master_columns(
        [](const auto & e) { return priced{e.seed == 2 ? 1.0 : -1.0}; });

    EXPECT_EQ(columns.evict_columns(model, positive<reduced_cost>{}), 1u);
    EXPECT_EQ(model.removed, (std::vector<int>{1}));  // seed 2's variable
}

GTEST_TEST(column_manager, eviction_selecting_nothing_leaves_model_alone) {
    // a round that converged must not modify the master after its last
    // solve : no add, no remove
    manager columns;
    fake_model model;
    columns.emplace_master_column(1, model.add_variable());
    columns.emplace_pool_column(2);
    columns.update_columns([](const auto &) { return priced{-1.0}; });

    int num_removes = 0;
    auto remove_columns = [&](fake_model &, auto &&) { ++num_removes; };
    EXPECT_EQ(columns.activate_columns(model, positive<reduced_cost>{},
                                       make_variable),
              0u);
    EXPECT_EQ(
        columns.evict_columns(model, positive<reduced_cost>{}, remove_columns),
        0u);
    EXPECT_EQ(num_removes, 0);
    EXPECT_EQ(model.next_variable, 1);
}

GTEST_TEST(column_manager, activate_then_evict_in_one_round) {
    // the age gate protects the column activated in the same round
    manager columns;
    fake_model model;
    columns.emplace_master_column(1, model.add_variable());
    columns.emplace_pool_column(2);
    const std::unordered_map<int, double> rc = {{1, 2.0}, {2, -1.0}};
    columns.update_columns(price_from(rc));

    std::vector<int> evicted;
    auto remove_columns = [&](fake_model &, auto && entries) {
        for(auto && e : entries) evicted.push_back(e.seed);
    };
    EXPECT_EQ(columns.activate_columns(model, negative<reduced_cost>{},
                                       make_variable),
              1u);
    EXPECT_EQ(columns.evict_columns(
                  model, conjunction<above<age>, above<reduced_cost>>{0, -1e9},
                  remove_columns),
              1u);
    EXPECT_EQ(evicted, (std::vector<int>{1}));
    EXPECT_TRUE(columns.master_variable(2).has_value());
}

GTEST_TEST(column_manager, maximizing_master_uses_the_opposite_signs) {
    using window_manager =
        column_manager<fake_model, int, property_list<reduced_cost>,
                       property_list<reduced_cost, reduced_cost_window<2>>>;
    window_manager columns;
    fake_model model;
    for(int seed : {1, 2, 3}) columns.emplace_pool_column(seed);
    const std::unordered_map<int, double> rc = {{1, 1.0}, {2, -1.0}, {3, 2.0}};
    columns.update_pool_columns(price_from(rc));

    // improving columns of a maximizing master have positive reduced costs
    EXPECT_EQ(columns.activate_columns(model, positive<reduced_cost>{},
                                       make_variable),
              2u);
    EXPECT_TRUE(columns.master_variable(1).has_value());
    EXPECT_TRUE(columns.master_variable(3).has_value());

    // and its unattractive columns stay negative for the whole window
    auto no_remove = [](fake_model &, auto &&) {};
    columns.update_master_columns(
        [](const auto & e) { return priced{e.seed == 1 ? -1.0 : 0.5}; });
    EXPECT_EQ(columns.evict_columns(model, window_below<2>{0.0}, no_remove),
              0u);  // one negative reduced cost, and the window's initial 0
    columns.update_master_columns(
        [](const auto & e) { return priced{e.seed == 1 ? -2.0 : 0.5}; });
    EXPECT_EQ(columns.evict_columns(model, window_below<2>{0.0}, no_remove),
              1u);
    EXPECT_FALSE(columns.master_variable(1).has_value());
    EXPECT_TRUE(columns.master_variable(3).has_value());
}

///////////////////////////////////////////////////////////////////////////////
////////////////////// Selectors narrow the candidates ////////////////////////
///////////////////////////////////////////////////////////////////////////////

GTEST_TEST(column_manager, at_most_k_best_ranks_by_state_property) {
    // keeps only the k best candidates according to a comparator reading a
    // state property (here : the most negative reduced costs)
    manager columns;
    fake_model model;
    for(int seed : {1, 2, 3, 4}) columns.emplace_pool_column(seed);

    const std::unordered_map<int, double> rc = {
        {1, -1.0}, {2, -5.0}, {3, -0.2}, {4, -3.0}};
    columns.update_pool_columns(price_from(rc));

    std::vector<int> activated_seeds;
    auto add_column = [&](fake_model & m, const auto & e) {
        activated_seeds.push_back(e.seed);
        return m.add_variable();
    };

    EXPECT_EQ(
        columns.activate_columns(model, negative<reduced_cost>{}, add_column,
                                 at_most_k_best{2, by_reduced_cost}),
        2u);
    std::ranges::sort(activated_seeds);
    EXPECT_EQ(activated_seeds, (std::vector<int>{2, 4}));  // -5.0 and -3.0
}

GTEST_TEST(column_manager, at_most_k_takes_the_first_candidates) {
    manager columns;
    fake_model model;
    for(int seed : {4, 1, 3, 2}) columns.emplace_pool_column(seed);
    columns.update_pool_columns([](const auto &) { return priced{-1.0}; });

    std::vector<int> activated_seeds;
    auto add_column = [&](fake_model & m, const auto & e) {
        activated_seeds.push_back(e.seed);
        return m.add_variable();
    };
    EXPECT_EQ(columns.activate_columns(model, negative<reduced_cost>{},
                                       add_column, at_most_k{2}),
              2u);
    EXPECT_EQ(activated_seeds, (std::vector<int>{4, 1}));  // emplacement order
}

GTEST_TEST(column_manager, at_most_k_random_draws_k_candidates) {
    manager columns;
    fake_model model;
    for(int seed : {1, 2, 3, 4, 5}) columns.emplace_pool_column(seed);
    columns.update_pool_columns([](const auto &) { return priced{-1.0}; });

    EXPECT_EQ(
        columns.activate_columns(model, negative<reduced_cost>{}, make_variable,
                                 at_most_k_random{3, std::mt19937{7}}),
        3u);
    EXPECT_EQ(columns.num_master_columns(), 3u);
}

///////////////////////////////////////////////////////////////////////////////
///////// Properties survive or reset across pool <-> master transitions //////
///////////////////////////////////////////////////////////////////////////////

GTEST_TEST(column_manager, age_resets_and_times_activated_persists) {
    manager columns;
    fake_model model;
    columns.emplace_pool_column(7);

    auto price = [](const auto &) { return priced{-1.0}; };
    columns.update_pool_columns(price);
    columns.update_pool_columns(price);
    EXPECT_EQ(pool_get<age>(columns, 7), 2u);

    columns.activate_columns(model, negative<reduced_cost>{}, make_variable);
    ASSERT_TRUE(columns.master_variable(7).has_value());
    EXPECT_EQ(master_get<age>(columns, 7), 0u);  // reset on activation
    EXPECT_EQ(master_get<times_activated>(columns, 7), 1u);

    // evict it back to the pool, then reactivate : times_activated accumulates
    columns.update_master_columns([](const auto &) { return priced{5.0}; });
    auto no_remove = [](fake_model &, auto &&) {};
    columns.evict_columns(model, positive<reduced_cost>{}, no_remove);
    ASSERT_EQ(columns.num_pool_columns(), 1u);
    EXPECT_EQ(pool_get<age>(columns, 7), 0u);  // reset on deactivation too

    columns.update_pool_columns(price);
    columns.activate_columns(model, negative<reduced_cost>{}, make_variable);
    EXPECT_EQ(master_get<times_activated>(columns, 7), 2u);
}

GTEST_TEST(column_manager, backoff_keeps_returning_columns_longer) {
    manager columns;
    fake_model model;
    columns.emplace_pool_column(1);
    auto no_remove = [](fake_model &, auto &&) {};
    const age_above_with_backoff aged{1, 2};  // age > 1 + 2 * times_activated
    auto rounds_before_eviction = [&] {
        columns.update_pool_columns([](const auto &) { return priced{-1.0}; });
        columns.activate_columns(model, negative<reduced_cost>{},
                                 make_variable);
        std::size_t rounds = 0;
        do {
            columns.update_master_columns(
                [](const auto &) { return priced{1.0}; });
            ++rounds;
        } while(columns.evict_columns(model, aged, no_remove) == 0);
        return rounds;
    };
    EXPECT_EQ(rounds_before_eviction(), 4u);  // age 4 > 1 + 2 * 1
    EXPECT_EQ(rounds_before_eviction(), 6u);  // age 6 > 1 + 2 * 2
}

///////////////////////////////////////////////////////////////////////////////
//////////////////////// Columns carry their Data /////////////////////////////
///////////////////////////////////////////////////////////////////////////////

namespace {
struct contribution {
    double value = 0.0;
};
using data_manager =
    column_manager<fake_model, int, property_list<reduced_cost>,
                   property_list<reduced_cost>, contribution>;
}  // namespace

GTEST_TEST(column_manager, data_is_given_at_emplacement_and_kept) {
    data_manager columns;
    fake_model model;
    ASSERT_TRUE(columns.emplace_pool_column(1, 2.5));
    EXPECT_FALSE(columns.emplace_pool_column(1, 9.0));  // keeps the first
    auto result = columns.emplace_pool_columns(
        std::vector<std::pair<int, contribution>>{{1, {7.0}}, {2, {0.5}}});
    EXPECT_EQ(result.num_inserted, 1u);
    EXPECT_EQ(result.num_already_in_pool, 1u);
    ASSERT_TRUE(columns.emplace_master_column(3, model.add_variable(), 4.0));

    std::unordered_map<int, double> data;
    for(auto && [seed, d, state] : columns.pool_columns()) data[seed] = d.value;
    for(auto && [seed, d, state] : columns.master_columns())
        data[seed] = d.value;
    EXPECT_EQ(data,
              (std::unordered_map<int, double>{{1, 2.5}, {2, 0.5}, {3, 4.0}}));
}

GTEST_TEST(column_manager, every_callback_reads_the_data) {
    // the reduced cost of a column may depend on its data, as tree_rr's
    // depends on its contribution
    data_manager columns;
    fake_model model;
    columns.emplace_pool_column(1, 2.5);
    columns.emplace_pool_column(2, 0.5);
    columns.update_pool_columns(
        [](const auto & e) { return priced{-e.data.value}; });

    std::unordered_map<int, double> added;
    auto add_column = [&](fake_model & m, const auto & e) {
        added[e.seed] = e.data.value;
        return m.add_variable();
    };
    EXPECT_EQ(columns.activate_columns(
                  model, [](const auto & e) { return e.data.value > 1.0; },
                  add_column),
              1u);
    EXPECT_EQ(added, (std::unordered_map<int, double>{{1, 2.5}}));

    columns.update_master_columns(
        [](const auto & e) { return priced{e.data.value}; });
    std::vector<double> removed;
    auto remove_columns = [&](fake_model &, auto && entries) {
        for(auto && e : entries) removed.push_back(e.data.value);
    };
    EXPECT_EQ(
        columns.evict_columns(model, positive<reduced_cost>{}, remove_columns),
        1u);
    EXPECT_EQ(removed, (std::vector<double>{2.5}));
    // back in the pool, with its data
    for(auto && e : columns.pool_columns()) {
        if(e.seed == 1) {
            EXPECT_EQ(e.data.value, 2.5);
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
////////////// purge_pool erases matching pool columns, in order //////////////
///////////////////////////////////////////////////////////////////////////////

GTEST_TEST(column_manager, purge_pool_spares_master_columns) {
    manager columns;
    fake_model model;
    for(int seed : {1, 2, 3}) columns.emplace_pool_column(seed);
    columns.emplace_master_column(4, model.add_variable());

    const std::unordered_map<int, double> rc = {{1, -1.0}, {2, 0.5}, {3, 2.0}};
    columns.update_pool_columns(price_from(rc));

    // drop the unattractive pool columns to bound the pool's footprint
    auto purged = columns.purge_pool([](const auto & e) {
        return e.state.template get<reduced_cost>() >= 0.0;
    });

    EXPECT_EQ(purged, 2u);
    EXPECT_EQ(columns.num_pool_columns(), 1u);
    EXPECT_TRUE(columns.contains(1));
    EXPECT_FALSE(columns.contains(2));
    EXPECT_FALSE(columns.contains(3));
    // the master column survives whatever the predicate would say
    EXPECT_EQ(columns.num_master_columns(), 1u);
    EXPECT_TRUE(columns.contains(4));
}

GTEST_TEST(column_manager, purge_pool_keeps_the_order) {
    // activation follows the emplacement order, before and after a purge
    manager columns;
    fake_model model;
    for(int seed : {1, 2, 3, 4, 5, 6}) columns.emplace_pool_column(seed);
    columns.update_pool_columns(
        [](const auto & e) { return priced{e.seed % 2 == 0 ? 1.0 : -1.0}; });

    EXPECT_EQ(
        columns.purge_pool([](const auto & e) { return e.seed % 2 == 0; }), 3u);
    EXPECT_EQ(pool_seeds(columns), (std::vector<int>{1, 3, 5}));
    for(int seed : {1, 3, 5}) EXPECT_TRUE(columns.contains(seed));

    std::vector<int> activated_seeds;
    auto add_column = [&](fake_model & m, const auto & e) {
        activated_seeds.push_back(e.seed);
        return m.add_variable();
    };
    columns.activate_columns(model, negative<reduced_cost>{}, add_column,
                             at_most_k{2});
    EXPECT_EQ(activated_seeds, (std::vector<int>{1, 3}));
}

///////////////////////////////////////////////////////////////////////////////
//////////////// seed_hash hashes ranges and tuples by default ////////////////
///////////////////////////////////////////////////////////////////////////////

GTEST_TEST(column_manager, seed_hash_defers_to_std_hash) {
    EXPECT_EQ(seed_hash<int>{}(42), std::hash<int>{}(42));
    EXPECT_EQ(seed_hash<std::string>{}("abc"), std::hash<std::string>{}("abc"));
}

GTEST_TEST(column_manager, seed_hash_combines_ranges_and_tuples) {
    const seed_hash<std::vector<int>> h;
    EXPECT_EQ(h({1, 2}), h({1, 2}));
    EXPECT_NE(h({1, 2}), h({2, 1}));
    EXPECT_NE(h({}), h({0}));
    const seed_hash<std::tuple<int, std::vector<int>>> t;
    EXPECT_NE(t({1, {2}}), t({2, {1}}));

    column_manager<fake_model, std::vector<int>> columns;
    EXPECT_TRUE(columns.emplace_pool_column(std::vector<int>{1, 2}));
    EXPECT_TRUE(columns.emplace_pool_column(std::vector<int>{2, 1}));
    EXPECT_FALSE(columns.emplace_pool_column(std::vector<int>{1, 2}));
    EXPECT_TRUE(columns.contains({2, 1}));
}
