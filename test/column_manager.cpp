#undef NDEBUG
#include <gtest/gtest.h>

#include <algorithm>
#include <optional>
#include <unordered_map>
#include <variant>
#include <vector>

#include "mippp/utility/column_manager.hpp"

using namespace mippp;

namespace {

// A minimal stand-in for a solver model : the column manager only needs the
// 'variable' and 'scalar' nested types and lets the add/remove lambdas
// materialize the actual model modifications, so no real solver is required
// to unit test the pool <-> master bookkeeping and the selection strategies.
struct fake_model {
    using variable = int;
    using scalar = double;
    int next_variable = 0;
    int new_variable() { return next_variable++; }
};

// the status variant a solver's basis returns through get_status (see
// has_lp_basis in model_concepts.hpp), spelled out over the leaf statuses
// since the fake model carries no real basis
using fake_basis_status =
    std::variant<basis_status::basic, basis_status::nonbasic_free,
                 basis_status::nonbasic_at_lower_bound,
                 basis_status::nonbasic_at_upper_bound,
                 basis_status::nonbasic_fixed>;

// times_activated is listed in both states so that it survives the
// pool <-> master transitions (see transfer_common_properties)
using manager =
    column_manager<fake_model, int,
                   property_list<reduced_cost, age, times_activated>,
                   property_list<reduced_cost, age, variable_value,
                                 variable_status<fake_basis_status>,
                                 times_activated>>;

// reads a single column's property from the views (the states are only
// exposed through pool_columns()/master_columns())
template <typename P>
std::optional<typename P::value_type> pool_get(const manager & columns,
                                               int seed) {
    for(auto && [s, state] : columns.pool_columns())
        if(s == seed) return state.template get<P>();
    return std::nullopt;
}
template <typename P>
std::optional<typename P::value_type> master_get(const manager & columns,
                                                 int seed) {
    for(auto && [s, state] : columns.master_columns())
        if(s == seed) return state.template get<P>();
    return std::nullopt;
}

// the recurring add-column lambda of the tests that ignore the seeds
int make_variable(fake_model & m, const int &) { return m.new_variable(); }

// a pricing callback reading the reduced costs from the given table
auto price_from(const std::unordered_map<int, double> & rc) {
    return [&rc](const int & seed) { return priced{rc.at(seed)}; };
}

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
}

///////////////////////////////////////////////////////////////////////////////
////////// Emplacement partitions the columns between pool and master /////////
///////////////////////////////////////////////////////////////////////////////

GTEST_TEST(column_manager, emplacement_and_bookkeeping) {
    manager columns;
    fake_model model;
    ASSERT_TRUE(columns.emplace_column(1));
    ASSERT_FALSE(columns.emplace_column(1));  // already known
    ASSERT_TRUE(columns.emplace_master_column(2, model.new_variable()));
    ASSERT_FALSE(columns.emplace_master_column(2, model.new_variable()));

    EXPECT_EQ(columns.num_columns(), 2u);
    EXPECT_EQ(columns.num_pool_columns(), 1u);
    EXPECT_EQ(columns.num_master_columns(), 1u);
    EXPECT_TRUE(columns.contains(1));
    EXPECT_TRUE(columns.contains(2));
    EXPECT_FALSE(columns.contains(3));

    // master_variable distinguishes the two states
    EXPECT_FALSE(columns.master_variable(1).has_value());
    EXPECT_EQ(columns.master_variable(2), 0);
    EXPECT_FALSE(columns.master_variable(3).has_value());
    // registering an initial master column counts as an activation
    EXPECT_EQ(master_get<times_activated>(columns, 2), 1u);
}

GTEST_TEST(column_manager, emplace_columns_reports_duplicates) {
    manager columns;
    fake_model model;
    columns.emplace_column(1);
    columns.emplace_master_column(2, model.new_variable());

    // num_already_in_master flags a pricing oracle regenerating master
    // columns : stale duals or a cycling process
    auto result = columns.emplace_columns(std::vector<int>{1, 2, 3, 4});
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
    columns.emplace_column(1);
    columns.emplace_column(2);
    columns.emplace_column(3);
    ASSERT_EQ(columns.num_pool_columns(), 3u);

    const std::unordered_map<int, double> rc = {{1, -1.5}, {2, 2.0}, {3, -3.0}};
    columns.update_pool_columns(price_from(rc));

    for(int seed : {1, 2, 3}) {
        EXPECT_EQ(pool_get<reduced_cost>(columns, seed), rc.at(seed));
        EXPECT_EQ(pool_get<age>(columns, seed), 1u);  // one round elapsed
    }

    columns.update_pool_columns(price_from(rc));
    for(int seed : {1, 2, 3})
        EXPECT_EQ(pool_get<age>(columns, seed), 2u);
}

GTEST_TEST(column_manager, master_refresh_updates_value_and_basis_status) {
    // a multi-property event (reduced cost + primal value + basis status)
    // updates every carried property in a single broadcast
    manager columns;
    fake_model model;
    columns.emplace_master_column(10, model.new_variable());
    columns.emplace_column(20);  // stays in the pool

    columns.update_master_columns([&](const int &, const int & var) {
        return master_refreshed<fake_basis_status>{
            -4.0, 2.5 * var, basis_status::nonbasic_at_upper_bound{}};
    });

    EXPECT_EQ(master_get<reduced_cost>(columns, 10), -4.0);
    EXPECT_EQ(master_get<variable_value>(columns, 10), 0.0);  // var == 0
    const auto status =
        master_get<variable_status<fake_basis_status>>(columns, 10);
    ASSERT_TRUE(status.has_value());
    EXPECT_TRUE(std::holds_alternative<basis_status::nonbasic_at_upper_bound>(
        *status));
    EXPECT_EQ(master_get<age>(columns, 10), 1u);

    // the pool column was not touched by update_master_columns
    EXPECT_EQ(pool_get<age>(columns, 20), 0u);
}

GTEST_TEST(column_manager, update_columns_reaches_both_states) {
    manager columns;
    fake_model model;
    columns.emplace_column(1);
    columns.emplace_master_column(2, model.new_variable());

    // the single-callback overload broadcasts one event to every column
    columns.update_columns(
        [](const int & seed) { return priced{seed * 1.0}; });
    EXPECT_EQ(pool_get<reduced_cost>(columns, 1), 1.0);
    EXPECT_EQ(master_get<reduced_cost>(columns, 2), 2.0);

    // the two-callback overload builds a distinct event per state
    columns.update_columns(
        [](const int &) { return priced{-1.0}; },
        [](const int &, const int &) { return priced{-2.0}; });
    EXPECT_EQ(pool_get<reduced_cost>(columns, 1), -1.0);
    EXPECT_EQ(master_get<reduced_cost>(columns, 2), -2.0);
}

///////////////////////////////////////////////////////////////////////////////
//////////////// Strategies select exactly the matching columns ///////////////
///////////////////////////////////////////////////////////////////////////////

GTEST_TEST(column_manager, activation_selects_matching_columns) {
    // the add lambda observes each activated seed
    manager columns;
    fake_model model;
    for(int seed : {1, 2, 3, 4}) columns.emplace_column(seed);

    const std::unordered_map<int, double> rc = {
        {1, -1.0}, {2, 0.5}, {3, -2.0}, {4, 0.0}};
    columns.update_pool_columns(price_from(rc));

    std::vector<int> activated_seeds;
    auto add_column = [&](fake_model & m, const int & seed) {
        // the strategy only hands us columns with negative reduced cost
        EXPECT_LT(rc.at(seed), 0.0)
            << "activated a non-improving column (seed " << seed << ")";
        activated_seeds.push_back(seed);
        return m.new_variable();
    };

    auto result = columns.manage_columns(
        model, all<negative<reduced_cost>>{}, add_column);

    EXPECT_EQ(result.num_activated, 2u);
    EXPECT_EQ(columns.num_master_columns(), 2u);
    std::ranges::sort(activated_seeds);
    EXPECT_EQ(activated_seeds, (std::vector<int>{1, 3}));

    // partition check read from the states themselves
    EXPECT_TRUE(columns.master_variable(1).has_value());
    EXPECT_TRUE(columns.master_variable(3).has_value());
    EXPECT_FALSE(columns.master_variable(2).has_value());
    EXPECT_FALSE(columns.master_variable(4).has_value());
    for(auto && [seed, state] : columns.master_columns())
        EXPECT_LT(state.get<reduced_cost>(), 0.0);
}

GTEST_TEST(column_manager, eviction_reads_state_in_remove_lambda) {
    // the remove lambda reads the state of each evicted column and the
    // evicted columns fall back to the pool with their reduced cost preserved
    manager columns;
    fake_model model;
    for(int seed : {1, 2, 3})
        columns.emplace_master_column(seed, model.new_variable());
    ASSERT_EQ(columns.num_master_columns(), 3u);

    const std::unordered_map<int, double> rc = {{1, 1.0}, {2, -0.5}, {3, 3.0}};
    columns.update_master_columns(
        [&](const int & seed, const int &) { return priced{rc.at(seed)}; });

    std::vector<int> evicted_seeds;
    auto remove_columns = [&](fake_model &, auto && evicted_entries) {
        for(auto && [seed, state] : evicted_entries) {
            // reading the state property of a column selected for eviction
            EXPECT_GT(state.template get<reduced_cost>(), 0.0)
                << "evicted an attractive column (seed " << seed << ")";
            evicted_seeds.push_back(seed);
        }
    };

    auto result = columns.manage_columns(model, none{},
                                         all<positive<reduced_cost>>{},
                                         make_variable, remove_columns);

    EXPECT_EQ(result.num_evicted, 2u);
    std::ranges::sort(evicted_seeds);
    EXPECT_EQ(evicted_seeds, (std::vector<int>{1, 3}));

    EXPECT_EQ(columns.num_master_columns(), 1u);
    EXPECT_EQ(columns.num_pool_columns(), 2u);
    EXPECT_TRUE(columns.master_variable(2).has_value());  // attractive, kept
    // the reduced cost survives the master -> pool transition
    EXPECT_EQ(pool_get<reduced_cost>(columns, 1), 1.0);
    EXPECT_EQ(pool_get<reduced_cost>(columns, 3), 3.0);
    EXPECT_EQ(pool_get<age>(columns, 1), 0u);  // age reset on deactivation
}

GTEST_TEST(column_manager, at_most_k_best_ranks_by_state_property) {
    // keeps only the k best candidates according to a comparator reading a
    // state property (here : the most negative reduced costs)
    manager columns;
    fake_model model;
    for(int seed : {1, 2, 3, 4}) columns.emplace_column(seed);

    const std::unordered_map<int, double> rc = {
        {1, -1.0}, {2, -5.0}, {3, -0.2}, {4, -3.0}};
    columns.update_pool_columns(price_from(rc));

    std::vector<int> activated_seeds;
    auto add_column = [&](fake_model & m, const int & seed) {
        activated_seeds.push_back(seed);
        return m.new_variable();
    };

    // lesser reduced cost is better
    auto by_reduced_cost = [](const auto & a, const auto & b) {
        return a.second.template get<reduced_cost>() <
               b.second.template get<reduced_cost>();
    };
    auto result = columns.manage_columns(
        model,
        at_most_k_best{std::size_t{2}, by_reduced_cost,
                       negative<reduced_cost>{}},
        add_column);

    EXPECT_EQ(result.num_activated, 2u);
    std::ranges::sort(activated_seeds);
    EXPECT_EQ(activated_seeds, (std::vector<int>{2, 4}));  // -5.0 and -3.0
}

///////////////////////////////////////////////////////////////////////////////
///////// Properties survive or reset across pool <-> master transitions //////
///////////////////////////////////////////////////////////////////////////////

GTEST_TEST(column_manager, age_resets_and_times_activated_persists) {
    manager columns;
    fake_model model;
    columns.emplace_column(7);

    auto price = [](const int &) { return priced{-1.0}; };
    columns.update_pool_columns(price);
    columns.update_pool_columns(price);
    EXPECT_EQ(pool_get<age>(columns, 7), 2u);

    columns.manage_columns(model, all<negative<reduced_cost>>{}, make_variable);
    ASSERT_TRUE(columns.master_variable(7).has_value());
    EXPECT_EQ(master_get<age>(columns, 7), 0u);  // reset on activation
    EXPECT_EQ(master_get<times_activated>(columns, 7), 1u);

    // evict it back to the pool, then reactivate : times_activated accumulates
    columns.update_master_columns(
        [](const int &, const int &) { return priced{5.0}; });
    auto no_remove = [](fake_model &, auto &&) {};
    columns.manage_columns(model, none{}, all<positive<reduced_cost>>{},
                           make_variable, no_remove);
    ASSERT_EQ(columns.num_pool_columns(), 1u);
    EXPECT_EQ(pool_get<age>(columns, 7), 0u);  // reset on deactivation too

    columns.update_pool_columns(price);
    columns.manage_columns(model, all<negative<reduced_cost>>{}, make_variable);
    EXPECT_EQ(master_get<times_activated>(columns, 7), 2u);
}

///////////////////////////////////////////////////////////////////////////////
/////////////// purge_pool erases matching pool columns only //////////////////
///////////////////////////////////////////////////////////////////////////////

GTEST_TEST(column_manager, purge_pool_spares_master_columns) {
    manager columns;
    fake_model model;
    for(int seed : {1, 2, 3}) columns.emplace_column(seed);
    columns.emplace_master_column(4, model.new_variable());

    const std::unordered_map<int, double> rc = {{1, -1.0}, {2, 0.5}, {3, 2.0}};
    columns.update_pool_columns(price_from(rc));

    // drop the unattractive pool columns to bound the pool's footprint
    auto purged = columns.purge_pool([](const auto & entry) {
        return std::get<1>(entry).template get<reduced_cost>() >= 0.0;
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
