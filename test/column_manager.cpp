#undef NDEBUG
#include <gtest/gtest.h>

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
template <typename P, typename Columns>
std::optional<typename P::value_type> pool_get(const Columns & columns,
                                               int seed) {
    for(auto && [s, state] : columns.pool_columns())
        if(s == seed) return state.template get<P>();
    return std::nullopt;
}
template <typename P, typename Columns>
std::optional<typename P::value_type> master_get(const Columns & columns,
                                                 int seed) {
    for(auto && [s, state] : columns.master_columns())
        if(s == seed) return state.template get<P>();
    return std::nullopt;
}

}  // namespace

// events broadcast by update_* must actually reach the properties, and 'age'
// must tick once per round that carries a reduced cost
TEST(ColumnManager, PoolPropertiesAreUpdatedByEvents) {
    manager columns;
    columns.emplace_column(1);
    columns.emplace_column(2);
    columns.emplace_column(3);
    ASSERT_EQ(columns.num_pool_columns(), 3u);

    const std::unordered_map<int, double> rc = {{1, -1.5}, {2, 2.0}, {3, -3.0}};
    columns.update_pool_columns(
        [&](const int & seed) { return priced{rc.at(seed)}; });

    for(int seed : {1, 2, 3}) {
        EXPECT_EQ(pool_get<reduced_cost>(columns, seed), rc.at(seed));
        EXPECT_EQ(pool_get<age>(columns, seed), 1u);  // one round elapsed
    }

    columns.update_pool_columns(
        [&](const int & seed) { return priced{rc.at(seed)}; });
    for(int seed : {1, 2, 3})
        EXPECT_EQ(pool_get<age>(columns, seed), 2u);
}

// a multi-property event (reduced cost + primal value + basis status) updates
// every carried property in a single broadcast, and skips the pool columns
TEST(ColumnManager, MasterRefreshedUpdatesValueAndBasisStatus) {
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

// the activation strategy must select exactly the columns satisfying its
// predicate ; the add lambda observes each activated seed
TEST(ColumnManager, ActivationSelectsColumnsMatchingPredicate) {
    manager columns;
    fake_model model;
    for(int seed : {1, 2, 3, 4}) columns.emplace_column(seed);

    const std::unordered_map<int, double> rc = {
        {1, -1.0}, {2, 0.5}, {3, -2.0}, {4, 0.0}};
    columns.update_pool_columns(
        [&](const int & seed) { return priced{rc.at(seed)}; });

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

// the eviction strategy must select exactly the columns satisfying its
// predicate ; the remove lambda reads the state of each evicted column and the
// evicted columns fall back to the pool with their reduced cost preserved
TEST(ColumnManager, EvictionReadsStateInRemoveLambda) {
    manager columns;
    fake_model model;
    for(int seed : {1, 2, 3})
        columns.emplace_master_column(seed, model.new_variable());
    ASSERT_EQ(columns.num_master_columns(), 3u);

    const std::unordered_map<int, double> rc = {{1, 1.0}, {2, -0.5}, {3, 3.0}};
    columns.update_master_columns(
        [&](const int & seed, const int &) { return priced{rc.at(seed)}; });

    std::vector<int> evicted_seeds;
    auto add_column = [&](fake_model & m, const int &) { return m.new_variable(); };
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
                                         add_column, remove_columns);

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

// at_most_k_best keeps only the k best candidates according to a comparator
// reading a state property (here : the most negative reduced costs)
TEST(ColumnManager, AtMostKBestActivationRanksByStateProperty) {
    manager columns;
    fake_model model;
    for(int seed : {1, 2, 3, 4}) columns.emplace_column(seed);

    const std::unordered_map<int, double> rc = {
        {1, -1.0}, {2, -5.0}, {3, -0.2}, {4, -3.0}};
    columns.update_pool_columns(
        [&](const int & seed) { return priced{rc.at(seed)}; });

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

// age resets when a column enters the master, times_activated persists across
// the pool <-> master round trips
TEST(ColumnManager, AgeResetsAndTimesActivatedPersists) {
    manager columns;
    fake_model model;
    columns.emplace_column(7);

    auto price = [](const int &) { return priced{-1.0}; };
    columns.update_pool_columns(price);
    columns.update_pool_columns(price);
    EXPECT_EQ(pool_get<age>(columns, 7), 2u);

    auto add_column = [&](fake_model & m, const int &) { return m.new_variable(); };
    columns.manage_columns(model, all<negative<reduced_cost>>{}, add_column);
    ASSERT_TRUE(columns.master_variable(7).has_value());
    EXPECT_EQ(master_get<age>(columns, 7), 0u);  // reset on activation
    EXPECT_EQ(master_get<times_activated>(columns, 7), 1u);

    // evict it back to the pool, then reactivate : times_activated accumulates
    columns.update_master_columns(
        [](const int &, const int &) { return priced{5.0}; });
    auto no_remove = [](fake_model &, auto &&) {};
    columns.manage_columns(model, none{}, all<positive<reduced_cost>>{},
                           add_column, no_remove);
    ASSERT_EQ(columns.num_pool_columns(), 1u);
    EXPECT_EQ(pool_get<age>(columns, 7), 0u);  // reset on deactivation too

    columns.update_pool_columns(price);
    columns.manage_columns(model, all<negative<reduced_cost>>{}, add_column);
    EXPECT_EQ(master_get<times_activated>(columns, 7), 2u);
}
