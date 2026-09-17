#undef NDEBUG
#include <gtest/gtest.h>

#include <functional>
#include <ranges>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "mippp/detail/cartesian_product_view.hpp"
#include "mippp/model_entities.hpp"
#include "mippp/utility/constraints_range.hpp"
#include "mippp/utility/keys_view.hpp"

// constraints_range resolves a key to its constraint through a strategy
// picked from the type of the key range. Each test pins the strategy chosen
// for one shape of keys and the positions it resolves.

using constraint = mippp::model_constraint<int>;
using mippp::constraints_range;
using mippp::indexed;
using mippp::indexed_named;
using mippp::named;
namespace detail = mippp::detail;

static std::vector<int> ids_of(const auto & constraints) {
    std::vector<int> ids;
    for(auto c : constraints) ids.push_back(c.id());
    return ids;
}

GTEST_TEST(constraints_range, contiguous_ids_from_the_first_constraint) {
    auto keys = std::views::iota(0, 3);
    constraints_range c(keys, constraint{5}, 3);
    static_assert(std::ranges::random_access_range<decltype(c)>);
    static_assert(std::ranges::sized_range<decltype(c)>);
    ASSERT_EQ(c.size(), 3u);
    ASSERT_EQ(ids_of(c), (std::vector<int>{5, 6, 7}));
    ASSERT_EQ(c[0].id(), 5);
    ASSERT_EQ(c[2].id(), 7);
    ASSERT_THROW(c[3], std::out_of_range);
}

GTEST_TEST(constraints_range, copyable_and_assignable) {
    auto keys = std::views::iota(0, 3);
    constraints_range c(keys, constraint{5}, 3);
    auto copy = c;
    c = copy;
    ASSERT_EQ(c(1).id(), 6);
    ASSERT_EQ(copy(2).id(), 7);
}

GTEST_TEST(constraints_range, iota_keys_resolve_arithmetically) {
    constraints_range c(std::views::iota(-2, 2), constraint{10}, 4);
    static_assert(
        std::same_as<
            decltype(c),
            constraints_range<int, constraint, detail::affine_index<int>>>);
    ASSERT_EQ(c(-2).id(), 10);
    ASSERT_EQ(c(1).id(), 13);
    ASSERT_THROW(c(-3), std::out_of_range);
    ASSERT_THROW(c(2), std::out_of_range);
}

GTEST_TEST(constraints_range, unsigned_iota_keys) {
    std::vector<int> v(4);
    constraints_range c(std::views::iota(std::size_t{0}, v.size()),
                        constraint{0}, 4);
    static_assert(
        std::same_as<decltype(c),
                     constraints_range<std::size_t, constraint,
                                       detail::affine_index<std::size_t>>>);
    ASSERT_EQ(c(3).id(), 3);
    ASSERT_THROW(c(4), std::out_of_range);
}

GTEST_TEST(constraints_range, product_keys_resolve_row_major) {
    auto keys = detail::cartesian_product(std::views::iota(0, 2),
                                          std::views::iota(0, 3));
    constraints_range c(keys, constraint{1}, 6);
    static_assert(
        std::same_as<
            decltype(c),
            constraints_range<std::tuple<int, int>, constraint,
                              detail::affine_index<std::tuple<int, int>>>>);
    ASSERT_EQ(c(std::tuple{1, 2}).id(), 6);
    ASSERT_EQ(c(1, 2).id(), 6);
    ASSERT_EQ(c(0, 1).id(), 2);
    ASSERT_EQ(c(std::pair{1, 0}).id(), 4);
    ASSERT_THROW(c(2, 0), std::out_of_range);
    ASSERT_THROW(c(0, 3), std::out_of_range);
    ASSERT_THROW(c(-1, 0), std::out_of_range);
    ASSERT_THROW(c(1, -1), std::out_of_range);
}

GTEST_TEST(constraints_range, nested_product_keys) {
    auto inner = detail::cartesian_product(std::views::iota(0, 2),
                                           std::views::iota(0, 2));
    auto keys = detail::cartesian_product(std::views::iota(0, 2), inner);
    constraints_range c(keys, constraint{0}, 8);
    ASSERT_EQ(c(1, std::tuple{1, 0}).id(), 6);
    ASSERT_EQ(c(std::tuple{0, std::tuple{1, 1}}).id(), 3);
    ASSERT_THROW(c(0, std::tuple{2, 0}), std::out_of_range);
}

GTEST_TEST(constraints_range, empty_product_keys) {
    auto keys = detail::cartesian_product(std::views::iota(0, 2),
                                          std::views::iota(0, 0));
    constraints_range c(keys, constraint{0}, 0);
    ASSERT_EQ(c.size(), 0u);
    ASSERT_THROW(c(0, 0), std::out_of_range);
}

// The gate names detail::cartesian_product_view, the standard view only
// where the fallback is not forced in.
#if defined(__cpp_lib_ranges_cartesian_product) && \
    !defined(MIPPP_PORTABLE_RANGE_SHAPES)
GTEST_TEST(constraints_range, standard_n_ary_product_keys) {
    auto keys = std::views::cartesian_product(
        std::views::iota(0, 2), std::views::iota(5, 8), std::views::iota(0, 4));
    constraints_range c(keys, constraint{0}, 24);
    static_assert(
        std::same_as<decltype(c),
                     constraints_range<
                         std::tuple<int, int, int>, constraint,
                         detail::affine_index<std::tuple<int, int, int>>>>);
    ASSERT_EQ(c(1, 6, 3).id(), 1 * 12 + 1 * 4 + 3);
    ASSERT_THROW(c(1, 4, 3), std::out_of_range);
    ASSERT_THROW(c(1, 8, 3), std::out_of_range);
}
#endif

GTEST_TEST(constraints_range, hashable_keys_use_a_hash_map) {
    std::vector<std::string> keys = {"a", "b", "c", "a"};
    constraints_range c(keys, constraint{0}, 4);
    static_assert(
        std::same_as<decltype(c),
                     constraints_range<std::string, constraint,
                                       detail::hash_key_index<std::string>>>);
    ASSERT_EQ(c("b").id(), 1);
    ASSERT_EQ(c("a").id(), 0);  // a duplicate key keeps its first constraint
    ASSERT_THROW(c("z"), std::out_of_range);
}

GTEST_TEST(constraints_range, ordered_keys_use_a_sorted_vector) {
    std::vector<std::pair<int, int>> keys = {{2, 1}, {1, 5}, {2, 1}, {0, 0}};
    constraints_range c(keys, constraint{3}, 4);
    static_assert(
        std::same_as<
            decltype(c),
            constraints_range<std::pair<int, int>, constraint,
                              detail::sorted_key_index<std::pair<int, int>>>>);
    ASSERT_EQ(c({1, 5}).id(), 4);
    ASSERT_EQ(c({2, 1}).id(), 3);
    ASSERT_EQ(c({0, 0}).id(), 6);
    ASSERT_EQ(c(0, 0).id(), 6);
    ASSERT_THROW(c({1, 1}), std::out_of_range);
    ASSERT_THROW(c({3, 0}), std::out_of_range);
}

struct order {
    int id;
};

GTEST_TEST(constraints_range, indexed_keys_use_a_table) {
    std::vector<order> orders = {{4}, {1}, {7}, {1}};
    constraints_range c(indexed(orders, &order::id), constraint{0}, 4);
    static_assert(
        std::same_as<decltype(c),
                     constraints_range<order, constraint,
                                       detail::table_key_index<int order::*>>>);
    ASSERT_EQ(c(order{4}).id(), 0);
    ASSERT_EQ(c(order{1}).id(), 1);
    ASSERT_EQ(c(order{7}).id(), 2);
    ASSERT_THROW(c(order{5}), std::out_of_range);
    ASSERT_THROW(c(order{-1}), std::out_of_range);
    ASSERT_THROW(c(order{8}), std::out_of_range);
}

GTEST_TEST(constraints_range, indexed_filtered_keys) {
    auto keys = indexed(std::views::iota(0, 10) | std::views::filter([](int i) {
                            return i % 2 == 0;
                        }),
                        std::identity{});
    constraints_range c(keys, constraint{0}, 5);
    ASSERT_EQ(c(4).id(), 2);
    ASSERT_EQ(c(8).id(), 4);
    ASSERT_THROW(c(3), std::out_of_range);
    ASSERT_THROW(c(10), std::out_of_range);
}

GTEST_TEST(constraints_range, indexed_keys_reject_negative_ids) {
    std::vector<int> keys = {1, -1};
    ASSERT_THROW(
        constraints_range(indexed(keys, std::identity{}), constraint{0}, 2),
        std::invalid_argument);
}

struct opaque {
    int rank = 0;
};

namespace user {
// a range that knows where each of its keys is: every key is its own position
struct positional_keys {
    std::vector<opaque> keys;
    auto begin() const { return keys.begin(); }
    auto end() const { return keys.end(); }
};
struct positional_index {
    std::size_t count;
    std::size_t position(const opaque & key) const {
        const auto rank = static_cast<std::size_t>(key.rank);
        return rank < count ? rank : mippp::npos;
    }
};
positional_index key_index(const positional_keys & keys) {
    return {keys.keys.size()};
}
}  // namespace user

GTEST_TEST(constraints_range, a_range_can_supply_its_own_index) {
    static_assert(mippp::key_index_for<user::positional_index, opaque>);
    user::positional_keys keys{{{0}, {1}, {2}}};
    constraints_range c(keys, constraint{4}, 3);
    static_assert(
        std::same_as<decltype(c), constraints_range<opaque, constraint,
                                                    user::positional_index>>);
    ASSERT_EQ(c(opaque{1}).id(), 5);
    ASSERT_THROW(c(opaque{3}), std::out_of_range);
}

GTEST_TEST(constraints_range, unindexable_keys_stay_iterable) {
    std::vector<opaque> keys(2);
    constraints_range c(keys, constraint{0}, 2);
    static_assert(
        std::same_as<decltype(c), constraints_range<opaque, constraint,
                                                    detail::no_key_index>>);
    ASSERT_EQ(ids_of(c), (std::vector<int>{0, 1}));
    ASSERT_EQ(c[1].id(), 1);
}

GTEST_TEST(constraints_range, single_pass_keys_stay_iterable) {
    std::istringstream in("1 2 3");
    auto keys = std::views::istream<int>(in);
    constraints_range c(keys, constraint{0}, 3);
    static_assert(
        std::same_as<decltype(c),
                     constraints_range<int, constraint, detail::no_key_index>>);
    ASSERT_EQ(c.size(), 3u);
}

// --- keys wrappers

struct order_name {
    std::string operator()(const order &) const { return ""; }
};
template <typename K>
concept renamable = requires(K & k) { named(k, order_name{}); };
template <typename K>
concept reindexable = requires(K & k) { indexed(k, &order::id); };

GTEST_TEST(keys_view, each_wrapper_carries_its_functions) {
    std::vector<order> orders = {{4}, {1}};
    auto by_id = indexed(orders, &order::id);
    static_assert(decltype(by_id)::has_id && !decltype(by_id)::has_name);
    auto by_name = named(orders, [](const order & o) {
        return "demand_" + std::to_string(o.id);
    });
    static_assert(!decltype(by_name)::has_id && decltype(by_name)::has_name);
    auto both = indexed_named(orders, &order::id, [](const order & o) {
        return "demand_" + std::to_string(o.id);
    });
    static_assert(decltype(both)::has_id && decltype(both)::has_name);
    ASSERT_EQ(std::ranges::distance(both), 2);
    ASSERT_EQ(std::invoke(both.id_fn(), orders[0]), 4);
    ASSERT_EQ(both.name_fn()(orders[1]), "demand_1");
    // wrappers do not nest
    static_assert(!renamable<decltype(by_id)>);
    static_assert(!reindexable<decltype(by_name)>);
}

GTEST_TEST(keys_view, a_name_does_not_change_the_index_strategy) {
    auto keys =
        named(std::views::iota(0, 3), [](int i) { return std::to_string(i); });
    static_assert(std::same_as<detail::key_index_t<decltype(keys)>,
                               detail::affine_index<int>>);
    constraints_range c(keys, constraint{4}, 3);
    ASSERT_EQ(c(2).id(), 6);
    ASSERT_THROW(c(3), std::out_of_range);
}

struct recording_model {
    std::vector<std::pair<int, std::string>> names;
    void set_constraint_name(constraint c, const std::string & name) {
        names.emplace_back(c.id(), name);
    }
};
struct nameless_model {};

GTEST_TEST(keys_view, names_are_applied_in_key_order_from_the_first_id) {
    std::vector<order> orders = {{4}, {1}};
    auto keys = named(orders, [](const order & o) {
        return "demand_" + std::to_string(o.id);
    });
    recording_model model;
    detail::name_constraints(model, keys, constraint{7});
    ASSERT_EQ(model.names, (std::vector<std::pair<int, std::string>>{
                               {7, "demand_4"}, {8, "demand_1"}}));
}

GTEST_TEST(keys_view, unnamed_keys_never_touch_the_model) {
    std::vector<int> keys = {1, 2};
    nameless_model model;
    detail::name_constraints(model, keys, constraint{0});
    auto indexed_keys = indexed(keys, std::identity{});
    detail::name_constraints(model, indexed_keys, constraint{0});
}
