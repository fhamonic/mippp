#include <gtest/gtest.h>

#include <ranges>
#include <tuple>
#include <vector>

#include "mippp/detail/cartesian_product_view.hpp"

// `detail::cartesian_product` is std::views::cartesian_product where the
// standard library has it and a local view elsewhere (libc++, or any build
// defining MIPPP_PORTABLE_RANGE_SHAPES). Both are covered by these tests;
// they pin the properties the expression layer and the test suites rely on.

using mippp::detail::cartesian_product;

GTEST_TEST(cartesian_product_view, pairs_in_row_major_order) {
    std::vector<int> a = {1, 2};
    std::vector<char> b = {'x', 'y', 'z'};
    std::vector<std::tuple<int, char>> pairs;
    for(auto && [i, c] : cartesian_product(a, b)) pairs.emplace_back(i, c);
    ASSERT_EQ(pairs,
              (std::vector<std::tuple<int, char>>{
                  {1, 'x'}, {1, 'y'}, {1, 'z'}, {2, 'x'}, {2, 'y'}, {2, 'z'}}));
}

GTEST_TEST(cartesian_product_view, empty_operand_empties_the_product) {
    std::vector<int> some = {1, 2, 3};
    std::vector<int> none;
    ASSERT_TRUE(std::ranges::empty(cartesian_product(none, some)));
    ASSERT_TRUE(std::ranges::empty(cartesian_product(some, none)));
    ASSERT_TRUE(std::ranges::empty(cartesian_product(none, none)));
}

GTEST_TEST(cartesian_product_view, is_a_sized_forward_view) {
    auto p = cartesian_product(std::views::iota(0, 4), std::views::iota(0, 5));
    static_assert(std::ranges::view<decltype(p)>);
    static_assert(std::ranges::forward_range<decltype(p)>);
    ASSERT_EQ(std::ranges::size(p), 20u);
    // multipass: a second walk sees the same elements
    int first_walk = 0, second_walk = 0;
    for(auto && [i, j] : p) first_walk += i * j;
    for(auto && [i, j] : p) second_walk += i * j;
    ASSERT_EQ(first_walk, second_walk);
    ASSERT_EQ(first_walk, 6 * 10);
}

GTEST_TEST(cartesian_product_view, lvalue_operands_yield_references) {
    std::vector<int> a = {1, 2};
    std::vector<int> b = {10};
    for(auto && [i, j] : cartesian_product(a, b)) {
        static_assert(std::is_same_v<decltype(i), int &>);
        i += j;
    }
    ASSERT_EQ(a, (std::vector<int>{11, 12}));
}

GTEST_TEST(cartesian_product_view, composes_with_filter_and_nesting) {
    auto items = std::views::iota(0, 4);
    auto strict_pairs = std::views::filter(
        cartesian_product(items, items),
        [](auto && p) { return std::get<0>(p) < std::get<1>(p); });
    ASSERT_EQ(std::ranges::distance(strict_pairs), 6);
    // the sudoku suite nests one product inside another
    auto coords =
        cartesian_product(std::views::iota(0, 2), std::views::iota(0, 2));
    int count = 0;
    for(auto && [v, b] : cartesian_product(std::views::iota(1, 4), coords)) {
        count += v * (std::get<0>(b) + std::get<1>(b));
    }
    ASSERT_EQ(count, (1 + 2 + 3) * (0 + 1 + 1 + 2));
}
