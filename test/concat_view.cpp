#undef NDEBUG
#include <gtest/gtest.h>

#include <algorithm>
#include <list>
#include <ranges>
#include <vector>

#include "mippp/detail/concat_view.hpp"

// `unordered_concat` has two implementations selected on __cpp_lib_ranges_concat
// (the std::views::concat one additionally swaps its operands on the GCC 15.1
// bug path). Both must accept *any* viewable_range -- notably plain containers,
// which is what a runtime_linear_expression hands over -- and both are free to
// produce the terms in either order, so every assertion below is written to be
// order-insensitive.

using mippp::detail::unordered_concat;

namespace {

template <std::ranges::input_range R>
std::vector<std::ranges::range_value_t<R>> sorted_elements(R && r) {
    std::vector<std::ranges::range_value_t<R>> v;
    for(auto && e : r) v.emplace_back(e);
    std::ranges::sort(v);
    return v;
}

template <typename R>
void ASSERT_ELEMENTS(R && r,
                     std::vector<std::ranges::range_value_t<R>> expected) {
    std::ranges::sort(expected);
    ASSERT_EQ(sorted_elements(r), expected);
}

}  // namespace

///////////////////////////////////////////////////////////////////////////////
/////////////////////////////// Range categories //////////////////////////////
///////////////////////////////////////////////////////////////////////////////

using two_vectors_t =
    decltype(unordered_concat(std::declval<std::vector<int> &>(),
                              std::declval<std::vector<int> &>()));

static_assert(std::ranges::view<two_vectors_t>);
static_assert(std::ranges::forward_range<two_vectors_t>);
static_assert(std::same_as<std::ranges::range_value_t<two_vectors_t>, int>);

///////////////////////////////////////////////////////////////////////////////
////////////////////////////// Accepted operands //////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// regression: the fallback concat_view used to be handed the raw operands, so
// CTAD deduced V1 = std::vector and failed the `view<V1>` constraint. Adding
// two containers -- e.g. two materialized linear expressions -- did not compile.
GTEST_TEST(unordered_concat, two_lvalue_containers) {
    std::vector<int> a = {1, 2, 3};
    std::vector<int> b = {4, 5};
    ASSERT_ELEMENTS(unordered_concat(a, b), {1, 2, 3, 4, 5});
}

GTEST_TEST(unordered_concat, const_lvalue_containers) {
    const std::vector<int> a = {1, 2, 3};
    const std::vector<int> b = {4, 5};
    ASSERT_ELEMENTS(unordered_concat(a, b), {1, 2, 3, 4, 5});
}

// a container passed as an rvalue must be *owned* by the result, not referenced
GTEST_TEST(unordered_concat, rvalue_containers_are_owned) {
    auto c = unordered_concat(std::vector<int>{1, 2, 3}, std::vector<int>{4, 5});
    ASSERT_ELEMENTS(c, {1, 2, 3, 4, 5});
}

GTEST_TEST(unordered_concat, container_and_view) {
    std::vector<int> a = {1, 2, 3};
    ASSERT_ELEMENTS(
        unordered_concat(a, std::views::transform(a, [](int i) { return -i; })),
        {1, 2, 3, -1, -2, -3});
}

GTEST_TEST(unordered_concat, two_views) {
    std::vector<int> a = {1, 2, 3, 4};
    ASSERT_ELEMENTS(
        unordered_concat(std::views::filter(a, [](int i) { return i % 2; }),
                         std::views::take(a, 2)),
        {1, 3, 1, 2});
}

// mixing an lvalue-reference range with one yielding prvalues exercises the
// common_reference requirements of concat_view_compatible
GTEST_TEST(unordered_concat, mixed_reference_and_prvalue_elements) {
    std::vector<int> a = {1, 2};
    auto doubled = std::views::transform(a, [](int i) { return i * 2; });
    ASSERT_ELEMENTS(unordered_concat(a, doubled), {1, 2, 2, 4});
}

// a non-contiguous, non-sized source: unordered_concat must not require more
// than an input range
GTEST_TEST(unordered_concat, non_contiguous_container) {
    std::list<int> a = {1, 2};
    std::vector<int> b = {3};
    ASSERT_ELEMENTS(unordered_concat(a, b), {1, 2, 3});
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////// Edge cases //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

GTEST_TEST(unordered_concat, empty_first_operand) {
    std::vector<int> a;
    std::vector<int> b = {1, 2};
    ASSERT_ELEMENTS(unordered_concat(a, b), {1, 2});
}

GTEST_TEST(unordered_concat, empty_second_operand) {
    std::vector<int> a = {1, 2};
    std::vector<int> b;
    ASSERT_ELEMENTS(unordered_concat(a, b), {1, 2});
}

GTEST_TEST(unordered_concat, both_empty) {
    std::vector<int> a;
    std::vector<int> b;
    auto c = unordered_concat(a, b);
    ASSERT_EQ(std::ranges::distance(c), 0);
    ASSERT_TRUE(std::ranges::begin(c) == std::ranges::end(c));
}

GTEST_TEST(unordered_concat, nested_concat) {
    std::vector<int> a = {1};
    std::vector<int> b = {2};
    std::vector<int> c = {3};
    ASSERT_ELEMENTS(unordered_concat(unordered_concat(a, b), c), {1, 2, 3});
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////// Multi-pass //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// forward_range means a second traversal sees the same elements -- expression
// consumers that traverse twice (e.g. reserve-then-fill) rely on this
GTEST_TEST(unordered_concat, multipass_is_stable) {
    std::vector<int> a = {1, 2, 3};
    std::vector<int> b = {4, 5};
    auto c = unordered_concat(a, b);
    ASSERT_EQ(sorted_elements(c), sorted_elements(c));
    ASSERT_EQ(std::ranges::distance(c), 5);
    ASSERT_EQ(std::ranges::distance(c), 5);
}

// writing through the concatenation must reach the underlying containers
GTEST_TEST(unordered_concat, references_the_source_containers) {
    std::vector<int> a = {1, 2};
    std::vector<int> b = {3};
    for(auto && e : unordered_concat(a, b)) e *= 10;
    ASSERT_EQ(a, (std::vector<int>{10, 20}));
    ASSERT_EQ(b, (std::vector<int>{30}));
}
