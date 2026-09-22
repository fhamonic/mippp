#include <gtest/gtest.h>

#include <array>
#include <concepts>
#include <string>
#include <tuple>
#include <utility>

#include "mippp/detail/invoke_key.hpp"

// invoke_key calls a function with the key when that is viable, and unpacks a
// std::pair or std::tuple key into one argument per element otherwise. Each
// test pins which of the two calls one shape of lambda gets.

using mippp::detail::invoke_key;
using mippp::detail::key_invocable;
using mippp::detail::key_invoke_result_t;

GTEST_TEST(invoke_key, plain_keys_are_passed_through) {
    ASSERT_EQ(invoke_key([](int i) { return i + 1; }, 41), 42);
    static_assert(!key_invocable<decltype([](int, int) {}), int>);
}

GTEST_TEST(invoke_key, an_arity_matching_lambda_receives_the_elements) {
    std::tuple key{2, 3};
    ASSERT_EQ(invoke_key([](int i, int j) { return 10 * i + j; }, key), 23);
    ASSERT_EQ(invoke_key([](auto i, auto j) { return 10 * i + j; }, key), 23);
    ASSERT_EQ(
        invoke_key([](int i, int j) { return 10 * i + j; }, std::pair{4, 5}),
        45);
    ASSERT_EQ(invoke_key([](const int & i, int && j) { return 10 * i + j; },
                         std::tuple{6, 7}),
              67);
}

GTEST_TEST(invoke_key, a_lambda_taking_the_key_wins_over_unpacking) {
    std::pair<int, int> key{2, 3};
    ASSERT_EQ(invoke_key([](std::pair<int, int> p) { return p.first; }, key),
              2);
    ASSERT_EQ(invoke_key(
                  [](auto && p) {
                      auto && [i, j] = p;
                      return 10 * i + j;
                  },
                  key),
              23);
    // a variadic lambda accepts both calls: it gets the whole key
    static_assert(
        std::same_as<key_invoke_result_t<decltype([](auto &&... args) {
                                             return sizeof...(args);
                                         }),
                                         std::tuple<int, int>>,
                     std::size_t>);
    ASSERT_EQ(invoke_key([](auto &&... args) { return sizeof...(args); },
                         std::tuple{1, 2}),
              1u);
}

GTEST_TEST(invoke_key, only_pairs_and_tuples_are_unpacked) {
    static_assert(
        key_invocable<decltype([](int, int) {}), std::tuple<int, int>>);
    static_assert(
        key_invocable<decltype([](int, int) {}), std::pair<int, int>>);
    static_assert(
        !key_invocable<decltype([](int, int) {}), std::array<int, 2>>);
    static_assert(key_invocable<decltype([]() {}), std::tuple<>>);
    static_assert(
        !key_invocable<decltype([](int, int, int) {}), std::tuple<int, int>>);
}

GTEST_TEST(invoke_key, elements_are_forwarded_with_the_key_value_category) {
    auto s = invoke_key([](std::string && a, int) { return std::move(a); },
                        std::tuple{std::string("moved"), 1});
    ASSERT_EQ(s, "moved");
    const std::tuple<std::string, int> key{"copied", 2};
    static_assert(
        std::same_as<
            key_invoke_result_t<decltype([](const std::string & a,
                                            int) -> const auto & { return a; }),
                                const std::tuple<std::string, int> &>,
            const std::string &>);
    ASSERT_EQ(invoke_key([](const std::string & a, int) { return a; }, key),
              "copied");
}
