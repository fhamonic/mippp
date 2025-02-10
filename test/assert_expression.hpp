#ifndef EXPRESSION_TEST_HELPER_HPP
#define EXPRESSION_TEST_HELPER_HPP

#include <initializer_list>
#include <ranges>

#include "assert_eq_ranges.hpp"

template <typename Expr, typename V, typename C, typename S>
void ASSERT_EXPRESSION(Expr && expr, std::initializer_list<V> vars,
                       std::initializer_list<C> coefs, S && scalar) {
    ASSERT_EQ_RANGES(expr.terms(), ranges::views::zip(coefs, vars));
    //  ASSERT_EQ_RANGES(std::ranges::views::keys(expr.terms()),
    //                   ranges::views::zip(coefs, vars));
    //  ASSERT_EQ_RANGES(std::ranges::views::values(expr.terms()),
    //                   ranges::views::zip(coefs, vars));
    ASSERT_EQ(expr.constant(), scalar);
}

#endif  // EXPRESSION_TEST_HELPER_HPP