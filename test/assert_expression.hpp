#ifndef EXPRESSION_TEST_HELPER_HPP
#define EXPRESSION_TEST_HELPER_HPP

#include <initializer_list>

#include "assert_eq_ranges.hpp"

template <typename Expr, typename V, typename C, typename S>
void ASSERT_EXPRESSION(Expr && expr, std::initializer_list<V> vars,
                       std::initializer_list<C> coefs, S && scalar) {
    ASSERT_EQ_RANGES(expr.variables(), vars);
    ASSERT_EQ_RANGES(expr.coefficients(), coefs);
    ASSERT_EQ(expr.constant(), scalar);
}

#endif //EXPRESSION_TEST_HELPER_HPP