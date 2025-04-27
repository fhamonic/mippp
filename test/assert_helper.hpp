#ifndef ASSERT_HELPER_HPP
#define ASSERT_HELPER_HPP

#include <initializer_list>
#include <ranges>

#include <range/v3/view/map.hpp>
#include <range/v3/view/zip.hpp>

template <typename R1, typename R2>
void ASSERT_EQ_RANGES(R1 && r1, R2 && r2) {
    ASSERT_EQ(std::ranges::distance(r1), std::ranges::distance(r2));
    for(const auto & [e1, e2] : ranges::views::zip(r1, r2)) {
        ASSERT_EQ(e1, e2);
    }
}

template <typename R1, typename T>
    requires std::same_as<std::ranges::range_value_t<R1>, T>
void ASSERT_EQ_RANGES(R1 && r1, std::initializer_list<T> l) {
    ASSERT_EQ(ranges::distance(r1), l.size());
    for(const auto & [e1, e2] : ranges::views::zip(r1, l)) {
        ASSERT_EQ(e1, e2);
    }
}

template <typename Expr, typename V, typename C, typename S>
void ASSERT_EXPRESSION(Expr && expr, std::initializer_list<V> vars,
                       std::initializer_list<C> coefs, S && scalar) {
    ASSERT_EQ_RANGES(ranges::view::keys(expr.linear_terms()), vars);
    ASSERT_EQ_RANGES(ranges::view::values(expr.linear_terms()), coefs);
    ASSERT_EQ(expr.constant(), scalar);
}

#include "range/v3/core.hpp"

template <typename Constr, typename V, typename C, typename S, typename B>
void ASSERT_CONSTRAINT(Constr && constr, std::initializer_list<V> vars,
                       std::initializer_list<C> coefs, S && rel, B && bound) {
    // ASSERT_EQ_RANGES(ranges::view::keys(constr.linear_terms()),
    //                  vars);
    // ASSERT_EQ_RANGES(ranges::view::values(constr.linear_terms()),
    //                  coefs);
    ASSERT_EQ(constr.sense(), rel);
    ASSERT_EQ(constr.rhs(), bound);
}

#endif  // ASSERT_HELPER_HPP