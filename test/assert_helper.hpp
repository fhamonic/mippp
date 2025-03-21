#ifndef ASSERT_HELPER_HPP
#define ASSERT_HELPER_HPP

#include <initializer_list>
#include <ranges>

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
    ASSERT_EQ(std::ranges::size(r1), l.size());
    for(const auto & [e1, e2] : ranges::views::zip(r1, l)) {
        ASSERT_EQ(e1, e2);
    }
}

template <typename Expr, typename V, typename C, typename S>
void ASSERT_EXPRESSION(Expr && expr, std::initializer_list<V> vars,
                       std::initializer_list<C> coefs, S && scalar) {
    ASSERT_EQ_RANGES(expr.linear_terms(), ranges::views::zip(vars, coefs));
    ASSERT_EQ(expr.constant(), scalar);
}

#include "range/v3/core.hpp"

template <typename Constr, typename V, typename C, typename L, typename U>
void ASSERT_CONSTRAINT(Constr && constr, std::initializer_list<V> vars,
                       std::initializer_list<C> coefs, L && lb, U && ub) {
    ASSERT_EQ_RANGES(constr.expression().linear_terms(),
                     ranges::views::zip(vars, coefs));
    ASSERT_EQ(linear_constraint_lower_bound(constr), lb);
    ASSERT_EQ(linear_constraint_upper_bound(constr), ub);
}

#endif  // ASSERT_HELPER_HPP