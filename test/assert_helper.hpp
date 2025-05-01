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
        if constexpr(std::floating_point<T>) {
            ASSERT_NEAR(e1, e2, 1e-13);
        } else {
            ASSERT_EQ(e1, e2);
        }
    }
}

template <typename Terms, typename V, typename C>
void ASSERT_LIN_TERMS(Terms && terms, std::initializer_list<V> vars,
                      std::initializer_list<C> coefs) {
    ASSERT_EQ_RANGES(ranges::view::keys(terms), vars);
    ASSERT_EQ_RANGES(ranges::view::values(terms), coefs);
}

template <typename Expr, typename V, typename C, typename S>
void ASSERT_LIN_EXPR(Expr && expr, std::initializer_list<V> vars,
                     std::initializer_list<C> coefs, S && scalar) {
    ASSERT_LIN_TERMS(expr.linear_terms(), vars, coefs);
    ASSERT_EQ(expr.constant(), scalar);
}

template <typename Terms, typename V1, typename V2, typename C>
void ASSERT_BILIN_TERMS(Terms && terms, std::initializer_list<V1> vars1,
                        std::initializer_list<V2> vars2,
                        std::initializer_list<C> coefs) {
    ASSERT_EQ_RANGES(ranges::view::transform(
                         terms, [](const auto & t) { return std::get<0>(t); }),
                     vars1);
    ASSERT_EQ_RANGES(ranges::view::transform(
                         terms, [](const auto & t) { return std::get<1>(t); }),
                     vars2);
    ASSERT_EQ_RANGES(ranges::view::transform(
                         terms, [](const auto & t) { return std::get<2>(t); }),
                     coefs);
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