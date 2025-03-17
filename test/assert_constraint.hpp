#ifndef CONSTRAINT_TEST_HELPER_HPP
#define CONSTRAINT_TEST_HELPER_HPP

#include <initializer_list>
#include <ranges>

#include "range/v3/core.hpp"

#include "assert_eq_ranges.hpp"

template <typename Constr, typename V, typename C, typename L, typename U>
void ASSERT_CONSTRAINT(Constr && constr, std::initializer_list<V> vars,
                       std::initializer_list<C> coefs, L && lb, U && ub) {
    // ASSERT_EQ_RANGES(std::ranges::views::keys(constr.expression().linear_terms()),
    //                  ranges::views::zip(coefs, vars));
    // ASSERT_EQ_RANGES(std::ranges::views::values(constr.expression().linear_terms()),
    //                  ranges::views::zip(coefs, vars));
    ASSERT_EQ_RANGES(constr.expression().linear_terms(),
                     ranges::views::zip(vars, coefs));
    ASSERT_EQ(linear_constraint_lower_bound(constr), lb);
    ASSERT_EQ(linear_constraint_upper_bound(constr), ub);
}

#endif  // CONSTRAINT_TEST_HELPER_HPP