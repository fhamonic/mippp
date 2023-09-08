#ifndef CONSTRAINT_TEST_HELPER_HPP
#define CONSTRAINT_TEST_HELPER_HPP

#include <initializer_list>

#include "assert_eq_ranges.hpp"

template <typename Constr, typename V, typename C, typename L, typename U>
void ASSERT_CONSTRAINT(Constr && constr, std::initializer_list<V> vars,
                       std::initializer_list<C> coefs, L && lb, U && ub) {
    ASSERT_EQ_RANGES(constr.variables(), vars);
    ASSERT_EQ_RANGES(constr.coefficients(), coefs);
    ASSERT_EQ(constr.lower_bound(), lb);
    ASSERT_EQ(constr.upper_bound(), ub);
}

#endif //CONSTRAINT_TEST_HELPER_HPP