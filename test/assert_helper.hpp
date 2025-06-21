#ifndef ASSERT_HELPER_HPP
#define ASSERT_HELPER_HPP

#include <initializer_list>
#include <ranges>

#include <range/v3/view/map.hpp>
#include <range/v3/view/zip.hpp>

#include "mippp/linear_expression.hpp"
#include "mippp/quadratic_expression.hpp"

template <typename R1, typename R2>
void ASSERT_EQ_RANGES(R1 && r1, R2 && r2) {
    ASSERT_EQ(std::ranges::distance(r1), std::ranges::distance(r2));
    for(const auto & [e1, e2] : ranges::views::zip(r1, r2)) {
        ASSERT_EQ(e1, e2);
    }
}

template <typename Terms>
    requires fhamonic::mippp::linear_term<std::ranges::range_value_t<Terms>>
void ASSERT_LIN_TERMS(
    Terms && terms,
    std::initializer_list<std::ranges::range_value_t<Terms>> expected_terms) {
    using term = std::ranges::range_value_t<Terms>;
    using variable = fhamonic::mippp::linear_term_variable_t<term>;
    using scalar = fhamonic::mippp::linear_term_scalar_t<term>;
    std::map<variable, scalar> factorized_terms;
    for(auto && [var, coef] : terms) factorized_terms[var] += coef;
    ASSERT_EQ(factorized_terms.size(), expected_terms.size());
    for(auto && [var, coef] : terms)
        ASSERT_NEAR(factorized_terms[var], coef, 1e-7);
}

template <typename Expr>
void ASSERT_LIN_EXPR(
    Expr && expr,
    std::initializer_list<fhamonic::mippp::linear_term_t<Expr>> expected_terms,
    fhamonic::mippp::linear_expression_scalar_t<Expr> expected_constant) {
    ASSERT_LIN_TERMS(expr.linear_terms(), expected_terms);
    ASSERT_EQ(expr.constant(), expected_constant);
}

struct variables_pair_cmp {
    bool operator()(auto && p1, auto && p2) const {
        auto [p1v1, p1v2] = p1;
        if(p1v2 < p1v1) std::swap(p1v1, p1v2);
        auto [p2v1, p2v2] = p2;
        if(p2v2 < p2v1) std::swap(p2v1, p2v2);
        if(p1v1 == p2v1) return p1v2 < p2v2;
        return p1v1 < p2v1;
    }
};

template <typename Terms>
    requires fhamonic::mippp::quadratic_term<std::ranges::range_value_t<Terms>>
void ASSERT_QUAD_TERMS(
    Terms && terms,
    std::initializer_list<std::ranges::range_value_t<Terms>> expected_terms) {
    using term = std::ranges::range_value_t<Terms>;
    using variable = fhamonic::mippp::quadratic_term_variable_t<term>;
    using scalar = fhamonic::mippp::quadratic_term_scalar_t<term>;
    std::map<std::pair<variable, variable>, scalar, variables_pair_cmp>
        factorized_terms;

    for(auto && [var1, var2, coef] : terms)
        factorized_terms[std::make_pair(var1, var2)] += coef;
    ASSERT_EQ(factorized_terms.size(), expected_terms.size());
    for(auto && [var1, var2, coef] : terms)
        ASSERT_NEAR(factorized_terms[std::make_pair(var1, var2)], coef, 1e-7);
}

#include "range/v3/core.hpp"

template <typename Constr>
void ASSERT_CONSTRAINT(
    Constr && constr,
    std::initializer_list<fhamonic::mippp::linear_term_t<Constr>>
        expected_terms,
    fhamonic::mippp::constraint_sense rel,
    fhamonic::mippp::linear_constraint_scalar_t<Constr> bound) {
    ASSERT_LIN_TERMS(constr.linear_terms(), expected_terms);
    ASSERT_EQ(constr.sense(), rel);
    ASSERT_EQ(constr.rhs(), bound);
}

#endif  // ASSERT_HELPER_HPP