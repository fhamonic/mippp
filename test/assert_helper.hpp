#pragma once

#include <initializer_list>
#include <map>
#include <ranges>

#include "mippp/linear_expression.hpp"
#include "mippp/quadratic_expression.hpp"

template <typename R1, typename R2>
void ASSERT_EQ_RANGES(R1 && r1, R2 && r2) {
    ASSERT_EQ(std::ranges::distance(r1), std::ranges::distance(r2));
    for(const auto & [e1, e2] : std::views::zip(r1, r2)) {
        ASSERT_EQ(e1, e2);
    }
}

template <typename Terms>
    requires mippp::linear_term<std::ranges::range_value_t<Terms>>
void ASSERT_LIN_TERMS(
    Terms && terms,
    std::initializer_list<std::ranges::range_value_t<Terms>> expected_terms) {
    using term = std::ranges::range_value_t<Terms>;
    using variable = mippp::linear_term_variable_t<term>;
    using scalar = mippp::linear_term_scalar_t<term>;
    std::map<variable, scalar> factorized_terms;
    for(auto && [var, coef] : terms) factorized_terms[var] += coef;
    ASSERT_EQ(factorized_terms.size(), expected_terms.size());
    for(auto && [var, coef] : expected_terms)
        ASSERT_NEAR(factorized_terms.at(var), coef, 1e-7);
}

template <typename Expr>
void ASSERT_LIN_EXPR(
    Expr && expr,
    std::initializer_list<mippp::linear_term_t<Expr>> expected_terms,
    mippp::linear_expression_scalar_t<Expr> expected_constant) {
    ASSERT_LIN_TERMS(expr.linear_terms(), expected_terms);
    ASSERT_EQ(expr.constant(), expected_constant);
}

// Compares ids rather than variables: with `using namespace operators` in
// the including test, MSVC resolves `v1 < v2` to the constraint-building
// operator< and then rejects the linear_constraint_view in the `if`.
struct variables_pair_cmp {
    bool operator()(auto && p1, auto && p2) const {
        auto [p1v1, p1v2] = p1;
        auto id1 = p1v1.id(), id2 = p1v2.id();
        if(id2 < id1) std::swap(id1, id2);
        auto [p2v1, p2v2] = p2;
        auto id3 = p2v1.id(), id4 = p2v2.id();
        if(id4 < id3) std::swap(id3, id4);
        if(id1 == id3) return id2 < id4;
        return id1 < id3;
    }
};

template <typename Terms>
    requires mippp::quadratic_term<std::ranges::range_value_t<Terms>>
void ASSERT_QUAD_TERMS(
    Terms && terms,
    std::initializer_list<std::ranges::range_value_t<Terms>> expected_terms) {
    using term = std::ranges::range_value_t<Terms>;
    using variable = mippp::quadratic_term_variable_t<term>;
    using scalar = mippp::quadratic_term_scalar_t<term>;
    std::map<std::pair<variable, variable>, scalar, variables_pair_cmp>
        factorized_terms;
    for(auto && [var1, var2, coef] : terms)
        factorized_terms[std::make_pair(var1, var2)] += coef;
    ASSERT_EQ(factorized_terms.size(), expected_terms.size());
    for(auto && [var1, var2, coef] : expected_terms)
        ASSERT_NEAR(factorized_terms.at(std::make_pair(var1, var2)), coef,
                    1e-7);
}

template <typename QExpr>
void ASSERT_QUAD_EXPR(
    QExpr && expr,
    std::initializer_list<mippp::quadratic_term_t<QExpr>> expected_quad_terms,
    std::initializer_list<mippp::quadratic_expression_linear_term_t<QExpr>>
        expected_linear_terms,
    mippp::quadratic_expression_scalar_t<QExpr> expected_constant) {
    ASSERT_QUAD_TERMS(expr.quadratic_terms(), expected_quad_terms);
    ASSERT_LIN_EXPR(expr.linear_part(), expected_linear_terms,
                    expected_constant);
}

template <typename Constr>
void ASSERT_CONSTRAINT(
    Constr && constr,
    std::initializer_list<mippp::linear_term_t<Constr>> expected_terms,
    mippp::constraint_sense rel,
    mippp::linear_constraint_scalar_t<Constr> bound) {
    ASSERT_LIN_TERMS(constr.linear_terms(), expected_terms);
    ASSERT_EQ(constr.sense(), rel);
    ASSERT_EQ(constr.rhs(), bound);
}
