#undef NDEBUG
#include <gtest/gtest.h>

#include <vector>

#include "mippp/model_entities.hpp"
#include "mippp/quadratic_expression.hpp"

#include "assert_helper.hpp"

using namespace mippp;
using namespace mippp::operators;

using Var = model_variable<int, double>;
using FloatVar = model_variable<int, float>;

GTEST_TEST(linear_expression_operators, square) {
    auto quadexpr = square(Var(2) - Var(11) * 3.2 + 13);
    ASSERT_QUAD_TERMS(quadexpr.quadratic_terms(), {{Var(2), Var(2), 1.0},
                                                   {Var(11), Var(11), 10.24},
                                                   {Var(2), Var(11), -6.4}});
    ASSERT_LIN_EXPR(quadexpr.linear_part(), {{Var(2), 26.0}, {Var(11), -83.2}},
                    169);
}

GTEST_TEST(linear_expression_operators, mul_terms) {
    auto quadexpr = (-Var(11) * 3.2 + 13) * (Var(7) + Var(5) + 2);
    ASSERT_QUAD_TERMS(quadexpr.quadratic_terms(),
                      {{Var(11), Var(7), -3.2}, {Var(11), Var(5), -3.2}});
    ASSERT_LIN_EXPR(quadexpr.linear_part(),
                    {{Var(11), -6.4}, {Var(7), 13.0}, {Var(5), 13.0}}, 26);
}

GTEST_TEST(quadratic_expression_operators, add_terms) {
    auto e1 = (-Var(11) * 3.2 + 13) * (Var(7) + Var(5) + 2);
    auto e2 = (Var(8) * 7.1 + 9) * (Var(3) + 1);
    auto quadexpr = e1 + e2;
    ASSERT_QUAD_TERMS(quadexpr.quadratic_terms(), {{Var(11), Var(7), -3.2},
                                                   {Var(11), Var(5), -3.2},
                                                   {Var(8), Var(3), 7.1}});
    ASSERT_LIN_EXPR(quadexpr.linear_part(),
                    {{Var(11), -6.4},
                     {Var(8), 7.1},
                     {Var(7), 13.0},
                     {Var(5), 13.0},
                     {Var(3), 9.0}},
                    35);
}

GTEST_TEST(quadratic_expression_operators, negate) {
    auto e = (-Var(11) * 3.2 + 13) * (Var(7) + Var(5) + 2);
    auto quadexpr = -e;
    ASSERT_QUAD_TERMS(quadexpr.quadratic_terms(),
                      {{Var(11), Var(7), 3.2}, {Var(11), Var(5), 3.2}});
    ASSERT_LIN_EXPR(quadexpr.linear_part(),
                    {{Var(11), 6.4}, {Var(7), -13.0}, {Var(5), -13.0}}, -26.0);
}

GTEST_TEST(quadratic_expression_operators, scalar_add) {
    auto e = (-Var(11) * 3.2 + 13) * (Var(7) + Var(5) + 2);
    auto quadexpr = e + 18.0;
    ASSERT_QUAD_TERMS(quadexpr.quadratic_terms(),
                      {{Var(11), Var(7), -3.2}, {Var(11), Var(5), -3.2}});
    ASSERT_LIN_EXPR(quadexpr.linear_part(),
                    {{Var(11), -6.4}, {Var(7), 13.0}, {Var(5), 13.0}}, 44.0);
}
GTEST_TEST(quadratic_expression_operators, scalar_add2) {
    auto e = (-Var(11) * 3.2 + 13) * (Var(7) + Var(5) + 2);
    auto quadexpr = 18.0 + e;
    ASSERT_QUAD_TERMS(quadexpr.quadratic_terms(),
                      {{Var(11), Var(7), -3.2}, {Var(11), Var(5), -3.2}});
    ASSERT_LIN_EXPR(quadexpr.linear_part(),
                    {{Var(11), -6.4}, {Var(7), 13.0}, {Var(5), 13.0}}, 44.0);
}

GTEST_TEST(quadratic_expression_operators, scalar_sub) {
    auto e = (-Var(11) * 3.2 + 13) * (Var(7) + Var(5) + 2);
    auto quadexpr = e - (-18.0);
    ASSERT_QUAD_TERMS(quadexpr.quadratic_terms(),
                      {{Var(11), Var(7), -3.2}, {Var(11), Var(5), -3.2}});
    ASSERT_LIN_EXPR(quadexpr.linear_part(),
                    {{Var(11), -6.4}, {Var(7), 13.0}, {Var(5), 13.0}}, 44.0);
}
GTEST_TEST(quadratic_expression_operators, scalar_sub2) {
    auto e = (-Var(11) * 3.2 + 13) * (Var(7) + Var(5) + 2);
    auto quadexpr = 18.0 - (-e);
    ASSERT_QUAD_TERMS(quadexpr.quadratic_terms(),
                      {{Var(11), Var(7), -3.2}, {Var(11), Var(5), -3.2}});
    ASSERT_LIN_EXPR(quadexpr.linear_part(),
                    {{Var(11), -6.4}, {Var(7), 13.0}, {Var(5), 13.0}}, 44.0);
}

GTEST_TEST(quadratic_expression_operators, scalar_mul) {
    auto e = (-Var(11) * 3.2 + 13) * (Var(7) + Var(5) + 2);
    auto quadexpr = e * 3.0;
    ASSERT_QUAD_TERMS(quadexpr.quadratic_terms(),
                      {{Var(11), Var(7), -9.6}, {Var(11), Var(5), -9.6}});
    ASSERT_LIN_EXPR(quadexpr.linear_part(),
                    {{Var(11), -19.2}, {Var(7), 39.0}, {Var(5), 39.0}}, 78.0);
}
GTEST_TEST(quadratic_expression_operators, scalar_mul2) {
    auto e = (-Var(11) * 3.2 + 13) * (Var(7) + Var(5) + 2);
    auto quadexpr = 3.0 * e;
    ASSERT_QUAD_TERMS(quadexpr.quadratic_terms(),
                      {{Var(11), Var(7), -9.6}, {Var(11), Var(5), -9.6}});
    ASSERT_LIN_EXPR(quadexpr.linear_part(),
                    {{Var(11), -19.2}, {Var(7), 39.0}, {Var(5), 39.0}}, 78.0);
}

GTEST_TEST(quadratic_expression_operators, scalar_div) {
    auto e = (-Var(11) * 3.2 + 13) * (Var(7) + Var(5) + 2);
    auto quadexpr = e / (1.0 / 3.0);
    ASSERT_QUAD_TERMS(quadexpr.quadratic_terms(),
                      {{Var(11), Var(7), -9.6}, {Var(11), Var(5), -9.6}});
    ASSERT_LIN_EXPR(quadexpr.linear_part(),
                    {{Var(11), -19.2}, {Var(7), 39.0}, {Var(5), 39.0}}, 78.0);
}

GTEST_TEST(quadratic_expression_operators, add_linear_expr) {
    auto e = (-Var(11) * 3.2 + 13) * (3 * Var(7) + 2);
    auto quadexpr = e + 6.0 * Var(4);
    ASSERT_QUAD_TERMS(quadexpr.quadratic_terms(), {{Var(11), Var(7), -9.6}});
    ASSERT_LIN_EXPR(quadexpr.linear_part(),
                    {{Var(11), -6.4}, {Var(7), 39.0}, {Var(4), 6.0}}, 26.0);
}
GTEST_TEST(quadratic_expression_operators, add_linear_expr2) {
    auto e = (-Var(11) * 3.2 + 13) * (3 * Var(7) + 2);
    auto quadexpr = 6.0 * Var(4) + e;
    ASSERT_QUAD_TERMS(quadexpr.quadratic_terms(), {{Var(11), Var(7), -9.6}});
    ASSERT_LIN_EXPR(quadexpr.linear_part(),
                    {{Var(11), -6.4}, {Var(7), 39.0}, {Var(4), 6.0}}, 26.0);
}
GTEST_TEST(quadratic_expression_operators, sub_linear_expr) {
    auto e = (-Var(11) * 3.2 + 13) * (3 * Var(7) + 2);
    auto quadexpr = e - 6.0 * Var(4);
    ASSERT_QUAD_TERMS(quadexpr.quadratic_terms(), {{Var(11), Var(7), -9.6}});
    ASSERT_LIN_EXPR(quadexpr.linear_part(),
                    {{Var(11), -6.4}, {Var(7), 39.0}, {Var(4), -6.0}}, 26.0);
}
GTEST_TEST(quadratic_expression_operators, sub_linear_expr2) {
    auto e = (-Var(11) * 3.2 + 13) * (3 * Var(7) + 2);
    auto quadexpr = 6.0 * Var(4) - e;
    ASSERT_QUAD_TERMS(quadexpr.quadratic_terms(), {{Var(11), Var(7), 9.6}});
    ASSERT_LIN_EXPR(quadexpr.linear_part(),
                    {{Var(11), 6.4}, {Var(7), -39.0}, {Var(4), 6.0}}, -26.0);
}
///////////////////////////////////////////////////////////////////////////////
////////////////////////// Multipliability constraint /////////////////////////
///////////////////////////////////////////////////////////////////////////////

// Products traverse one operand once per term of the other, so both operands
// must expose a restartable term range through `const &`. xsum() does not: its
// terms are a join of ranges materialized on the fly.

GTEST_TEST(quadratic_expression_concepts, multipass_linear_terms) {
    std::vector<Var> vars{Var(1), Var(2)};

    // a bare variable and the eager expression views are forward ranges
    static_assert(multipass_linear_terms<Var>);
    static_assert(multipass_linear_terms<decltype(Var(1) + 2.0)>);
    static_assert(multipass_linear_terms<decltype(Var(1) - Var(2) * 3.0)>);

    // xsum() is single-pass
    static_assert(!multipass_linear_terms<decltype(xsum(vars))>);
    static_assert(!multipass_linear_terms<decltype(xsum(
                      vars, [](auto && v) { return v * 2.0; }))>);

    // materialize() restores a restartable range
    static_assert(multipass_linear_terms<decltype(materialize(xsum(vars)))>);

    // the concept must see through references and cv-qualification
    static_assert(multipass_linear_terms<const Var &>);
    static_assert(!multipass_linear_terms<decltype(xsum(vars)) &>);
}

GTEST_TEST(quadratic_expression_concepts, const_readable_linear_terms) {
    std::vector<Var> vars{Var(1), Var(2)};

    static_assert(const_readable_linear_terms<Var>);
    // built over a named range: the terms view is copyable, so const-readable
    // (but still single-pass, hence not multipliable)
    static_assert(const_readable_linear_terms<decltype(xsum(vars))>);
    // built over an rvalue range: the terms view owns it and is move-only
    static_assert(
        !const_readable_linear_terms<decltype(xsum(std::vector<Var>{Var(1)}))>);
}

// An unsatisfied constraint must reject the view type itself, not just fire a
// static_assert at the operator -- otherwise `std::is_constructible_v` & co
// lie.
template <typename LE>
concept squarable = requires { typename linear_expression_square<LE>; };
template <typename LE1, typename LE2>
concept multipliable =
    requires { typename linear_expression_mul_view<LE1, LE2>; };

GTEST_TEST(quadratic_expression_concepts, views_reject_single_pass_operands) {
    std::vector<Var> vars{Var(1), Var(2)};
    using single_pass = decltype(xsum(vars));
    using restartable = decltype(materialize(xsum(vars)));

    static_assert(!squarable<single_pass>);
    static_assert(squarable<restartable>);

    static_assert(!multipliable<single_pass, restartable>);
    static_assert(!multipliable<restartable, single_pass>);
    static_assert(multipliable<restartable, restartable>);
}

// `model_variable` bakes its scalar into its own type, so isolating the scalar
// half of the compatibility check needs an expression that reuses `Var` with a
// different coefficient type.
struct float_coef_expression {
    constexpr auto linear_terms() const noexcept {
        return std::views::single(std::pair<Var, float>(Var(1), 1.0f));
    }
    constexpr zero_t constant() const noexcept { return {}; }
};

GTEST_TEST(quadratic_expression_concepts, compatible_quadratic_expressions) {
    using quad = decltype(square(Var(1) + 2.0));
    // same variable type, different scalar type
    using quad_float_coef = decltype(square(float_coef_expression{}));
    // same scalar type, different variable type
    using quad_long_id =
        decltype(square(model_variable<long, double>(1) + 2.0));

    static_assert(
        std::same_as<quadratic_expression_variable_t<quad>,
                     quadratic_expression_variable_t<quad_float_coef>>);
    static_assert(std::same_as<quadratic_expression_scalar_t<quad>,
                               quadratic_expression_scalar_t<quad_long_id>>);

    static_assert(compatible_quadratic_expressions<quad, quad>);
    static_assert(!compatible_quadratic_expressions<quad, quad_float_coef>);
    static_assert(!compatible_quadratic_expressions<quad, quad_long_id>);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////// Materialize as escape hatch /////////////////////////
///////////////////////////////////////////////////////////////////////////////

GTEST_TEST(quadratic_expression_operators, square_of_materialized_xsum) {
    std::vector<Var> vars{Var(4), Var(7)};
    auto e = materialize(xsum(vars, [](auto && v) { return v * 2.0; }) + 3.0);
    auto quadexpr = square(e);
    // (2*x4 + 2*x7 + 3)^2
    ASSERT_QUAD_TERMS(
        quadexpr.quadratic_terms(),
        {{Var(4), Var(4), 4.0}, {Var(7), Var(7), 4.0}, {Var(4), Var(7), 8.0}});
    ASSERT_LIN_EXPR(quadexpr.linear_part(), {{Var(4), 12.0}, {Var(7), 12.0}},
                    9.0);
}

GTEST_TEST(quadratic_expression_operators, product_of_materialized_xsums) {
    std::vector<Var> vars1{Var(1), Var(2)};
    std::vector<Var> vars2{Var(3)};
    auto e1 = materialize(xsum(vars1) + 1.0);
    auto e2 = materialize(xsum(vars2) - 2.0);
    auto quadexpr = e1 * e2;
    // (x1 + x2 + 1) * (x3 - 2)
    ASSERT_QUAD_TERMS(quadexpr.quadratic_terms(),
                      {{Var(1), Var(3), 1.0}, {Var(2), Var(3), 1.0}});
    ASSERT_LIN_EXPR(quadexpr.linear_part(),
                    {{Var(1), -2.0}, {Var(2), -2.0}, {Var(3), 1.0}}, -2.0);
}

///////////////////////////////////////////////////////////////////////////////
/////////////////////// Statically-zero constant branches /////////////////////
///////////////////////////////////////////////////////////////////////////////

GTEST_TEST(linear_expression_operators, square_without_constant) {
    auto quadexpr = square(Var(2) - Var(11) * 3.2);
    ASSERT_QUAD_TERMS(quadexpr.quadratic_terms(), {{Var(2), Var(2), 1.0},
                                                   {Var(11), Var(11), 10.24},
                                                   {Var(2), Var(11), -6.4}});
    // a statically-zero constant drops the whole linear part
    ASSERT_LIN_EXPR(quadexpr.linear_part(), {}, 0.0);
}

GTEST_TEST(linear_expression_operators, mul_with_one_zero_constant) {
    std::vector<Var> vars{Var(1), Var(2)};
    auto e = materialize(xsum(vars));  // constant is statically zero
    auto quadexpr = e * (Var(3) + 5.0);
    // (x1 + x2) * (x3 + 5)
    ASSERT_QUAD_TERMS(quadexpr.quadratic_terms(),
                      {{Var(1), Var(3), 1.0}, {Var(2), Var(3), 1.0}});
    ASSERT_LIN_EXPR(quadexpr.linear_part(), {{Var(1), 5.0}, {Var(2), 5.0}},
                    0.0);
}

GTEST_TEST(linear_expression_operators, mul_with_both_zero_constants) {
    auto quadexpr = (Var(1) + Var(2)) * (Var(3) - Var(4));
    ASSERT_QUAD_TERMS(quadexpr.quadratic_terms(), {{Var(1), Var(3), 1.0},
                                                   {Var(1), Var(4), -1.0},
                                                   {Var(2), Var(3), 1.0},
                                                   {Var(2), Var(4), -1.0}});
    // both constants statically zero -> empty linear part
    ASSERT_LIN_EXPR(quadexpr.linear_part(), {}, 0.0);
}
