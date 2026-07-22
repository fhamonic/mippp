#undef NDEBUG
#include <gtest/gtest.h>

#include <concepts>
#include <ranges>
#include <utility>
#include <vector>

#include "mippp/model_entities.hpp"
#include "mippp/quadratic_expression.hpp"

#include "assert_helper.hpp"

using namespace mippp;
using namespace mippp::operators;

using Var = model_variable<int, double>;

///////////////////////////////////////////////////////////////////////////////
///////////////// Products of linear expressions are quadratic ////////////////
///////////////////////////////////////////////////////////////////////////////

namespace {
// (13 - 3.2*x11) * (x7 + x5 + 2)
//   = -3.2*x11*x7 - 3.2*x11*x5 - 6.4*x11 + 13*x7 + 13*x5 + 26
auto base_product() { return (-Var(11) * 3.2 + 13) * (Var(7) + Var(5) + 2); }

// asserts that `e` is base_product() scaled by `k`, constant shifted by `c`
template <typename QE>
void assert_base_product(QE && e, double k, double c) {
    ASSERT_QUAD_EXPR(
        e, {{Var(11), Var(7), -3.2 * k}, {Var(11), Var(5), -3.2 * k}},
        {{Var(11), -6.4 * k}, {Var(7), 13.0 * k}, {Var(5), 13.0 * k}},
        26.0 * k + c);
}
}  // namespace

GTEST_TEST(quadratic_expression_operators, mul) {
    assert_base_product(base_product(), 1.0, 0.0);
}

GTEST_TEST(quadratic_expression_operators, square) {
    // (x2 - 3.2*x11 + 13)^2
    ASSERT_QUAD_EXPR(square(Var(2) - Var(11) * 3.2 + 13),
                     {{Var(2), Var(2), 1.0},
                      {Var(11), Var(11), 10.24},
                      {Var(2), Var(11), -6.4}},
                     {{Var(2), 26.0}, {Var(11), -83.2}}, 169.0);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////// Quadratic algebra ///////////////////////////////
///////////////////////////////////////////////////////////////////////////////

GTEST_TEST(quadratic_expression_operators, add_subtract_quadratic) {
    auto e1 = base_product();
    auto e2 = (Var(8) * 7.1 + 9) * (Var(3) + 1);
    ASSERT_QUAD_EXPR(e1 + e2,
                     {{Var(11), Var(7), -3.2},
                      {Var(11), Var(5), -3.2},
                      {Var(8), Var(3), 7.1}},
                     {{Var(11), -6.4},
                      {Var(8), 7.1},
                      {Var(7), 13.0},
                      {Var(5), 13.0},
                      {Var(3), 9.0}},
                     35.0);
    ASSERT_QUAD_EXPR(e1 - e2,
                     {{Var(11), Var(7), -3.2},
                      {Var(11), Var(5), -3.2},
                      {Var(8), Var(3), -7.1}},
                     {{Var(11), -6.4},
                      {Var(8), -7.1},
                      {Var(7), 13.0},
                      {Var(5), 13.0},
                      {Var(3), -9.0}},
                     17.0);
}

GTEST_TEST(quadratic_expression_operators, negate) {
    assert_base_product(-base_product(), -1.0, 0.0);
}

GTEST_TEST(quadratic_expression_operators, scalar_add_sub) {
    auto e = base_product();
    assert_base_product(e + 18.0, 1.0, 18.0);
    assert_base_product(18.0 + e, 1.0, 18.0);
    assert_base_product(e - (-18.0), 1.0, 18.0);
    assert_base_product(18.0 - (-e), 1.0, 18.0);
}

GTEST_TEST(quadratic_expression_operators, scalar_mul_div) {
    auto e = base_product();
    assert_base_product(e * 3.0, 3.0, 0.0);
    assert_base_product(3.0 * e, 3.0, 0.0);
    assert_base_product(e / (1.0 / 3.0), 3.0, 0.0);
}

GTEST_TEST(quadratic_expression_operators, add_subtract_linear) {
    auto e = base_product();
    auto le = 6.0 * Var(4);
    ASSERT_QUAD_EXPR(
        e + le, {{Var(11), Var(7), -3.2}, {Var(11), Var(5), -3.2}},
        {{Var(11), -6.4}, {Var(7), 13.0}, {Var(5), 13.0}, {Var(4), 6.0}}, 26.0);
    ASSERT_QUAD_EXPR(
        le + e, {{Var(11), Var(7), -3.2}, {Var(11), Var(5), -3.2}},
        {{Var(11), -6.4}, {Var(7), 13.0}, {Var(5), 13.0}, {Var(4), 6.0}}, 26.0);
    ASSERT_QUAD_EXPR(
        e - le, {{Var(11), Var(7), -3.2}, {Var(11), Var(5), -3.2}},
        {{Var(11), -6.4}, {Var(7), 13.0}, {Var(5), 13.0}, {Var(4), -6.0}},
        26.0);
    ASSERT_QUAD_EXPR(
        le - e, {{Var(11), Var(7), 3.2}, {Var(11), Var(5), 3.2}},
        {{Var(11), 6.4}, {Var(7), -13.0}, {Var(5), -13.0}, {Var(4), 6.0}},
        -26.0);
}

///////////////////////////////////////////////////////////////////////////////
////////////////////////// Products need a second pass ////////////////////////
///////////////////////////////////////////////////////////////////////////////

// A cartesian product walks one operand once per term of the other, so both
// operands must expose a restartable term range through `const &`. xsum() does
// not: its terms are a join of ranges materialized on the fly.

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

    // the concept sees through references and cv-qualification
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
namespace {
template <typename LE>
concept squarable = requires { typename linear_expression_square<LE>; };
template <typename LE1, typename LE2>
concept multipliable =
    requires { typename linear_expression_mul_view<LE1, LE2>; };
}  // namespace

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

GTEST_TEST(quadratic_expression_operators, square_of_materialized_xsum) {
    std::vector<Var> vars{Var(4), Var(7)};
    auto e = materialize(xsum(vars, [](auto && v) { return v * 2.0; }) + 3.0);
    // (2*x4 + 2*x7 + 3)^2
    ASSERT_QUAD_EXPR(
        square(e),
        {{Var(4), Var(4), 4.0}, {Var(7), Var(7), 4.0}, {Var(4), Var(7), 8.0}},
        {{Var(4), 12.0}, {Var(7), 12.0}}, 9.0);
}

GTEST_TEST(quadratic_expression_operators, product_of_materialized_xsums) {
    std::vector<Var> vars1{Var(1), Var(2)};
    std::vector<Var> vars2{Var(3)};
    auto e1 = materialize(xsum(vars1) + 1.0);
    auto e2 = materialize(xsum(vars2) - 2.0);
    // (x1 + x2 + 1) * (x3 - 2)
    ASSERT_QUAD_EXPR(e1 * e2, {{Var(1), Var(3), 1.0}, {Var(2), Var(3), 1.0}},
                     {{Var(1), -2.0}, {Var(2), -2.0}, {Var(3), 1.0}}, -2.0);
}

///////////////////////////////////////////////////////////////////////////////
/////////////////// `views::all`-style operand storage ////////////////////////
///////////////////////////////////////////////////////////////////////////////

// Products store their operands like range adaptors store ranges
// (`detail::expression_all_t`): a named operand is referenced through
// `detail::linear_expression_ref` -- no deep copy of its term container --
// while an rvalue operand is moved into the view, which then owns it.

GTEST_TEST(quadratic_expression_concepts, named_operands_are_referenced) {
    std::vector<Var> vars{Var(1), Var(2)};
    auto e = materialize(xsum(vars));
    using rt_expr = decltype(e);

    static_assert(
        std::same_as<decltype(square(e)),
                     linear_expression_square<
                         detail::linear_expression_ref<rt_expr>>>);
    static_assert(
        std::same_as<decltype(e * e),
                     linear_expression_mul_view<
                         detail::linear_expression_ref<rt_expr>,
                         detail::linear_expression_ref<rt_expr>>>);

    static_assert(std::same_as<decltype(square(materialize(xsum(vars)))),
                               linear_expression_square<rt_expr>>);
}

GTEST_TEST(quadratic_expression_operators, square_references_named_operand) {
    // referenced means *shared*: a later mutation of the operand is seen by
    // the view, exactly as through a `ref_view` of its terms
    std::vector<Var> vars{Var(1)};
    auto e = materialize(xsum(vars));
    auto quadexpr = square(e);
    e += 2.0 * Var(2);
    // (x1 + 2*x2)^2
    ASSERT_QUAD_TERMS(quadexpr.quadratic_terms(), {{Var(1), Var(1), 1.0},
                                                   {Var(2), Var(2), 4.0},
                                                   {Var(1), Var(2), 4.0}});
}

///////////////////////////////////////////////////////////////////////////////
/////////////////////// Statically-zero constant branches /////////////////////
///////////////////////////////////////////////////////////////////////////////

GTEST_TEST(quadratic_expression_operators, square_without_constant) {
    // a statically-zero constant drops the whole linear part
    ASSERT_QUAD_EXPR(square(Var(2) - Var(11) * 3.2),
                     {{Var(2), Var(2), 1.0},
                      {Var(11), Var(11), 10.24},
                      {Var(2), Var(11), -6.4}},
                     {}, 0.0);
}

GTEST_TEST(quadratic_expression_operators, mul_with_one_zero_constant) {
    std::vector<Var> vars{Var(1), Var(2)};
    auto e = materialize(xsum(vars));  // constant is statically zero
    // (x1 + x2) * (x3 + 5)
    ASSERT_QUAD_EXPR(e * (Var(3) + 5.0),
                     {{Var(1), Var(3), 1.0}, {Var(2), Var(3), 1.0}},
                     {{Var(1), 5.0}, {Var(2), 5.0}}, 0.0);
}

GTEST_TEST(quadratic_expression_operators, mul_with_both_zero_constants) {
    // both constants statically zero -> empty linear part
    ASSERT_QUAD_EXPR((Var(1) + Var(2)) * (Var(3) - Var(4)),
                     {{Var(1), Var(3), 1.0},
                      {Var(1), Var(4), -1.0},
                      {Var(2), Var(3), 1.0},
                      {Var(2), Var(4), -1.0}},
                     {}, 0.0);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////// Compatibility ///////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// `model_variable` bakes its scalar into its own type, so isolating the scalar
// half of the compatibility check needs an expression that reuses `Var` with a
// different coefficient type.
namespace {
struct float_coef_expression {
    constexpr auto linear_terms() const noexcept {
        return std::views::single(std::pair<Var, float>(Var(1), 1.0f));
    }
    constexpr zero_t constant() const noexcept { return {}; }
};
}  // namespace

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
//////////////////////////////// Evaluation ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// no type models both expression concepts, so the linear and quadratic
// `evaluate` overloads never compete
static_assert(!linear_expression<decltype(base_product())>);
static_assert(!quadratic_expression<Var>);

namespace {
auto base_product_values() {
    // x5 = 1, x7 = 3, x11 = 2
    std::vector<double> values(12, 0.0);
    values[5] = 1.0;
    values[7] = 3.0;
    values[11] = 2.0;
    return entity_mapping<Var, std::vector<double>>(std::move(values));
}
}  // namespace

GTEST_TEST(evaluate_quadratic_expression, product_of_linear_expressions) {
    auto values = base_product_values();
    // (13 - 3.2 * 2) * (3 + 1 + 2)
    ASSERT_NEAR(evaluate(base_product(), values), 39.6, 1e-12);
}

GTEST_TEST(evaluate_quadratic_expression, lvalue_expression_and_map) {
    const auto values = base_product_values();
    const auto e = base_product();
    ASSERT_NEAR(evaluate(e, values), 39.6, 1e-12);
}

GTEST_TEST(evaluate_quadratic_expression, square_sums_symmetric_terms) {
    auto values = base_product_values();
    // (x5 + x7 - 1)^2 = (1 + 3 - 1)^2 ; however the symmetric cross terms
    // are stored, evaluation must reproduce the square
    ASSERT_NEAR(evaluate(square(Var(5) + Var(7) - 1), values), 9.0, 1e-12);
}

GTEST_TEST(evaluate_quadratic_expression, pure_product_no_linear_part) {
    auto values = base_product_values();
    ASSERT_NEAR(evaluate(Var(5) * Var(7), values), 3.0, 1e-12);
    ASSERT_NEAR(evaluate(Var(11) * Var(11), values), 4.0, 1e-12);
}
