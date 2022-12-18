/**
 * @file solver_builder.hpp
 * @author François Hamonic (francois.hamonic@gmail.com)
 * @brief OSI_Builder class declaration
 * @version 0.1
 * @date 2021-08-4
 * 
 * @copyright Copyright (c) 2020
 */
#ifndef EXPRESSION_ALGEBRA_HPP
#define EXPRESSION_ALGEBRA_HPP

#include <numeric>
#include <vector>

#include <range/v3/view/zip.hpp>
#include <range/v3/algorithm/for_each.hpp>

#include "expressions/linear_expression.hpp"
#include "expressions/quadratic_expression.hpp"

#include "constraints/linear_constraints.hpp"
#include "constraints/quadratic_constraints.hpp"

#include "expressions_algebra/linear_expression_algebra.hpp"

namespace Algebra {
    enum OptimizationSense { MIN=-1, MAX=1 };
    constexpr double infinity = std::numeric_limits<double>::max();

    // LINEAR
    struct QuadraticTerm {
        int var1;
        int var2;
        double coef;
        constexpr QuadraticTerm(Var v1, Var v2, double c)
            : var1(v1.get())
            , var2(v2.get())
            , coef(c) {}
        constexpr QuadraticTerm& operator*(double c) {
            coef += c;
            return *this;
        }
    };

    constexpr QuadraticTerm operator*(LinearTerm t, Var v) {
        return QuadraticTerm(t.var, v, t.coef);
    }

    inline QuadraticExpr& operator+(QuadraticExpr & e, double c) {
        return e.add(c);
    }
    inline QuadraticExpr& operator+(QuadraticExpr & e, LinearTerm t) {
        return e.add(t.var, t.coef);
    }
    inline QuadraticExpr& operator+(QuadraticExpr & e, QuadraticTerm t) {
        return e.add(t.var1, t.var2, t.coef);
    }

    inline QuadraticExpr operator+(QuadraticTerm t1, QuadraticTerm t2) {
        return std::forward<QuadraticExpr&&>(QuadraticExpr(t1) + t2);
    }
    inline QuadraticExpr operator+(LinearExpr&& le, QuadraticTerm t) {
        QuadraticExpr e(std::move(le));
        e.add(t);
        return e;
    }

} //Algebra

#endif //EXPRESSION_ALGEBRA_HPP