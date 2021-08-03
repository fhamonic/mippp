#ifndef QUADRATIC_EXPRESSION_HPP
#define QUADRATIC_EXPRESSION_HPP

#include <vector>

#include <range/v3/all.hpp>

#include "expressions/linear_expression.hpp"

struct QuadraticTerm;
struct QuadraticExpr;

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

struct QuadraticExpr {
    LinearExpr linear_expr;
    std::vector<int> vars1;
    std::vector<int> vars2;
    std::vector<double> coefs;
    QuadraticExpr(LinearExpr&& e)
        : linear_expr(std::move(e)) {}
    QuadraticExpr(QuadraticTerm t)
        : vars1(1, t.var1)
        , vars2(1, t.var2)
        , coefs(1, t.coef) {}
    QuadraticExpr& operator+(double c) {
        linear_expr + c;
        return *this;
    }
    QuadraticExpr& operator+(LinearTerm t) {
        linear_expr + t;
        return *this;
    }
    QuadraticExpr& operator+(QuadraticTerm t) {
        vars1.emplace_back(t.var1);
        vars2.emplace_back(t.var2);
        coefs.emplace_back(t.coef);
        return *this;
    }
};

inline QuadraticExpr operator+(QuadraticTerm t1, QuadraticTerm t2) {
    return std::forward<QuadraticExpr&&>(QuadraticExpr(t1) + t2);
}
inline QuadraticExpr operator+(LinearExpr&& t, QuadraticTerm v) {
    return QuadraticExpr(std::move(t)) + v;
}

#endif //QUADRATIC_EXPRESSION_HPP