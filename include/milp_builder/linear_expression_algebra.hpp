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

#include "milp_builder/strong_types.hpp"
#include "milp_builder/linear_expression.hpp"
#include "milp_builder/linear_constraints.hpp"

struct LinearTerm {
    double coef;
    int var;

    constexpr LinearTerm(LinearTerm&& t)
        : coef(t.coef)
        , var(t.var) { /*std::cout << "move ctor LinearTerm" << std::endl;*/ }
    constexpr LinearTerm(const LinearTerm & t)
        : coef(t.coef)
        , var(t.var) { /*std::cout << "copy ctor LinearTerm" << std::endl;*/ }

    constexpr LinearTerm(Var v, double c=1.0)
        : coef(c)
        , var(v.id()) { /*std::cout << "custom ctor LinearTerm" << std::endl;*/ }
    
    constexpr LinearTerm& operator*(double c) {
        coef *= c;
        return *this;
    }
};

constexpr LinearTerm operator*(Var v, double c) {
    LinearTerm t(v, c);
    return t;
}
constexpr LinearTerm operator*(double c, Var v) {
    LinearTerm t(v, c);
    return t;
}

inline LinearExpr& operator+(LinearExpr & e, double c) { 
    return e.add(c);
}
inline LinearExpr& operator+(LinearExpr&& e, double c) { 
    return e.add(c);
}
inline LinearExpr& operator+(LinearExpr & e, const LinearTerm & t) { 
    return e.add(t.var, t.coef);
}
inline LinearExpr& operator+(LinearExpr&& e, const LinearTerm & t) { 
    return e.add(t.var, t.coef);
}

inline LinearExpr operator+(const LinearTerm & t1, const LinearTerm & t2) {
    LinearExpr e(t1.var, t1.coef);
    e + t2;
    return e;
}
inline LinearExpr operator+(const LinearTerm & t, double c) {
    LinearExpr e(t.var, t.coef);
    e.add(c);
    return e;
}
inline LinearExpr operator+(double c, const LinearTerm & t) {
    LinearExpr e(t.var, t.coef);
    e.add(c);
    return e;
}

inline LinearExpr & operator-(LinearExpr & e1, const LinearExpr & e2) {
    e1.constant -= e2.constant;
    ranges::for_each(ranges::view::zip(e2.vars, e2.coefs), 
        [&e1](const auto p){ e1.add(p.first, -p.second); });
    return e1;
}

inline LinearIneqConstraint operator<=(LinearExpr && e1, const LinearExpr & e2) {
    e1 - e2;
    return LinearIneqConstraint(LESS, std::move(e1));
}
inline LinearIneqConstraint operator>=(LinearExpr && e1, const LinearExpr & e2) {
    e1 - e2;
    return LinearIneqConstraint(GREATER, std::move(e1));
}
inline LinearIneqConstraint operator<=(LinearExpr & e1, const LinearExpr & e2) {
    return std::move(e1) <= e2;
}
inline LinearIneqConstraint operator>=(LinearExpr & e1, const LinearExpr & e2) {
    return std::move(e1) >= e2;
}

#endif //EXPRESSION_ALGEBRA_HPP