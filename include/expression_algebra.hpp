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

namespace Algebra {
    enum OptimizationSense { MIN=-1, MAX=1 };
    constexpr double INFTY = std::numeric_limits<double>::max();

    // TODO if retained inlude in LinearExpr body
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

    // inline LinearIneqConstraint operator<=(LinearExpr & e1, const LinearExpr & e2) {
    //     e1 - e2;
    //     return LinearIneqConstraint(LESS, std::move(e1));
    // }
    // inline LinearIneqConstraint operator>=(LinearExpr & e1, const LinearExpr & e2) {
    //     e1 - e2;
    //     return LinearIneqConstraint(GREATER, std::move(e1));
    // }

} //Algebra

#endif //EXPRESSION_ALGEBRA_HPP