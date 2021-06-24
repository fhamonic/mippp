/**
 * @file print_solver_builder.hpp
 * @author François Hamonic (francois.hamonic@gmail.com)
 * @brief ostream << operators for SolverBuilder_Utils classes
 * @version 0.1
 * @date 2020-10-27
 * 
 * @copyright Copyright (c) 2020
 */
#ifndef PRINT_SOLVER_BUILDER_HPP
#define PRINT_SOLVER_BUILDER_HPP

#include <ostream>
#include <range/v3/all.hpp>

#include "solver_builder.hpp"

namespace SolverBuilder_Utils {
    inline std::ostream& operator<<(std::ostream& os, const LinearExpression & linear_expr) {
        ranges::for_each(ranges::view::zip(linear_expr.getCoefficients(), linear_expr.getIndices()), 
            [&os](const auto p){ os << p.first << "*x" << p.second << " + "; });
        return os << linear_expr.getConstant();
    }
    inline std::ostream& operator<<(std::ostream& os, const QuadraticExpression & quad_expr) {
        ranges::for_each(ranges::view::zip(quad_expr.getQuadCoefficients(), quad_expr.getQuadIndices1(), quad_expr.getQuadIndices2()), 
            [&os](const auto p){ os << std::get<0>(p) << "*x" << std::get<1>(p) << "x" << std::get<2>(p) << " + "; });
        return os << quad_expr.getLineraExpression();
    }
    inline std::ostream& operator<<(std::ostream& os, linear_ineq_constraint constraint) {
        return os << constraint.linear_expression << (constraint.sense == LESS ? " <= 0" : " >= 0");
    }
    inline std::ostream& operator<<(std::ostream& os, linear_range_constraint constraint) {
        return os << constraint.lower_bound << " <= " << constraint.linear_expression << " <= " << constraint.upper_bound;
    }
    inline std::ostream& operator<<(std::ostream& os, quadratic_ineq_constraint constraint) {
        return os << constraint.quadratic_expression << (constraint.sense == LESS ? " <= 0" : " >= 0");
    }
} //namespace SolverBuilder_Utils

#endif //PRINT_SOLVER_BUILDER_HPP