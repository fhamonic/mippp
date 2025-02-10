#ifndef QUADRATIC_CONSTRAINTS_HPP
#define QUADRATIC_CONSTRAINTS_HPP

#include <numeric>
#include <ostream>

#include <range/v3/algorithm/for_each.hpp>

#include "constraints/constants.hpp"
#include "expressions/quadratic_expression.hpp"

struct QuadraticIneqConstraint {
    InequalitySense sense;
    QuadraticExpr quadratic_expression;
    QuadraticIneqConstraint() = default;
};

struct QuadraticRangeConstraint {
    double lower_bound, upper_bound;
    QuadraticExpr linear_expression_view;
    QuadraticRangeConstraint()
            : lower_bound{std::numeric_limits<double>::min()}
            , upper_bound{std::numeric_limits<double>::max()} {}
};


inline std::ostream& operator<<(std::ostream& os, 
                                const QuadraticIneqConstraint & constraint) {
    return os << constraint.quadratic_expression 
              << (constraint.sense == LESS ? " <= 0" : " >= 0");
}

#endif //QUADRATIC_CONSTRAINTS_HPP