#ifndef LINEAR_CONSTRAINTS_HPP
#define LINEAR_CONSTRAINTS_HPP

#include <numeric>
#include <ostream>

#include <range/v3/algorithm/for_each.hpp>

#include "constraints/constants.hpp"
#include "expressions/linear_expression.hpp"

struct LinearIneqConstraint {
    InequalitySense sense;
    LinearExpr linear_expression;
    LinearIneqConstraint() = default;
    LinearIneqConstraint(InequalitySense s, LinearExpr && e)
        : sense(s)
        , linear_expression(std::move(e)) {};
};

struct LinearRangeConstraint {
    double lower_bound, upper_bound;
    LinearExpr linear_expression;
    LinearRangeConstraint()
            : lower_bound{std::numeric_limits<double>::min()}
            , upper_bound{std::numeric_limits<double>::max()} {}
};


inline std::ostream& operator<<(std::ostream& os, 
                                const LinearIneqConstraint & constraint) {
    return os << constraint.linear_expression
              << (constraint.sense == LESS ? " <= 0" : " >= 0");
}
inline std::ostream& operator<<(std::ostream& os, 
                                const LinearRangeConstraint & constraint) {
    return os << constraint.lower_bound 
              << " <= " << constraint.linear_expression 
              << " <= " << constraint.upper_bound;
}

#endif //LINEAR_CONSTRAINTS_HPP