#ifndef MIPPP_LINEAR_CONSTRAINTS_HPP
#define MIPPP_LINEAR_CONSTRAINTS_HPP

#include <numeric>
#include <ostream>
#include <range/v3/algorithm/for_each.hpp>

#include "mippp/linear_expression.hpp"

namespace fhamonic {
namespace mippp {

enum InequalitySense { LESS = -1, EQUAL = 0, GREATER = 1 };

struct LinearIneqConstraint {
    InequalitySense sense;
    LinearExpr linear_expression;
    LinearIneqConstraint() = default;
    LinearIneqConstraint(InequalitySense s, LinearExpr && e)
        : sense(s), linear_expression(std::move(e)){};
    LinearIneqConstraint & simplify() {
        linear_expression.simplify();
        return *this;
    }
};

struct LinearRangeConstraint {
    double lower_bound, upper_bound;
    LinearExpr linear_expression;
    LinearRangeConstraint()
        : lower_bound{std::numeric_limits<double>::min()}
        , upper_bound{std::numeric_limits<double>::max()} {}
};

inline std::ostream & operator<<(std::ostream & os,
                                 const LinearIneqConstraint & constraint) {
    return os << constraint.linear_expression
              << (constraint.sense == LESS ? " <= 0" : " >= 0");
}
inline std::ostream & operator<<(std::ostream & os,
                                 const LinearRangeConstraint & constraint) {
    return os << constraint.lower_bound
              << " <= " << constraint.linear_expression
              << " <= " << constraint.upper_bound;
}

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_LINEAR_CONSTRAINTS_HPP