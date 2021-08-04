#include <iostream>

#include "expression_builder.hpp"

using namespace Builder;

int main() {
    quadratic_ineq_constraint_lhs_easy_init constraint;
    auto constr = constraint(23.0)(0, 2.0)(1, 3.0)(1, 2, 3.0).less()(0, 2.0)(10.0).take_data();

    std::cout << constr << std::endl;

    return EXIT_SUCCESS;
}