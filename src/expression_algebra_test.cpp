#include <iostream>
#include <iomanip>

#include "expression_algebra.hpp"

using namespace Algebra;

int main() {
    auto constr = 1 + 2 * Var(0) * 2.0 + Var(1) * 3.0 >= Var(0) * 2.0 + 10.0;

    std::cout << constr << std::endl;

    return EXIT_SUCCESS;
}