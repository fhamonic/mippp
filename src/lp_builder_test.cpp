#include <iostream>

#include "lp_builder.hpp"

int main() {
    LP_Builder builder;

    auto x_var = builder.addVars(11, [](int a, int b){ return a + b; });
    auto y_var = builder.addVars(10, [](int i){ return i; });
    
    std::cout << x_var(3, 4) << " " << y_var(4) << std::endl;

    auto lhs = builder.addIneqConstr();
    lhs(x_var(3, 4), 2)(y_var(3)).less()(x_var(10, 0))(5.0);

    return EXIT_SUCCESS;
}