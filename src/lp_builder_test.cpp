#include <iostream>

#include "lp_builder.hpp"

int main() {
    LP_Builder builder(LP_Builder::MAXIMIZE);

    auto x_var = builder.addVars(11, [](int a, int b){ return a + b; }, 1);
    auto y_var = builder.addVars(10, [](int i){ return i; });
    
    std::cout << x_var(3, 4).id() << " " << y_var(4).id() << std::endl;

    auto c = builder.addLessThanConstr();
    c.lhs( 6 , x_var(3, 4)*2 , y_var(3) )
     .rhs( 1*x_var(10, 0)*1 , 5.0 );

    builder.addRangeConstr(0, 25)( x_var(1,1) * 5 );

    std::cout << builder << std::endl;

    return EXIT_SUCCESS;
}