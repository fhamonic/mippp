#include <iostream>

#include "milppp.hpp"
#include "milppp/solver_traits/grb_traits.hpp"

int main() {
    using MILP = MILP_Builder<GrbTraits>;
    MILP builder(MILP::OptSense::MAXIMIZE);

    auto x = builder.addVar();
    auto y = builder.addVar();
    
    builder.setObjCoef(y, 1);

    builder.addLessThanConstr().lhs( -x , y ).rhs( 1 );
    builder.addLessThanConstr().lhs( 3*x , 2*y ).rhs( 12 );
    builder.addLessThanConstr().lhs( 2*x , 3*y ).rhs( 12 );

    std::cout << builder << std::endl;

    auto model = builder.build();
    GRBoptimize(model.model);

    double obj;
    GRBgetdblattr(model.model, GRB_DBL_ATTR_OBJVAL, &obj);

    std::cout << obj << std::endl;

    double * solution = new double[builder.nbVars()];
    GRBgetdblattrarray(model.model, GRB_DBL_ATTR_X, 0, builder.nbVars(), solution);

    for(Var v : builder.variables())
        std::cout << solution[v.id()] << std::endl;

    return EXIT_SUCCESS;
}