#include <iostream>

#include "mippp_cbc.hpp"

using namespace fhamonic::mippp;

int main() {
    using MILP = MILP_Builder<CbcTraits>;
    MILP builder(MILP::OptSense::MAXIMIZE);

    auto x = builder.addVar();
    auto y = builder.addVar();

    builder.setObjCoef(y, 1);

    builder.addLessThanConstr().lhs(-x, y).rhs(1);
    builder.addLessThanConstr().lhs(3 * x, 2 * y).rhs(12);
    builder.addLessThanConstr().lhs(2 * x, 3 * y).rhs(12);

    std::cout << builder << std::endl;

    CbcModel model = builder.build();
    CbcMain0(model);

    model.setLogLevel(0);
    model.solver()->writeLp("cbc_traits_test.lp");
    model.branchAndBound();

    std::cout << model.getObjValue() << std::endl;

    const double * solution = model.getColSolution();
    for (Var v : builder.variables())
        std::cout << solution[v.id()] << std::endl;

    return EXIT_SUCCESS;
}