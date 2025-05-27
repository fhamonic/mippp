#ifndef MODELS_HPP
#define MODELS_HPP

#include "mippp/model_concepts.hpp"

#include "mippp/solvers/cbc/all.hpp"
#include "mippp/solvers/clp/all.hpp"
#include "mippp/solvers/copt/all.hpp"
#include "mippp/solvers/cplex/all.hpp"
#include "mippp/solvers/glpk/all.hpp"
#include "mippp/solvers/gurobi/all.hpp"
#include "mippp/solvers/highs/all.hpp"
#include "mippp/solvers/mosek/all.hpp"
#include "mippp/solvers/scip/all.hpp"
#include "mippp/solvers/soplex/all.hpp"

namespace fhamonic::mippp {

#define MODEL_TEST_W_PATH(MODEL_TYPE, API_TYPE, PATH)            \
    struct MODEL_TYPE##_test {                                   \
        using model_type = MODEL_TYPE;                           \
        API_TYPE api{PATH};                                      \
        auto construct_model() const { return MODEL_TYPE(api); } \
    };
#define MODEL_TEST(MODEL_TYPE, API_TYPE) \
    MODEL_TEST_W_PATH(MODEL_TYPE, API_TYPE, "")

// clang-format off
////////////////////////////////////////////////////////////
MODEL_TEST(gurobi_lp, gurobi_api);
    has_readable_constraint_sense<gurobi_lp> &&
    has_readable_constraint_rhs<gurobi_lp> &&
    has_feasibility_tolerance<gurobi_lp>

////////////////////////////////////////////////////////////
MODEL_TEST(gurobi_milp, gurobi_api);
    has_readable_constraint_sense<gurobi_milp> &&
    has_readable_constraint_rhs<gurobi_milp> &&
    has_feasibility_tolerance<gurobi_milp>

////////////////////////////////////////////////////////////
MODEL_TEST(clp_lp, clp_api);
    has_readable_constraint_sense<clp_lp> &&
    has_readable_constraint_rhs<clp_lp> &&
    has_feasibility_tolerance<clp_lp>

////////////////////////////////////////////////////////////
MODEL_TEST(cbc_milp, cbc_api);
    has_readable_constraint_sense<cbc_milp> && 
    has_readable_constraint_rhs<cbc_milp> && 
    has_feasibility_tolerance<cbc_milp> &&
    has_optimality_tolerance<cbc_milp>

////////////////////////////////////////////////////////////
MODEL_TEST(glpk_lp, glpk_api);
    has_feasibility_tolerance<glpk_lp>

////////////////////////////////////////////////////////////
MODEL_TEST(glpk_milp, glpk_api);
    has_feasibility_tolerance<glpk_milp>

////////////////////////////////////////////////////////////
MODEL_TEST(scip_milp, scip_api);
    has_feasibility_tolerance<scip_milp> &&
    has_optimality_tolerance<scip_milp>


// clang-format on
}  // namespace fhamonic::mippp

#endif  // MODELS_HPP