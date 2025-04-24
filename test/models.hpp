#ifndef MODELS_HPP
#define MODELS_HPP

#include "mippp/model_concepts.hpp"

#include "mippp/solvers/cbc/all.hpp"
#include "mippp/solvers/clp/all.hpp"
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

////////////////////////////////////////////////////////////
MODEL_TEST(grb_lp, grb_api);
static_assert(lp_model<grb_lp>);
static_assert(has_readable_objective<grb_lp>);
static_assert(has_modifiable_objective<grb_lp>);
static_assert(has_readable_variables_bounds<grb_lp>);
// static_assert(has_readable_constraint_lhs<grb_lp>);
static_assert(has_readable_constraint_sense<grb_lp>);
static_assert(has_readable_constraint_rhs<grb_lp>);
// static_assert(has_readable_constraints<grb_lp>);
static_assert(has_lp_status<grb_lp>);
static_assert(has_dual_solution<grb_lp>);
static_assert(has_feasibility_tolerance<grb_lp>);

////////////////////////////////////////////////////////////
MODEL_TEST(grb_milp, grb_api);
static_assert(milp_model<grb_milp>);
static_assert(has_readable_objective<grb_milp>);
static_assert(has_modifiable_objective<grb_milp>);
static_assert(has_readable_variables_bounds<grb_milp>);
// static_assert(has_readable_constraint_lhs<grb_milp>);
static_assert(has_readable_constraint_sense<grb_milp>);
static_assert(has_readable_constraint_rhs<grb_milp>);
// static_assert(has_readable_constraints<grb_milp>);
static_assert(has_feasibility_tolerance<grb_milp>);

////////////////////////////////////////////////////////////
MODEL_TEST(clp_lp, clp_api);
static_assert(lp_model<clp_lp>);
static_assert(has_readable_objective<clp_lp>);
static_assert(has_modifiable_objective<clp_lp>);
static_assert(has_readable_variables_bounds<clp_lp>);
// static_assert(has_readable_constraint_lhs<clp_lp>);
static_assert(has_readable_constraint_sense<clp_lp>);
static_assert(has_readable_constraint_rhs<clp_lp>);
// static_assert(has_readable_constraints<clp_lp>);
static_assert(has_lp_status<clp_lp>);
static_assert(has_dual_solution<clp_lp>);
static_assert(has_feasibility_tolerance<clp_lp>);

////////////////////////////////////////////////////////////
MODEL_TEST(cbc_milp, cbc_api);
static_assert(milp_model<cbc_milp>);
static_assert(has_readable_objective<cbc_milp>);
static_assert(has_modifiable_objective<cbc_milp>);
static_assert(has_readable_variables_bounds<cbc_milp>);
// static_assert(has_readable_constraint_lhs<cbc_milp>);
static_assert(has_readable_constraint_sense<cbc_milp>);
static_assert(has_readable_constraint_rhs<cbc_milp>);
// static_assert(has_readable_constraints<cbc_milp>);
static_assert(has_feasibility_tolerance<cbc_milp>);

////////////////////////////////////////////////////////////
MODEL_TEST(glpk_lp, glpk_api);
static_assert(lp_model<glpk_lp>);
static_assert(has_readable_variables_bounds<glpk_lp>);
static_assert(has_lp_status<glpk_lp>);
static_assert(has_dual_solution<glpk_lp>);
static_assert(has_feasibility_tolerance<glpk_lp>);

////////////////////////////////////////////////////////////
MODEL_TEST(glpk_milp, glpk_api);
static_assert(milp_model<glpk_milp>);
static_assert(has_readable_variables_bounds<glpk_milp>);

////////////////////////////////////////////////////////////
MODEL_TEST_W_PATH(highs_lp, highs_api, "/usr/local/lib");
static_assert(lp_model<highs_lp>);
// static_assert(has_modifiable_objective<highs_lp>);
static_assert(has_lp_status<highs_lp>);
static_assert(has_dual_solution<highs_lp>);

////////////////////////////////////////////////////////////
MODEL_TEST_W_PATH(highs_milp, highs_api, "/usr/local/lib");
static_assert(milp_model<highs_milp>);
// static_assert(has_modifiable_objective<highs_milp>);

////////////////////////////////////////////////////////////
MODEL_TEST(soplex_lp, soplex_api);
static_assert(lp_model<soplex_lp>);
static_assert(has_dual_solution<soplex_lp>);

////////////////////////////////////////////////////////////
MODEL_TEST(scip_milp, scip_api);
static_assert(lp_model<scip_milp>);
static_assert(milp_model<scip_milp>);
static_assert(has_modifiable_objective<scip_milp>);

////////////////////////////////////////////////////////////
MODEL_TEST_W_PATH(
    mosek_lp, mosek_api,
    "/home/plaiseek/Softwares/mosek/11.0/tools/platform/linux64x86/bin");
static_assert(lp_model<mosek_lp>);
static_assert(has_readable_variables_bounds<mosek_lp>);
static_assert(has_dual_solution<mosek_lp>);

// ////////////////////////////////////////////////////////////
// MODEL_TEST_W_PATH(mosek_milp, mosek_api,
// "/home/plaiseek/Softwares/mosek/11.0/tools/platform/linux64x86/bin");
// static_assert(milp_model<mosek_milp>);
// static_assert(has_dual_solution<mosek_milp>);

////////////////////////////////////////////////////////////
MODEL_TEST_W_PATH(
    cplex_lp, cplex_api,
    "/home/plaiseek/Softwares/cplex-community/cplex/bin/x86-64_linux");
static_assert(lp_model<cplex_lp>);
static_assert(has_readable_variables_bounds<cplex_lp>);
// static_assert(has_modifiable_objective<cplex_lp>);

////////////////////////////////////////////////////////////
MODEL_TEST_W_PATH(cplex_milp, cplex_api,
"/home/plaiseek/Softwares/cplex-community/cplex/bin/x86-64_linux");
static_assert(milp_model<cplex_milp>);
static_assert(has_readable_variables_bounds<cplex_milp>);
// static_assert(has_modifiable_objective<cplex_milp>);

}  // namespace fhamonic::mippp

#endif  // MODELS_HPP