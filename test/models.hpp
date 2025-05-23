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
static_assert(
    lp_model<gurobi_lp> &&
    has_readable_objective<gurobi_lp> &&
    has_modifiable_objective<gurobi_lp> &&
    has_readable_variables_bounds<gurobi_lp> &&
    has_readable_constraint_sense<gurobi_lp> &&
    has_readable_constraint_rhs<gurobi_lp> &&
    has_lp_status<gurobi_lp> &&
    has_dual_solution<gurobi_lp> &&
    has_feasibility_tolerance<gurobi_lp>);

////////////////////////////////////////////////////////////
MODEL_TEST(gurobi_milp, gurobi_api);
static_assert(
    milp_model<gurobi_milp> &&
    has_readable_objective<gurobi_milp> &&
    has_modifiable_objective<gurobi_milp> &&
    has_readable_variables_bounds<gurobi_milp> &&
    has_readable_constraint_sense<gurobi_milp> &&
    has_readable_constraint_rhs<gurobi_milp> &&
    has_feasibility_tolerance<gurobi_milp>);

////////////////////////////////////////////////////////////
MODEL_TEST(clp_lp, clp_api);
static_assert(
    lp_model<clp_lp> &&
    has_readable_objective<clp_lp> &&
    has_modifiable_objective<clp_lp> &&
    has_readable_variables_bounds<clp_lp> &&
    has_modifiable_variables_bounds<clp_lp> &&
    has_readable_constraint_sense<clp_lp> &&
    has_readable_constraint_rhs<clp_lp> &&
    has_lp_status<clp_lp> &&
    has_dual_solution<clp_lp> &&
    has_feasibility_tolerance<clp_lp>);

////////////////////////////////////////////////////////////
MODEL_TEST(cbc_milp, cbc_api);
static_assert(
    milp_model<cbc_milp> && 
    has_readable_objective<cbc_milp> && 
    has_modifiable_objective<cbc_milp> && 
    has_readable_variables_bounds<cbc_milp> && 
    has_modifiable_variables_bounds<cbc_milp> && 
    has_readable_constraint_sense<cbc_milp> && 
    has_readable_constraint_rhs<cbc_milp> && 
    has_feasibility_tolerance<cbc_milp> &&
    has_optimality_tolerance<cbc_milp>);

////////////////////////////////////////////////////////////
MODEL_TEST(glpk_lp, glpk_api);
static_assert(
    lp_model<glpk_lp> &&
    has_readable_variables_bounds<glpk_lp> &&
    has_modifiable_variables_bounds<glpk_lp> &&
    has_lp_status<glpk_lp> &&
    has_dual_solution<glpk_lp> &&
    has_feasibility_tolerance<glpk_lp>);

////////////////////////////////////////////////////////////
MODEL_TEST(glpk_milp, glpk_api);
static_assert(
    milp_model<glpk_milp> &&
    has_readable_variables_bounds<glpk_milp> &&
    has_modifiable_variables_bounds<glpk_milp> &&
    has_feasibility_tolerance<glpk_milp>);

////////////////////////////////////////////////////////////
MODEL_TEST(highs_lp, highs_api);
static_assert(
    lp_model<highs_lp> &&
    has_modifiable_objective<highs_lp> &&
    has_readable_variables_bounds<highs_lp> &&
    has_modifiable_variables_bounds<highs_lp> &&
    has_lp_status<highs_lp> &&
    has_dual_solution<highs_lp>);

////////////////////////////////////////////////////////////
MODEL_TEST(highs_milp, highs_api);
static_assert(
    milp_model<highs_milp> &&
    has_modifiable_objective<highs_milp> &&
    has_readable_variables_bounds<highs_milp> &&
    has_modifiable_variables_bounds<highs_milp>);

////////////////////////////////////////////////////////////
MODEL_TEST(soplex_lp, soplex_api);
static_assert(
    lp_model<soplex_lp> &&
    has_dual_solution<soplex_lp>);

////////////////////////////////////////////////////////////
MODEL_TEST(scip_milp, scip_api);
static_assert(
    lp_model<scip_milp> &&
    milp_model<scip_milp> &&
    has_modifiable_objective<scip_milp> &&
    has_feasibility_tolerance<scip_milp> &&
    has_optimality_tolerance<scip_milp>);

////////////////////////////////////////////////////////////
MODEL_TEST(mosek_lp, mosek_api);
static_assert(
    lp_model<mosek_lp> &&
    has_readable_variables_bounds<mosek_lp> &&
    has_modifiable_variables_bounds<mosek_lp> &&
    has_dual_solution<mosek_lp>);

////////////////////////////////////////////////////////////
MODEL_TEST(mosek_milp, mosek_api);
static_assert(
    milp_model<mosek_milp> &&
    has_readable_variables_bounds<mosek_milp> &&
    has_modifiable_variables_bounds<mosek_milp>);

////////////////////////////////////////////////////////////
MODEL_TEST(cplex_lp, cplex_api);
static_assert(
    lp_model<cplex_lp> &&
    has_readable_variables_bounds<cplex_lp> &&
    has_modifiable_variables_bounds<cplex_lp>);

////////////////////////////////////////////////////////////
MODEL_TEST(cplex_milp, cplex_api);
static_assert(
    milp_model<cplex_milp> &&
    has_readable_variables_bounds<cplex_milp> &&
    has_modifiable_variables_bounds<cplex_milp>);

////////////////////////////////////////////////////////////
MODEL_TEST(copt_lp, copt_api);
static_assert(
    lp_model<copt_lp> &&
    has_readable_variables_bounds<copt_lp> &&
    has_modifiable_variables_bounds<copt_lp>);

////////////////////////////////////////////////////////////
MODEL_TEST(copt_milp, copt_api);
static_assert(
    milp_model<copt_milp> &&
    has_readable_variables_bounds<copt_milp> &&
    has_modifiable_variables_bounds<copt_milp>);

// clang-format on
}  // namespace fhamonic::mippp

#endif  // MODELS_HPP