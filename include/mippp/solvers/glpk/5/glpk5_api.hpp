#ifndef MIPPP_GLPK_5_API_HPP
#define MIPPP_GLPK_5_API_HPP

#include "dylib.hpp"

#include "glpk.h"

#define GLPK_FUNCTIONS(F)               \
    F(glp_create_prob, create_prob)     \
    F(glp_delete_prob, delete_prob)     \
    F(glp_set_obj_dir, set_obj_dir)     \
    F(glp_get_obj_dir, get_obj_dir)     \
    F(glp_add_cols, add_cols)           \
    F(glp_set_obj_coef, set_obj_coef)   \
    F(glp_get_obj_coef, get_obj_coef)   \
    F(glp_set_col_bnds, set_col_bnds)   \
    F(glp_get_col_lb, get_col_lb)       \
    F(glp_get_col_ub, get_col_ub)       \
    F(glp_set_col_kind, set_col_kind)   \
    F(glp_get_col_kind, get_col_kind)   \
    F(glp_set_col_name, set_col_name)   \
    F(glp_get_col_name, get_col_name)   \
    F(glp_add_rows, add_rows)           \
    F(glp_set_mat_row, set_mat_row)     \
    F(glp_set_mat_col, set_mat_col)     \
    F(glp_set_row_bnds, set_row_bnds)   \
    F(glp_get_row_lb, get_row_lb)       \
    F(glp_get_row_ub, get_row_ub)       \
    F(glp_set_row_name, set_row_name)   \
    F(glp_get_row_name, get_row_name)   \
    F(glp_get_row_type, get_row_type)   \
    F(glp_get_num_cols, get_num_cols)   \
    F(glp_get_num_rows, get_num_rows)   \
    F(glp_get_num_nz, get_num_nz)       \
    F(glp_set_row_stat, set_row_stat)   \
    F(glp_get_row_stat, get_row_stat)   \
    F(glp_set_col_stat, set_col_stat)   \
    F(glp_get_col_stat, get_col_stat)   \
    F(glp_simplex, simplex)             \
    F(glp_get_status, get_status)       \
    F(glp_get_prim_stat, get_prim_stat) \
    F(glp_get_dual_stat, get_dual_stat) \
    F(glp_get_obj_val, get_obj_val)     \
    F(glp_get_col_prim, get_col_prim)   \
    F(glp_get_row_dual, get_row_dual)

#define DECLARE_GLPK_FUN(FULL, SHORT)     \
    using SHORT##_fun_t = decltype(FULL); \
    SHORT##_fun_t * SHORT;
#define CONSTRUCT_GLPK_FUN(FULL, SHORT) \
    , SHORT(lib.get_function<SHORT##_fun_t>(#FULL))

namespace fhamonic {
namespace mippp {

class glpk5_api {
private:
    dylib lib;

public:
    GLPK_FUNCTIONS(DECLARE_GLPK_FUN)

public:
    inline glpk5_api(const char * lib_path = "", const char * lib_name = "glpk")
        : lib(lib_path, lib_name) GLPK_FUNCTIONS(CONSTRUCT_GLPK_FUN) {}
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_GLPK_5_API_HPP