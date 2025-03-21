#ifndef MIPPP_SOPLEX_API_HPP
#define MIPPP_SOPLEX_API_HPP

#include "dylib.hpp"

#include "glpk.h"

#define SOPLEX_FUNCTIONS(F)                              
    glp_create_prob
    glp_delete_prob
    
    glp_set_obj_dir
    glp_get_obj_dir

    glp_add_cols
    glp_set_obj_coef
    glp_set_col_bnds
    glp_set_col_name

    glp_get_obj_coef
    glp_get_col_lb
    glp_get_col_ub
    glp_get_col_name

    glp_add_rows
    glp_set_row_bnds
    glp_set_row_name

    glp_get_row_lb
    glp_get_row_ub
    glp_get_row_name

    glp_get_num_cols
    glp_get_num_rows
    glp_get_num_nz

    glp_simplex

    glp_get_obj_val
    glp_get_col_prim
    glp_get_row_dual
    

#define DECLARE_SOPLEX_FUN(FULL, SHORT)      \
    using SHORT##_fun_t = decltype(FULL); \
    SHORT##_fun_t * SHORT;
#define CONSTRUCT_SOPLEX_FUN(FULL, SHORT) \
    , SHORT(lib.get_function<SHORT##_fun_t>(#FULL))

namespace fhamonic {
namespace mippp {

class soplex_api {
private:
    dylib lib;

public:
    SOPLEX_FUNCTIONS(DECLARE_SOPLEX_FUN)

public:
    inline soplex_api(const char * lib_name = "soplexshared", const char * lib_path = "")
        : lib(lib_path, lib_name) SOPLEX_FUNCTIONS(CONSTRUCT_SOPLEX_FUN) {}
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_SOPLEX_API_HPP