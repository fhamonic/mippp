# ################### Packages ###################
find_package(GTest REQUIRED)
include(GoogleTest)

find_package(fmt REQUIRED)
find_package(melon REQUIRED)

# ################# TEST targets #################
add_executable(
    mippp_test
    main.cpp
    model_variable.cpp
    linear_expression.cpp
    linear_constraint.cpp
    quadratic_expression.cpp
    solvers/cbc.cpp
    solvers/clp.cpp
    solvers/copt.cpp
    solvers/cplex.cpp
    solvers/glpk.cpp
    solvers/gurobi.cpp
    solvers/highs.cpp
    solvers/mosek.cpp
    solvers/scip.cpp
    solvers/soplex.cpp
    solvers/xpress.cpp
    )
target_include_directories(mippp_test PUBLIC ${PROJECT_SOURCE_DIR}/test)
target_link_libraries(mippp_test mippp)
target_link_libraries(mippp_test GTest::gtest)
target_link_libraries(mippp_test fmt::fmt)
target_link_libraries(mippp_test melon::melon)
gtest_discover_tests(mippp_test)
