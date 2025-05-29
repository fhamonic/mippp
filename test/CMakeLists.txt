# ################### Packages ###################
find_package(GTest REQUIRED)
include(GoogleTest)

# ################# TEST targets #################
add_executable(
    mippp_test
    main.cpp
    model_variable.cpp
    linear_expression.cpp
    linear_constraint.cpp
    # bilinear_expression.cpp
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
    )
target_include_directories(mippp_test PUBLIC ${PROJECT_SOURCE_DIR}/test)
target_link_libraries(mippp_test mippp)
target_link_libraries(mippp_test GTest::gtest)
gtest_discover_tests(mippp_test)
