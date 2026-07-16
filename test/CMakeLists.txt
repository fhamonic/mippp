# ################### Packages ###################
find_package(GTest REQUIRED)
include(GoogleTest)

find_package(melon REQUIRED)

# ################# TEST targets #################
set(MIPPP_TEST_ALL_SOLVER_SOURCES
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
if(TEST_SOURCE)
    string(TOLOWER ${TEST_SOURCE} TEST_SOURCE_LOWER)
    set(MIPPP_TEST_SOLVER_SOURCES solvers/${TEST_SOURCE_LOWER}.cpp)
else()
    set(MIPPP_TEST_SOLVER_SOURCES ${MIPPP_TEST_ALL_SOLVER_SOURCES})
endif()

add_executable(
    mippp_test
    main.cpp
    model_variable.cpp
    linear_expression.cpp
    linear_constraint.cpp
    quadratic_expression.cpp
    unordered_dense_map.cpp
    column_manager.cpp
    ${MIPPP_TEST_SOLVER_SOURCES}
    )
target_include_directories(mippp_test PUBLIC ${PROJECT_SOURCE_DIR}/test)
target_link_libraries(mippp_test mippp)
target_link_libraries(mippp_test GTest::gtest)
target_link_libraries(mippp_test melon::melon)
gtest_discover_tests(mippp_test)

