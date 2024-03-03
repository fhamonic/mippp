# ################### Packages ###################
find_package(GTest)
include(GoogleTest)

# ################# TEST targets #################
add_executable(
    mippp_test
    main_test.cpp variable_test.cpp variable_operators_test.cpp
    linear_term_operators_test.cpp linear_constraint_operators_test.cpp)
target_link_libraries(mippp_test mippp)
target_link_libraries(mippp_test GTest::gtest)
gtest_discover_tests(mippp_test)

if(${GUROBI_FOUND})
    add_executable(shared_gurobi_test main_test.cpp shared_grb_test.cpp)
    target_link_libraries(shared_gurobi_test mippp)
    target_link_libraries(shared_gurobi_test GTest::gtest)
    gtest_discover_tests(shared_gurobi_test)
endif()
# if(${CPLEX_FOUND})
#     add_executable(shared_cplex_test main_test.cpp shared_cplex_test.cpp)
#     target_link_libraries(shared_cplex_test mippp)
#     target_link_libraries(shared_cplex_test GTest::gtest)
#     gtest_discover_tests(shared_cplex_test)
# endif()
if(${SCIP_FOUND})
    add_executable(shared_scip_test main_test.cpp shared_scip_test.cpp)
    target_link_libraries(shared_scip_test mippp)
    target_link_libraries(shared_scip_test GTest::gtest)
    gtest_discover_tests(shared_scip_test)
endif()
if(${COINOR_FOUND})
    add_executable(shared_cbc_test main_test.cpp shared_cbc_test.cpp)
    target_link_libraries(shared_cbc_test mippp)
    target_link_libraries(shared_cbc_test GTest::gtest)
    gtest_discover_tests(shared_cbc_test)
endif()

add_executable(cli_cbc_test main_test.cpp cli_cbc_test.cpp)
target_link_libraries(cli_cbc_test mippp)
target_link_libraries(cli_cbc_test GTest::gtest)
gtest_discover_tests(cli_cbc_test)

add_executable(cli_grb_test main_test.cpp cli_grb_test.cpp)
target_link_libraries(cli_grb_test mippp)
target_link_libraries(cli_grb_test GTest::gtest)
gtest_discover_tests(cli_grb_test)

add_executable(cli_scip_test main_test.cpp cli_scip_test.cpp)
target_link_libraries(cli_scip_test mippp)
target_link_libraries(cli_scip_test GTest::gtest)
gtest_discover_tests(cli_scip_test)
