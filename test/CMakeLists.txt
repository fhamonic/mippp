# ################### Packages ###################
find_package(GTest REQUIRED)
#find_package(TBB REQUIRED)
include(GoogleTest)

# ################# TEST targets #################
add_executable(
    mippp_test
    main_test.cpp variable_test.cpp variable_operators_test.cpp
    linear_expression_operators_test.cpp linear_constraint_operators_test.cpp)
target_link_libraries(mippp_test mippp)
target_link_libraries(mippp_test GTest::gtest)
gtest_discover_tests(mippp_test)

if(${LINK_GUROBI})
    add_executable(linked_gurobi_test main_test.cpp linked_grb_test.cpp)
    target_link_libraries(linked_gurobi_test mippp)
    target_link_libraries(linked_gurobi_test GTest::gtest)
    gtest_discover_tests(linked_gurobi_test)
endif()
if(${LINK_CPLEX})
    add_executable(linked_cplex_test main_test.cpp linked_cplex_test.cpp)
    target_link_libraries(linked_cplex_test mippp)
    target_link_libraries(linked_cplex_test GTest::gtest)
    gtest_discover_tests(linked_cplex_test)
endif()
if(${LINK_SCIP})
    add_executable(linked_scip_test main_test.cpp linked_scip_test.cpp)
    target_link_libraries(linked_scip_test mippp)
    target_link_libraries(linked_scip_test GTest::gtest) # onetbb::onetbb
    gtest_discover_tests(linked_scip_test)
endif()
if(${LINK_COINOR})
    add_executable(linked_cbc_test main_test.cpp linked_cbc_test.cpp)
    target_link_libraries(linked_cbc_test mippp)
    target_link_libraries(linked_cbc_test GTest::gtest)
    gtest_discover_tests(linked_cbc_test)
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
