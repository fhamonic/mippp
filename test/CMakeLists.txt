# ################### Packages ###################
find_package(GTest)
include(GoogleTest)

# ################# TEST targets #################
# if(${GUROBI_FOUND}) add_library(gurobi_test) target_link_libraries(gurobi_test
# gurobi_backend) target_link_libraries(gurobi_test GTest::gtest)
# gtest_discover_tests(gurobi_test) endif() if(${CPLEX_FOUND})
# add_library(cplex_test) target_link_libraries(cplex_test cplex_backend)
# target_link_libraries(cplex_test GTest::gtest)
# gtest_discover_tests(cplex_test) endif() if(${SCIP_FOUND})
# add_library(scip_test) target_link_libraries(scip_test scip_backend)
# target_link_libraries(scip_test GTest::gtest) gtest_discover_tests(scip_test)
# endif() if(${COIN_FOUND}) add_library(cbc_test) target_link_libraries(cbc_test
# cbc_backend) target_link_libraries(cbc_test GTest::gtest)
# gtest_discover_tests(cbc_test) endif()

add_executable(mippp_interface_test main_test.cpp variable_test.cpp variable_operators_test.cpp linear_term_operators_test.cpp )
target_link_libraries(mippp_interface_test mippp_interface)
target_link_libraries(mippp_interface_test GTest::gtest)
gtest_discover_tests(mippp_interface_test)
