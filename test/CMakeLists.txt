# ################### Packages ###################
find_package(GTest)
include(GoogleTest)

# ################# TEST targets #################
add_executable(
    mippp_interface_test
    main_test.cpp variable_test.cpp variable_operators_test.cpp
    linear_term_operators_test.cpp linear_constraint_operators_test.cpp)
target_link_libraries(mippp_interface_test mippp_interface)
target_link_libraries(mippp_interface_test GTest::gtest)
gtest_discover_tests(mippp_interface_test)

if(${GUROBI_FOUND})
    add_executable(gurobi_test main_test.cpp grb_test.cpp)
    target_link_libraries(gurobi_test mippp)
    target_link_libraries(gurobi_test GTest::gtest)
    gtest_discover_tests(gurobi_test)
endif()
if(${SCIP_FOUND})
    add_executable(scip_test main_test.cpp scip_test.cpp)
    target_link_libraries(scip_test mippp)
    target_link_libraries(scip_test GTest::gtest)
    gtest_discover_tests(scip_test)
endif()
# if(${CPLEX_FOUND}) ...
if(${COINOR_FOUND})
    add_executable(cbc_test main_test.cpp cbc_test.cpp)
    target_link_libraries(cbc_test mippp)
    target_link_libraries(cbc_test GTest::gtest)
    gtest_discover_tests(cbc_test)
endif()
