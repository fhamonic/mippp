# ################### Packages ###################
find_package(GTest REQUIRED)
#find_package(TBB REQUIRED)
include(GoogleTest)

# ################# TEST targets #################
add_executable(
    mippp_test
    main_test.cpp variable_test.cpp
    variable_operators_test.cpp
    linear_expression_operators_test.cpp
    linear_constraint_operators_test.cpp
    grb_milp_model_test.cpp
    clp_lp_model_test.cpp
    cbc_milp_model_test.cpp
    models_test.cpp
    )
target_link_libraries(mippp_test mippp)
target_link_libraries(mippp_test GTest::gtest)
gtest_discover_tests(mippp_test)
