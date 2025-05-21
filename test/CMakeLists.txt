# ################### Packages ###################
find_package(GTest REQUIRED)
include(GoogleTest)

# ################# TEST targets #################
add_executable(
    mippp_test
    main.cpp
    variable.cpp
    linear_expression.cpp
    linear_constraint.cpp
    # bilinear_expression.cpp
    model_methods.cpp
    milp_examples.cpp
    )
target_link_libraries(mippp_test mippp)
target_link_libraries(mippp_test GTest::gtest)
gtest_discover_tests(mippp_test)
