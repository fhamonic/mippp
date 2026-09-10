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
    mapping.cpp
    concat_view.cpp
    zero.cpp
    dynamic_library.cpp
    ${MIPPP_TEST_SOLVER_SOURCES}
    )
target_include_directories(mippp_test PUBLIC ${PROJECT_SOURCE_DIR}/test)
target_link_libraries(mippp_test mippp)
target_link_libraries(mippp_test GTest::gmock)
target_link_libraries(mippp_test melon::melon)

# shared libraries opened at runtime by dynamic_library.cpp, never linked: the
# same fixture built twice under one file name in two directories, standing in
# for two versions of a solver
add_library(mippp_test_library SHARED dynamic_library_fixture.cpp)
add_library(mippp_test_library_b SHARED dynamic_library_fixture.cpp)
target_compile_definitions(mippp_test_library_b PRIVATE MIPPP_TEST_ANSWER=43)
set_target_properties(
    mippp_test_library_b
    PROPERTIES OUTPUT_NAME mippp_test_library
               LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/variant_b
               RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/variant_b
               ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/variant_b)
if(WIN32)
    # match dynamic_library::prefix, which MinGW's default "lib" would not
    set_target_properties(mippp_test_library mippp_test_library_b
                          PROPERTIES PREFIX "")
endif()
target_compile_definitions(
    mippp_test
    PRIVATE MIPPP_TEST_LIBRARY="$<TARGET_FILE:mippp_test_library>"
            MIPPP_TEST_LIBRARY_B="$<TARGET_FILE:mippp_test_library_b>")
add_dependencies(mippp_test mippp_test_library mippp_test_library_b)
gtest_discover_tests(mippp_test)

