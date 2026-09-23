# ################### Packages ###################
find_package(GTest REQUIRED)
include(GoogleTest)

find_package(melon REQUIRED)

# ################# TEST targets #################
set(MIPPP_TEST_ALL_SOLVER_SOURCES
    solvers/cbc.cpp
    solvers/clp.cpp
    solvers/copt.cpp
    solvers/dumb.cpp
    solvers/cplex.cpp
    solvers/glpk.cpp
    solvers/gurobi.cpp
    solvers/highs.cpp
    solvers/mosek.cpp
    solvers/scip.cpp
    solvers/soplex.cpp
    solvers/xpress.cpp
    )
# TEST_SOURCE is a ';'-separated list of backends, case-insensitive: "highs" or
# "CLP;CBC". Each solver file instantiates every shared suite for every model
# type of its backend and costs about as much to compile as all the core tests
# together, so restricting the list is what keeps a targeted build short.
if(TEST_SOURCE)
    set(MIPPP_TEST_SOLVER_SOURCES "")
    foreach(_solver IN LISTS TEST_SOURCE)
        string(TOLOWER ${_solver} _solver)
        list(APPEND MIPPP_TEST_SOLVER_SOURCES solvers/${_solver}.cpp)
    endforeach()
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
    cartesian_product_view.cpp
    entity_range.cpp
    invoke_key.cpp
    zero.cpp
    status.cpp
    variant.cpp
    dynamic_library.cpp
    ${MIPPP_TEST_SOLVER_SOURCES}
    )
target_include_directories(mippp_test PUBLIC ${PROJECT_SOURCE_DIR}/test)
# the suites and the library assert(); keep them live in every configuration
if(MSVC)
    target_compile_options(mippp_test PRIVATE /UNDEBUG)
else()
    target_compile_options(mippp_test PRIVATE -UNDEBUG)
endif()
target_link_libraries(mippp_test mippp)
target_link_libraries(mippp_test GTest::gmock)
target_link_libraries(mippp_test melon::melon)
if(MSVC)
    # A solver file instantiates dozens of typed suites; without this the
    # object exceeds the default COFF section limit and dies with C1128.
    target_compile_options(mippp_test PRIVATE /bigobj)
    # The suites read MIPPP_REQUIRED_SOLVERS and the fuzzing seed through
    # std::getenv, which MSVC deprecates (C4996) in favour of its own
    # _dupenv_s.
    target_compile_definitions(mippp_test PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()

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

# ################# Sanitizers ###################
set(MIPPP_SANITIZE
    ""
    CACHE STRING
          "Sanitizers to build the tests with, e.g. \"address\" or
           \"address,undefined\" (GCC/Clang only)")
if(MIPPP_SANITIZE)
    target_compile_options(mippp_test PRIVATE -fsanitize=${MIPPP_SANITIZE}
                                              -fno-sanitize-recover=all)
    target_link_options(mippp_test PRIVATE -fsanitize=${MIPPP_SANITIZE})
    # UBSan only reports by default and lets the process exit 0, so a test
    # that trips it still passes; -fno-sanitize-recover turns the report into
    # a failure.
endif()

# The interruption tests time their solves against the wall clock: run next to
# the rest by a parallel ctest, a loaded machine stretches a solve past the
# limit plus its overshoot.
gtest_discover_tests(mippp_test TEST_LIST MIPPP_TESTS
    TEST_FILTER "-*/TimeLimitTest/*.interrupts_long_solve")
gtest_discover_tests(mippp_test TEST_LIST MIPPP_TIMED_TESTS
    TEST_FILTER "*/TimeLimitTest/*.interrupts_long_solve"
    PROPERTIES RUN_SERIAL TRUE)

# ########## Header self-containment check ###########
# One translation unit per public header, generated at configure time: each
# header must compile on its own, with nothing included before it. This is
# also what compiles every backend on every compiler of the CI matrix when a
# job builds the tests of one solver only (see TEST_SOURCE above).
file(
    GLOB_RECURSE MIPPP_PUBLIC_HEADERS
    RELATIVE ${PROJECT_SOURCE_DIR}/include
    CONFIGURE_DEPENDS ${PROJECT_SOURCE_DIR}/include/mippp/*.hpp)

set(MIPPP_HEADER_CHECK_SOURCES "")
foreach(header ${MIPPP_PUBLIC_HEADERS})
    string(REGEX REPLACE "[/.]" "_" header_slug ${header})
    set(header_check_source
        ${CMAKE_CURRENT_BINARY_DIR}/header_check/${header_slug}.cpp)
    file(GENERATE OUTPUT ${header_check_source}
         CONTENT "#include \"${header}\"\n")
    list(APPEND MIPPP_HEADER_CHECK_SOURCES ${header_check_source})
endforeach()

add_library(mippp_header_check OBJECT ${MIPPP_HEADER_CHECK_SOURCES})
target_link_libraries(mippp_header_check mippp)
add_dependencies(mippp_test mippp_header_check)
