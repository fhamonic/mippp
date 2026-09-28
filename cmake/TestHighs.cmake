include_guard(GLOBAL)

option(MIPPP_TEST_FETCH_HIGHS
       "Fetch and build a pinned HiGHS shared library for the tests" OFF)

# Return CTest environment modifications. HiGHS is loaded at runtime, never
# linked to the test executable or exported as a MIP++ dependency.
function(mippp_test_highs target environment)
    set(${environment} "" PARENT_SCOPE)
    if(NOT MIPPP_TEST_FETCH_HIGHS)
        return()
    endif()

    if(NOT "$ENV{MIPPP_HIGHS_LIBRARY}" STREQUAL "")
        # An explicit installation always wins, including one whose loading
        # fails. Do not conceal a broken installation with a fetched version.
        set(library "$ENV{MIPPP_HIGHS_LIBRARY}")
        message(STATUS "HiGHS tests use MIPPP_HIGHS_LIBRARY=${library}")
    else()
        include(FetchContent)
        # Function scope keeps HiGHS options out of the caller's settings.
        set(FAST_BUILD ON)
        set(BUILD_SHARED_LIBS ON)
        set(BUILD_CXX ON)
        set(BUILD_CXX_EXE OFF)
        set(BUILD_TESTING OFF)
        set(BUILD_EXAMPLES OFF)
        set(BUILD_EXTRA_UNIT_TESTS OFF)
        set(FORTRAN OFF)
        set(CSHARP OFF)
        set(PYTHON_BUILD_SETUP OFF)
        set(ZLIB OFF)
        set(HIPO OFF)
        set(CUPDLP_GPU OFF)
        set(HIGHSINT64 OFF)

        # HiGHS 1.12.0, within the wrapper's validated range. Populate first
        # so add_subdirectory can exclude its targets and install rules from
        # the parent project (also on CMake 3.23).
        FetchContent_Declare(mippp_test_highs
            GIT_REPOSITORY https://github.com/ERGO-Code/HiGHS.git
            GIT_TAG 755a8e027a99a8d4ecf153a8dde4b2a767cdf384
            SOURCE_SUBDIR mippp-populate-only)
        FetchContent_MakeAvailable(mippp_test_highs)
        add_subdirectory("${mippp_test_highs_SOURCE_DIR}"
                         "${mippp_test_highs_BINARY_DIR}" EXCLUDE_FROM_ALL)
        add_dependencies(${target} highs)
        set(library "$<TARGET_FILE:highs>")
    endif()

    # cmake_list_append preserves any other required solvers supplied by CI.
    # TARGET_FILE selects the DLL/dylib/so and the active configuration.
    set(${environment}
        "MIPPP_HIGHS_LIBRARY=set:${library}"
        "MIPPP_REQUIRED_SOLVERS=cmake_list_append:HIGHS"
        PARENT_SCOPE)
endfunction()
