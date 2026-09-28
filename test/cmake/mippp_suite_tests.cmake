# mippp_add_suite_tests(<target> [SERIAL_TESTS <filter>])
#
# Registers with ctest one test per GoogleTest test suite of <target>, listed
# after each build, where gtest_discover_tests registers one per test. Every
# ctest test is a process that pays the executable's startup, about half a
# second in the sanitized build: grouped by suite, the tests of a backend that
# cannot run skip in one process, and the tests of the others share one.
#
# A suite whose every test skipped exits with a dedicated code that ctest
# reports as skipped, and one where anything failed exits with 1. The tests
# matched by SERIAL_TESTS, a GoogleTest filter of '*' and '?' patterns joined
# by ':', are registered apart with RUN_SERIAL, one ctest test each, unless
# they make up their whole suite, which is then one serial ctest test.
function(mippp_add_suite_tests target)
    cmake_parse_arguments(PARSE_ARGV 1 arg "" "SERIAL_TESTS" "")
    set(executor "")
    if(CMAKE_CROSSCOMPILING)
        get_property(executor TARGET ${target}
                     PROPERTY CROSSCOMPILING_EMULATOR)
    endif()
    set(file_base "${CMAKE_CURRENT_BINARY_DIR}/${target}_suites")
    add_custom_command(
        TARGET ${target}
        POST_BUILD
        BYPRODUCTS "${file_base}_tests.cmake"
        COMMAND
            "${CMAKE_COMMAND}" -D "TEST_EXECUTABLE=$<TARGET_FILE:${target}>"
            -D "TEST_EXECUTOR=${executor}"
            -D "TEST_WORKING_DIR=${CMAKE_CURRENT_BINARY_DIR}"
            -D "SERIAL_TESTS=${arg_SERIAL_TESTS}" -D "SKIP_EXIT_CODE=77"
            -D "CTEST_FILE=${file_base}_tests.cmake" -P
            "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/mippp_list_suite_tests.cmake"
        VERBATIM)
    file(
        WRITE "${file_base}_include.cmake"
        "if(EXISTS \"${file_base}_tests.cmake\")\n"
        "  include(\"${file_base}_tests.cmake\")\n"
        "else()\n"
        "  add_test(${target}_NOT_BUILT ${target}_NOT_BUILT)\n"
        "endif()\n")
    set_property(DIRECTORY APPEND PROPERTY TEST_INCLUDE_FILES
                                           "${file_base}_include.cmake")
endfunction()
