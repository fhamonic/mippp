# Script mode, run by the POST_BUILD step of mippp_add_suite_tests(): lists the
# tests of TEST_EXECUTABLE and writes to CTEST_FILE the add_test() calls of one
# ctest test per test suite.

cmake_minimum_required(VERSION 3.23)

execute_process(
    COMMAND ${TEST_EXECUTOR} "${TEST_EXECUTABLE}" --gtest_list_tests
    WORKING_DIRECTORY "${TEST_WORKING_DIR}"
    TIMEOUT 60
    OUTPUT_VARIABLE listing
    RESULT_VARIABLE result)
if(NOT result EQUAL 0)
    message(FATAL_ERROR "Listing the tests of ${TEST_EXECUTABLE} failed "
                        "(${result}):\n${listing}")
endif()

# '*' and '?' are the only wildcards of a GoogleTest filter pattern
set(serial_regexes "")
string(REPLACE ":" ";" serial_patterns "${SERIAL_TESTS}")
foreach(pattern IN LISTS serial_patterns)
    string(REPLACE "." "\\." regex "${pattern}")
    string(REPLACE "*" ".*" regex "${regex}")
    string(REPLACE "?" "." regex "${regex}")
    list(APPEND serial_regexes "^${regex}$")
endforeach()

# A suite line ends with '.' and its tests follow, indented by two spaces. The
# "# TypeParam = ..." comments go first: a type may hold brackets, which would
# stop a CMake list from splitting at the line breaks.
string(REGEX REPLACE "#[^\n]*" "" listing "${listing}")
string(REPLACE "\r" "" listing "${listing}")
string(REPLACE "\n" ";" lines "${listing}")
set(suites "")
set(suite "")
foreach(line IN LISTS lines)
    if(line MATCHES "^  ([^ ]+)")
        list(APPEND "tests_of_${suite}" "${CMAKE_MATCH_1}")
    elseif(line MATCHES "^([^ ]+)\\. *$")
        set(suite "${CMAKE_MATCH_1}")
        list(APPEND suites "${suite}")
    endif()
endforeach()

set(script "")
macro(add_suite_test name filter serial)
    string(APPEND script "add_test([==[${name}]==]")
    foreach(word IN LISTS TEST_EXECUTOR)
        string(APPEND script " [==[${word}]==]")
    endforeach()
    string(APPEND script
           " [==[${TEST_EXECUTABLE}]==] [==[--gtest_filter=${filter}]==]"
           " [==[--mippp_skip_exit_code=${SKIP_EXIT_CODE}]==])\n"
           "set_tests_properties([==[${name}]==] PROPERTIES"
           " WORKING_DIRECTORY [==[${TEST_WORKING_DIR}]==]"
           " SKIP_RETURN_CODE ${SKIP_EXIT_CODE}")
    if(${serial})
        string(APPEND script " RUN_SERIAL TRUE")
    endif()
    string(APPEND script ")\n")
endmacro()

foreach(suite IN LISTS suites)
    set(parallel_tests "")
    set(serial_tests "")
    foreach(test IN LISTS "tests_of_${suite}")
        set(serial FALSE)
        foreach(regex IN LISTS serial_regexes)
            if("${suite}.${test}" MATCHES "${regex}")
                set(serial TRUE)
                break()
            endif()
        endforeach()
        if(serial)
            list(APPEND serial_tests "${suite}.${test}")
        else()
            list(APPEND parallel_tests "${suite}.${test}")
        endif()
    endforeach()
    if(NOT serial_tests)
        add_suite_test("${suite}" "${suite}.*" FALSE)
    elseif(NOT parallel_tests)
        add_suite_test("${suite}" "${suite}.*" TRUE)
    else()
        list(JOIN serial_tests ":" excluded)
        add_suite_test("${suite}" "${suite}.*-${excluded}" FALSE)
        foreach(test IN LISTS serial_tests)
            add_suite_test("${test}" "${test}" TRUE)
        endforeach()
    endif()
endforeach()

file(WRITE "${CTEST_FILE}" "${script}")
