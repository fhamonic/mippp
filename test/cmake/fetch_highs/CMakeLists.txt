cmake_minimum_required(VERSION 3.23)
project(test_fetch_highs LANGUAGES CXX)

enable_testing()
add_executable(highs_probe probe.cpp)
target_compile_features(highs_probe PRIVATE cxx_std_23)
target_include_directories(highs_probe PRIVATE ../../../include)
target_link_libraries(highs_probe PRIVATE ${CMAKE_DL_LIBS})
if(MSVC)
    target_compile_definitions(highs_probe PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()

# Deliberately disagree with HiGHS' requirements. The helper must scope its
# options instead of changing the enclosing project's build configuration.
set(BUILD_SHARED_LIBS OFF)
set(BUILD_TESTING ON)
set(BUILD_EXAMPLES ON)
include(../../../cmake/TestHighs.cmake)
mippp_test_highs(highs_probe _highs_environment)
if(BUILD_SHARED_LIBS OR NOT BUILD_TESTING OR NOT BUILD_EXAMPLES)
    message(FATAL_ERROR "HiGHS options escaped into the parent project")
endif()

include(GoogleTest)
if(_highs_environment)
    target_compile_definitions(highs_probe PRIVATE EXPECT_HIGHS=1)
    string(REPLACE ";" "\\;" _highs_environment "${_highs_environment}")
    gtest_discover_tests(highs_probe PROPERTIES
        ENVIRONMENT "MIPPP_REQUIRED_SOLVERS=CLP"
        ENVIRONMENT_MODIFICATION "${_highs_environment}")
else()
    if(TARGET highs)
        message(FATAL_ERROR "HiGHS was added with fetching disabled")
    endif()
    gtest_discover_tests(highs_probe PROPERTIES
        ENVIRONMENT "MIPPP_REQUIRED_SOLVERS=CLP")
endif()
