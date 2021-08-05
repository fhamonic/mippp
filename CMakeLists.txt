cmake_minimum_required(VERSION 3.12)

project(Solver_Builder VERSION 1.0
    DESCRIPTION "Library attempting to provide a nice API for at least GUROBI and Cbc solver"
    LANGUAGES CXX)

# set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR})

#################### Options ####################
SET(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)

include(${CMAKE_CURRENT_SOURCE_DIR}/CMakeBuildTypeUtils.cmake)

option(WARNINGS "" OFF)
option(COMPILE_FOR_NATIVE "" OFF)
option(COMPILE_WITH_LTO "" OFF)
option(WITH_GUROBI "" OFF)

#################### Packages ####################
include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup(TARGETS)


################### EXEC target ####################
add_executable(builder_test src/expression_builder_test.cpp)
target_include_directories(builder_test PUBLIC include)
target_link_libraries(builder_test CONAN_PKG::range-v3)
target_compile_options_if_exists(builder_test PRIVATE ${WARNINGS} -Wall)
target_compile_options_if_exists(builder_test PRIVATE ${COMPILE_FOR_NATIVE} -march=native)
target_compile_options_if_exists(builder_test PRIVATE ${COMPILE_WITH_LTO} -flto)

add_executable(algebra_test src/expression_algebra_test.cpp)
target_include_directories(algebra_test PUBLIC include)
target_link_libraries(algebra_test CONAN_PKG::range-v3)
target_compile_options_if_exists(algebra_test PRIVATE ${WARNINGS} -Wall)
target_compile_options_if_exists(algebra_test PRIVATE ${COMPILE_FOR_NATIVE} -march=native)
target_compile_options_if_exists(algebra_test PRIVATE ${COMPILE_WITH_LTO} -flto)


add_executable(lp_builder_test src/lp_builder_test.cpp)
target_include_directories(lp_builder_test PUBLIC include)
target_link_libraries(lp_builder_test CONAN_PKG::range-v3)
target_compile_options_if_exists(lp_builder_test PRIVATE ${WARNINGS} -Wall)
target_compile_options_if_exists(lp_builder_test PRIVATE ${COMPILE_FOR_NATIVE} -march=native)
target_compile_options_if_exists(lp_builder_test PRIVATE ${COMPILE_WITH_LTO} -flto)