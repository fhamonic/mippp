cmake_minimum_required(VERSION 3.12)

project(Solver_Builder VERSION 1.0
    DESCRIPTION "Library attempting to provide a nice API for at least GUROBI and Cbc solver"
    LANGUAGES CXX)

# set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR})

#################### Options ####################
SET(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CONAN_DISABLE_CHECK_COMPILER ON)

include(${CMAKE_CURRENT_SOURCE_DIR}/CMakeBuildTypeUtils.cmake)

option(WARNINGS "" OFF)
option(COMPILE_FOR_NATIVE "" OFF)
option(COMPILE_WITH_LTO "" OFF)
option(WITH_GUROBI "" OFF)

#################### Packages ####################
include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup(TARGETS)

find_package(LEMON REQUIRED)
find_package(COIN REQUIRED)
if(${WITH_GUROBI} STREQUAL "ON")
    find_package(GUROBI REQUIRED)
endif()

################### EXEC target ####################
add_executable(test src/test.cpp)
target_include_directories(test PUBLIC include)

target_link_libraries(test CONAN_PKG::fast-cpp-csv-parser)
target_link_libraries(test CONAN_PKG::tbb)
target_link_libraries(test CONAN_PKG::boost)
target_link_libraries(test CONAN_PKG::eigen)

target_link_libraries(test ${LEMON_LIBRARIES})
target_link_libraries(test ${COIN_LIBRARIES})
if(${WITH_GUROBI} STREQUAL "ON")
    target_include_directories(test PUBLIC ${GUROBI_INCLUDE_DIR})
    target_link_libraries(test ${GUROBI_LIBRARIES})
endif()

target_compile_options_if_exists(test PRIVATE ${WARNINGS} -Wall)
target_compile_options_if_exists(test PRIVATE ${COMPILE_FOR_NATIVE} -march=native)
target_compile_options_if_exists(test PRIVATE ${COMPILE_WITH_LTO} -flto)