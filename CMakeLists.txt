cmake_minimum_required(VERSION 3.12)

project(milppp VERSION 1.0
    DESCRIPTION "Library attempting to provide an elegant and efficient way for instantiating Mixed Integer Linear Programs."
    LANGUAGES CXX)

# set(CMAKE_VERBOSE_MAKEFILE ON)

#################### Options ####################
SET(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)

option(WARNINGS "" OFF)
option(COMPILE_FOR_NATIVE "" OFF)
option(COMPILE_WITH_LTO "" OFF)

#################### Modules ####################
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
include(GNUInstallDirs)
include(CMakeBuildTypeUtils)

#################### Packages ####################
include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup(TARGETS)

find_package(COIN REQUIRED)
find_package(GUROBI REQUIRED)

#################### Library ####################
add_library(milppp INTERFACE)
target_include_directories(milppp INTERFACE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(milppp INTERFACE CONAN_PKG::range-v3)

################### EXEC target ####################
if(${GUROBI_FOUND})
    add_executable(grb_milp_builder_test src/grb_milp_builder_test.cpp)
    target_link_libraries(grb_milp_builder_test milppp)
    target_include_directories(grb_milp_builder_test PUBLIC ${GUROBI_INCLUDE_DIR})
    target_link_libraries(grb_milp_builder_test ${GUROBI_LIBRARIES})
    target_compile_options_if_exists(grb_milp_builder_test PRIVATE ${WARNINGS} -Wall)
    target_compile_options_if_exists(grb_milp_builder_test PRIVATE ${COMPILE_FOR_NATIVE} -march=native)
    target_compile_options_if_exists(grb_milp_builder_test PRIVATE ${COMPILE_WITH_LTO} -flto)
endif()

if(${COIN_FOUND})
    add_executable(cbc_milp_builder_test src/cbc_milp_builder_test.cpp)
    target_link_libraries(cbc_milp_builder_test milppp)
    target_include_directories(cbc_milp_builder_test PUBLIC ${COIN_INCLUDE_DIR})
    target_link_libraries(cbc_milp_builder_test ${COIN_LIBRARIES})
    target_compile_options_if_exists(cbc_milp_builder_test PRIVATE ${WARNINGS} -Wall)
    target_compile_options_if_exists(cbc_milp_builder_test PRIVATE ${COMPILE_FOR_NATIVE} -march=native)
    target_compile_options_if_exists(cbc_milp_builder_test PRIVATE ${COMPILE_WITH_LTO} -flto)
endif()