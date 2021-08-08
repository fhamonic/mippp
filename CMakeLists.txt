cmake_minimum_required(VERSION 3.12)

project(MILP_Builder VERSION 1.0
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

#################### Packages ####################
add_library(milp_builder INTERFACE)
target_compile_features(milp_builder INTERFACE cxx_std_17)
target_include_directories(milp_builder INTERFACE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

################### EXEC target ####################
add_executable(milp_builder_test src/milp_builder_test.cpp)
target_link_libraries(milp_builder_test CONAN_PKG::range-v3)
target_include_directories(milp_builder_test PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_compile_options_if_exists(milp_builder_test PRIVATE ${WARNINGS} -Wall)
target_compile_options_if_exists(milp_builder_test PRIVATE ${COMPILE_FOR_NATIVE} -march=native)
target_compile_options_if_exists(milp_builder_test PRIVATE ${COMPILE_WITH_LTO} -flto)