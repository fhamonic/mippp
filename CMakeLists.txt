cmake_minimum_required(VERSION 3.12)

project(
    mippp
    VERSION 1.0
    DESCRIPTION
        "Library attempting to provide an elegant and efficient way for instantiating Mixed Integer Linear Programs."
    LANGUAGES CXX)

# set(CMAKE_VERBOSE_MAKEFILE ON)

# ################### Options ####################
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(ENABLE_TESTING "Enable Test Builds" OFF)

# ################### Modules ####################
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_BINARY_DIR}" ${CMAKE_MODULE_PATH})
include(GNUInstallDirs)
include(CompilerWarnings)
include(Conan)
run_conan()

# ################### Packages ###################
find_package(range-v3)

find_package(GUROBI REQUIRED)
find_package(CPLEX REQUIRED)
find_package(SCIP REQUIRED)
find_package(COIN REQUIRED)

# ############### Select Backend #################
set(IS_SOLVER_PREFERENCE
    (${USE_GUROBI}
    OR
    ${USE_CPLEX}
    OR
    ${USE_SCIP}
    OR
    ${USE_COIN})

if(NOT IS_SOLVER_PREFERENCE)
    if(${GUROBI_FOUND})
        set(${USE_GUROBI} ON)
    elseif(${CPLEX_FOUND})
        set(${USE_CPLEX} ON)
    elseif(${SCIP_FOUND})
        set(${USE_SCIP} ON)
    elseif(${COIN_FOUND})
        set(${USE_COIN} ON)
    else()
        message(FATAL_ERROR "No solver backend available")
    endif()
endif()

if(${USE_GUROBI})
    add_library(solver_backend PUBLIC mippp_gurobi_impl.cpp)
    target_include_directories(solver_backend PUBLIC ${GUROBI_INCLUDE_DIR})
    target_link_libraries(solver_backend PUBLIC ${GUROBI_LIBRARIES})
elseif(${USE_CPLEX})
    add_library(solver_backend PUBLIC mippp_cplex_impl.cpp)
    target_include_directories(solver_backend PUBLIC ${CPLEX_INCLUDE_DIR})
    target_link_libraries(solver_backend PUBLIC ${CPLEX_LIBRARIES})
elseif(${USE_SCIP})
    add_library(solver_backend PUBLIC mippp_scip_impl.cpp)
    target_include_directories(solver_backend PUBLIC ${SCIP_INCLUDE_DIR})
    target_link_libraries(solver_backend PUBLIC ${SCIP_LIBRARIES})
elseif(${USE_COIN})
    add_library(solver_backend PUBLIC mippp_coinor_impl.cpp)
    target_include_directories(solver_backend PUBLIC ${COIN_INCLUDE_DIR})
    target_link_libraries(solver_backend PUBLIC ${COIN_LIBRARIES})
endif()

# ################### Library ####################
add_library(mippp INTERFACE)
target_include_directories(
    mippp INTERFACE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(mippp INTERFACE range-v3::range-v3)
target_link_libraries(mippp INTERFACE solver_backend)

# ################### TESTS ######################
if(ENABLE_TESTING)
    enable_testing()
    message("Building Tests.")
    set_project_warnings(melon)
    add_subdirectory(test)
endif()
