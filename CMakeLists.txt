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

find_package(GUROBI)
# find_package(CPLEX)
# find_package(SCIP)
find_package(COIN)

# ############### Select Backend #################
if(NOT (${USE_GUROBI} OR ${USE_CPLEX} OR ${USE_SCIP} OR ${USE_CBC}))
    if(${GUROBI_FOUND})
        set(${USE_GUROBI} ON)
    elseif(${CPLEX_FOUND})
        set(${USE_CPLEX} ON)
    elseif(${SCIP_FOUND})
        set(${USE_SCIP} ON)
    elseif(${COIN_FOUND})
        set(${USE_CBC} ON)
    else()
        message(FATAL_ERROR "No solver backend available")
    endif()
endif()

if(${GUROBI_FOUND})
    add_library(gurobi_backend INTERFACE backends/mippp_gurobi_impl.cpp)
    target_include_directories(gurobi_backend INTERFACE ${GUROBI_INCLUDE_DIR})
    target_link_libraries(gurobi_backend INTERFACE ${GUROBI_LIBRARIES})
endif()
if(${CPLEX_FOUND})
    add_library(cplex_backend INTERFACE backends/mippp_cplex_impl.cpp)
    target_include_directories(cplex_backend INTERFACE ${CPLEX_INCLUDE_DIR})
    target_link_libraries(cplex_backend INTERFACE ${CPLEX_LIBRARIES})
endif()
if(${SCIP_FOUND})
    add_library(scip_backend INTERFACE backends/mippp_scip_impl.cpp)
    target_include_directories(scip_backend INTERFACE ${SCIP_INCLUDE_DIR})
    target_link_libraries(scip_backend INTERFACE ${SCIP_LIBRARIES})
endif()
if(${COIN_FOUND})
    add_library(cbc_backend INTERFACE backends/mippp_cbc_impl.cpp)
    target_include_directories(cbc_backend INTERFACE ${COIN_INCLUDE_DIR})
    target_link_libraries(cbc_backend INTERFACE ${COIN_LIBRARIES})
endif()

# ############## Interface target ################
add_library(mippp_interface INTERFACE)
target_include_directories(
    mippp_interface INTERFACE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(mippp_interface INTERFACE range-v3::range-v3)

# ############### Exported target ################
add_library(mippp INTERFACE)
target_link_libraries(mippp INTERFACE mippp_interface)

if(${USE_GUROBI})
target_link_libraries(mippp INTERFACE gurobi_backend)
elseif(${USE_CPLEX})
target_link_libraries(mippp INTERFACE cplex_backend)
elseif(${USE_SCIP})
target_link_libraries(mippp INTERFACE scip_backend)
elseif(${USE_CBC})
target_link_libraries(mippp INTERFACE cbc_backend)
endif()

# ################### TESTS ######################
if(ENABLE_TESTING)
    enable_testing()
    message("Building Tests.")
    set_project_warnings(mippp_interface)
    add_subdirectory(test)
endif()
