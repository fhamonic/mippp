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
# find_package(CPLEX) find_package(SCIP)
find_package(COIN)

# ############## Interface target ################
add_library(mippp_interface INTERFACE)
target_include_directories(mippp_interface
                           INTERFACE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(mippp_interface INTERFACE range-v3::range-v3)

# ############### Exported target ################
add_library(mippp INTERFACE)
target_link_libraries(mippp INTERFACE mippp_interface)

if(${GUROBI_FOUND})
    add_library(gurobi_backend INTERFACE)
    target_include_directories(gurobi_backend INTERFACE ${GUROBI_INCLUDE_DIR})
    target_link_libraries(gurobi_backend INTERFACE ${GUROBI_LIBRARIES})

    target_link_libraries(mippp INTERFACE gurobi_backend)
    target_compile_definitions(mippp INTERFACE FOUND_GUROBI)
endif()
if(${CPLEX_FOUND})
    add_library(cplex_backend INTERFACE)
    target_include_directories(cplex_backend INTERFACE ${CPLEX_INCLUDE_DIR})
    target_link_libraries(cplex_backend INTERFACE ${CPLEX_LIBRARIES})

    target_link_libraries(mippp INTERFACE cplex_backend)
    target_compile_definitions(mippp INTERFACE FOUND_CPLEX)
endif()
if(${SCIP_FOUND})
    add_library(scip_backend INTERFACE)
    target_include_directories(scip_backend INTERFACE ${SCIP_INCLUDE_DIR})
    target_link_libraries(scip_backend INTERFACE ${SCIP_LIBRARIES})

    target_link_libraries(mippp INTERFACE scip_backend)
    target_compile_definitions(mippp INTERFACE FOUND_SCIP)
endif()
if(${COIN_FOUND})
    add_library(cbc_backend INTERFACE)
    target_include_directories(cbc_backend INTERFACE ${COIN_INCLUDE_DIR})
    target_link_libraries(cbc_backend INTERFACE ${COIN_LIBRARIES})

    target_link_libraries(mippp INTERFACE cbc_backend)
    target_compile_definitions(mippp INTERFACE FOUND_CBC)
endif()

# ################### TESTS ######################
if(ENABLE_TESTING)
    enable_testing()
    message("Building Tests.")
    set_project_warnings(mippp_interface)
    add_subdirectory(test)
endif()
