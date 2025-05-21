cmake_minimum_required(VERSION 3.12)

project(
  mippp
  VERSION 1.0
  DESCRIPTION
    "A modern interface for linear programming solvers using C++20 ranges and concepts."
  LANGUAGES CXX)

# set(CMAKE_VERBOSE_MAKEFILE ON)

# ################### Options ####################
option(ENABLE_TESTING "Enable Test Builds" OFF)

# ################### Modules ####################
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_BINARY_DIR}" ${CMAKE_MODULE_PATH})
include(GNUInstallDirs)
include(CompilerWarnings)

# ################### Packages ###################
find_package(range-v3 REQUIRED)
find_package(dylib REQUIRED)

# ############### Exported target ################
add_library(mippp INTERFACE)
target_include_directories(mippp
                           INTERFACE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(mippp INTERFACE range-v3::range-v3)
target_link_libraries(mippp INTERFACE dylib::dylib)

target_compile_options(mippp INTERFACE -fconcepts-diagnostics-depth=30)

# ################### TESTS ######################
if(${ENABLE_TESTING} AND NOT ${CMAKE_CROSSCOMPILING})
  enable_testing()
  message("Building Tests.")
  set_project_warnings(mippp)
  add_subdirectory(test)
endif()
