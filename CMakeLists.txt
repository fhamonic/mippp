cmake_minimum_required(VERSION 3.23)

project(
  mippp
  VERSION 1.0.0
  DESCRIPTION
    "A modern interface for linear programming solvers using C++23 features."
  HOMEPAGE_URL "https://github.com/fhamonic/mippp"
  LANGUAGES CXX)

# ################### Options ####################
option(ENABLE_TESTING "Enable Test Builds" OFF)
option(ENABLE_EXAMPLES "Build Examples" OFF)

# ################### Modules ####################
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_BINARY_DIR}" ${CMAKE_MODULE_PATH})
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(CompilerWarnings)

# ################### Packages ###################
find_package(dylib REQUIRED)

# ############### Exported target ################
add_library(mippp INTERFACE)
add_library(mippp::mippp ALIAS mippp)
target_compile_features(mippp INTERFACE cxx_std_23)
target_include_directories(
  mippp INTERFACE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(mippp INTERFACE dylib::dylib)

# ################# Installation #################
install(
  TARGETS mippp
  EXPORT mipppTargets)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/mippp
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(
  EXPORT mipppTargets
  NAMESPACE mippp::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/mippp)

configure_package_config_file(
  ${PROJECT_SOURCE_DIR}/cmake/mipppConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/mipppConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/mippp)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/mipppConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
  ARCH_INDEPENDENT)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/mipppConfig.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/mipppConfigVersion.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/mippp)

# ################### TESTS ######################
if(${ENABLE_TESTING} AND NOT ${CMAKE_CROSSCOMPILING})
  enable_testing()
  message("Building Tests.")
  set_project_warnings(mippp)
  add_subdirectory(test)
endif()

# ################## EXAMPLES ####################
if(${ENABLE_EXAMPLES} AND NOT ${CMAKE_CROSSCOMPILING})
  message("Building Examples.")
  add_subdirectory(examples)
endif()