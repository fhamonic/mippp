cmake_minimum_required(VERSION 3.23)

# The version number is parsed from include/mippp/version.hpp, its single
# source of truth (conanfile.py parses it too).
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/include/mippp/version.hpp"
     _mippp_version_hpp)
foreach(level MAJOR MINOR PATCH)
  if(NOT _mippp_version_hpp MATCHES "#define MIPPP_VERSION_${level} ([0-9]+)")
    message(FATAL_ERROR
            "Failed to parse MIPPP_VERSION_${level} from version.hpp")
  endif()
  set(MIPPP_VERSION_${level} ${CMAKE_MATCH_1})
endforeach()

project(
  mippp
  VERSION ${MIPPP_VERSION_MAJOR}.${MIPPP_VERSION_MINOR}.${MIPPP_VERSION_PATCH}
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

# ############### Exported target ################
add_library(mippp INTERFACE)
add_library(mippp::mippp ALIAS mippp)
target_compile_features(mippp INTERFACE cxx_std_23)
target_include_directories(
  mippp INTERFACE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
# dlopen lives in libdl on older glibc; empty elsewhere
target_link_libraries(mippp INTERFACE ${CMAKE_DL_LIBS})

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