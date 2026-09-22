cmake_minimum_required(VERSION 3.23)

project(cutting_stock LANGUAGES CXX)

# Standalone builds look MIP++ up (Conan's CMakeDeps or an installed package);
# inside the mippp source tree (-DENABLE_EXAMPLES=ON) the target already exists.
if(NOT TARGET mippp::mippp)
  find_package(mippp 1.0 CONFIG REQUIRED)
endif()
# This example also uses the MELON graph library for solving unbounded knapsack.
if(NOT TARGET melon::melon)
  find_package(melon REQUIRED)
endif()

add_executable(cutting_stock main.cpp)
target_link_libraries(cutting_stock PRIVATE mippp::mippp melon::melon)
