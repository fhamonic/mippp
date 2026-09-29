cmake_minimum_required(VERSION 3.23)

project(infeasible_transportation LANGUAGES CXX)

# Standalone builds look MIP++ up (Conan's CMakeDeps or an installed package);
# inside the mippp source tree (-DENABLE_EXAMPLES=ON) the target already exists.
if(NOT TARGET mippp::mippp)
  find_package(mippp 1.0 CONFIG REQUIRED)
endif()

add_executable(infeasible_transportation main.cpp)
target_link_libraries(infeasible_transportation PRIVATE mippp::mippp)
