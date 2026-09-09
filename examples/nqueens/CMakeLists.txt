cmake_minimum_required(VERSION 3.23)

project(nqueens LANGUAGES CXX)

# Standalone builds look MIP++ up (Conan's CMakeDeps or an installed package);
# inside the mippp source tree (-DENABLE_EXAMPLES=ON) the target already exists.
if(NOT TARGET mippp::mippp)
  find_package(mippp 1.0 CONFIG REQUIRED)
endif()

add_executable(nqueens main.cpp)
target_link_libraries(nqueens PRIVATE mippp::mippp)
