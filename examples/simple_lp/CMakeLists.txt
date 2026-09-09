cmake_minimum_required(VERSION 3.23)

project(simple_lp LANGUAGES CXX)

# Standalone builds look MIP++ up (Conan's CMakeDeps or an installed package);
# inside the mippp source tree (-DENABLE_EXAMPLES=ON) the target already exists.
if(NOT TARGET mippp::mippp)
  find_package(mippp 1.0 CONFIG REQUIRED)
endif()

add_executable(simple_lp main.cpp)
target_link_libraries(simple_lp PRIVATE mippp::mippp)
