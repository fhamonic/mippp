cmake_minimum_required(VERSION 3.23)

project(travelling_salesman_colgen LANGUAGES CXX)

# Standalone builds look MIP++ up (Conan's CMakeDeps or an installed package);
# inside the mippp source tree (-DENABLE_EXAMPLES=ON) the target already exists.
if(NOT TARGET mippp::mippp)
  find_package(mippp 1.1 CONFIG REQUIRED)
endif()
# This example also uses the MELON graph library for minimum spanning trees.
if(NOT TARGET melon::melon)
  find_package(melon REQUIRED)
endif()

add_executable(travelling_salesman_colgen main.cpp)
target_link_libraries(travelling_salesman_colgen PRIVATE mippp::mippp
                                                         melon::melon)
