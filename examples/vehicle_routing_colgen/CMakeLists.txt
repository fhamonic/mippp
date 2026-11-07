cmake_minimum_required(VERSION 3.23)

project(vehicle_routing_colgen LANGUAGES CXX)

# Standalone builds look MIP++ up (Conan's CMakeDeps or an installed package);
# inside the mippp source tree (-DENABLE_EXAMPLES=ON) the target already exists.
if(NOT TARGET mippp::mippp)
  find_package(mippp 1.1 CONFIG REQUIRED)
endif()

add_executable(vehicle_routing_colgen main.cpp)
target_link_libraries(vehicle_routing_colgen PRIVATE mippp::mippp)
