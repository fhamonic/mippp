# Examples for MIP++. Each subdirectory is a self-contained project (its own
# CMakeLists.txt and conanfile.py) that can be copied as the starting point of a
# new program; see the README. Build them all from the repository root with:
#
#     ENABLE_EXAMPLES=ON conan build . -of=build -b=missing -pr=<profile> -c tools.build:skip_test=true
#
# Every example loads its solver's shared library at runtime (see the README for
# how to make solver libraries discoverable).

foreach(_example simple_lp nqueens sudoku cutting_stock)
  add_subdirectory(${_example})
endforeach()

# The TSP example also needs the MELON graph library.
find_package(melon QUIET)
if(TARGET melon::melon)
  add_subdirectory(travelling_salesman_dfj)
else()
  message(STATUS "melon not found: skipping the travelling_salesman_dfj example")
endif()
