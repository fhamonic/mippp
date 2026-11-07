# Examples for MIP++. Each subdirectory is a self-contained project (its own
# CMakeLists.txt and conanfile.py) that can be copied as the starting point of a
# new program; see the README. Build them all from the repository root with:
#
#     ENABLE_EXAMPLES=ON conan build . -of=build -b=missing -pr=<profile> -c tools.build:skip_test=true
#
# Every example loads its solver's shared library at runtime (see the README for
# how to make solver libraries discoverable).

foreach(_example simple_lp nqueens sudoku infeasible_transportation
                 vehicle_routing_colgen)
  add_subdirectory(${_example})
endforeach()

# These examples also need the MELON graph library.
find_package(melon QUIET)
if(TARGET melon::melon)
  foreach(_example cutting_stock travelling_salesman_dfj
                   travelling_salesman_colgen)
    add_subdirectory(${_example})
  endforeach()
else()
  message(STATUS "melon not found: skipping the cutting_stock, "
                 "travelling_salesman_dfj and travelling_salesman_colgen "
                 "examples")
endif()
