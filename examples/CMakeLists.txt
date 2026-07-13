# Examples for MIP++. Enable from the top-level build with:
#
#     cmake -DENABLE_EXAMPLES=ON ...
#
# Each example is a standalone program linking the header-only `mippp` target
# and loading its solver's shared library at runtime (see the README for how to
# make solver libraries discoverable).

set(MIPPP_EXAMPLES
    simple_lp
    nqueens
    sudoku
    tsp_lazy_constraints
    cutting_stock)

foreach(_example ${MIPPP_EXAMPLES})
  add_executable(example_${_example} ${_example}.cpp)
  target_link_libraries(example_${_example} PRIVATE mippp)
endforeach()
