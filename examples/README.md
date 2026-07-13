# MIP++ examples

Small, self-contained programs demonstrating the MIP++ modeling interface. They
depend only on `mippp` (no MELON, no GoogleTest) and are adapted from the
library's test suites and the
[mippp_nqueens](https://github.com/fhamonic/mippp_nqueens) benchmark.

| Example | Demonstrates | Default backend |
| --- | --- | --- |
| [`simple_lp.cpp`](simple_lp.cpp) | The minimal build/solve/read loop | HiGHS |
| [`nqueens.cpp`](nqueens.cpp) | Lambda-indexed variables, constraint families over ranges | HiGHS |
| [`sudoku.cpp`](sudoku.cpp) | Multi-dimensional lambda indexing, cartesian-product constraints | HiGHS |
| [`tsp_lazy_constraints.cpp`](tsp_lazy_constraints.cpp) | Branch-and-cut via the candidate-solution callback (lazy subtour elimination) | Gurobi |
| [`cutting_stock.cpp`](cutting_stock.cpp) | Column generation with dual values and `add_column` | HiGHS |

Every example selects its backend through two aliases at the top of the file,
e.g.

```cpp
using api_type = highs_api;
using milp_type = highs_milp;
```

Change them to target another solver (`gurobi_api`/`gurobi_milp`,
`scip_api`/`scip_milp`, `cplex_api`/`cplex_lp`, ...). Note that
`tsp_lazy_constraints` needs the candidate-solution callback, which is currently
validated on **Gurobi, CPLEX and COPT** only.

## Building

From the repository root:

```bash
cmake -S . -B build -DENABLE_EXAMPLES=ON
cmake --build build
```

The executables are named `example_<name>` (e.g. `example_nqueens`).

## Running

MIP++ loads each solver's C API at runtime, so the corresponding shared library
must be on your loader path when you run an example (e.g. `LD_LIBRARY_PATH` on
Linux). See the repository [CONTRIBUTING.md](../CONTRIBUTING.md) for details, or
set `MIPPP_<SOLVER>_LIBRARY` to point at a specific library file.

```bash
./build/examples/example_simple_lp
./build/examples/example_nqueens 8      # board size as an optional argument
```
